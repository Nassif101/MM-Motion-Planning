using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using MotionPlanningSim.Visualization;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    public enum ViewerCamera
    {
        Orbit,
        Chase,
        Payload,
        Free,
    }

    /// <summary>
    /// The viewer's composition root (design 5, 6.3-6.6). It owns the stream client, the viewer state and the view
    /// journal; once per frame it drains the client, poses the robot and scenario objects, and turns hotkeys into
    /// backend commands. It runs after the camera script so its camera choice wins. Command line: see
    /// <see cref="ViewerOptions"/> (connection, journal, quality, frame times and a scripted demo).
    /// </summary>
    [DefaultExecutionOrder(100)]
    public sealed partial class PlantViewerBootstrap : MonoBehaviour
    {
        const double DisplayLagS = 0.1;  // absorbs the measured arrival jitter (p99 gap 44 ms); user, 2026-11-20
        const int MaxFramesPerUpdate = 1024;
        const double JournalPoseHz = 10.0;

        [SerializeField] private PlantRobotView robot;
        [SerializeField] private PlantEnvironmentView environment;
        [SerializeField] private PlantViewerSite site;
        [SerializeField] private ThirdPersonRobotCamera robotCamera;
        [SerializeField] private FreeCamera freeCamera;
        [SerializeField] private PlantViewerHud hud;
        [SerializeField] private PlantViewerInput input;
        [SerializeField] private PlantMinimap minimap;
        [SerializeField] private PlantGoalPresenter goals;
        [SerializeField] private PlantRoutePresenter route;
        [SerializeField] private PlantPlanOverlay planOverlay;
        [SerializeField] private PlantTravelledPath travelled;
        [SerializeField] private PlantLidarOverlay lidar;
        [SerializeField] private PlantPredictionOverlay prediction;
        [SerializeField] private PlantLegend legend;
        [SerializeField] private PlantTelemetryPanel telemetry;
        [SerializeField] private PlantConsolePanel consolePanel;

        private readonly ViewerState state = new ViewerState();
        private readonly StatusMessage message = new StatusMessage(5.0);  // final review I1: messages expire
        private ViewerCommands commands;
        private StreamClient client;
        private JournalRecorder journal;
        private string journalFolder;
        private JObject appliedHello;
        private ViewerCamera view = ViewerCamera.Orbit;
        private double requestedSpeed = 1.0;
        private double lastSampleT = double.NaN;
        private double displayedT = double.NaN;  // no State shown yet
        private double lastPoseJournal;
        private float fps;
        private MapFrame map;
        private string mapError;
        private string goalsDisabled = "no session";
        private (double x, double y)? pressed;
        private (double x, double y)? lastCursor;
        private double lastCursorJournal, lastDragJournal;
        private ViewerOptions options;
        private string optionsError;
        private enum Demo { Off, WaitConnect, WaitSession, Done }
        private Demo demo = Demo.Off;
        private double connectedAt = double.NaN;
        private List<(double wall, double display)> frameTimes;
        private double runStartedAt = double.NaN;
        private bool frameTimesWritten;

        public ViewerState State => state;
        public ViewerCommands Commands => commands;
        public ViewerCamera View => view;
        /// <summary>The minimap panel's state; the overlay settings are its one source (4B-1 Task 3 review I3).</summary>
        public bool MinimapOpen => overlays[Panel.Minimap];

        /// <summary>The overlay and panel toggles and presets (4B-1); public for automation and the live checks.</summary>
        public OverlaySettings Overlays => overlays;

        private readonly OverlaySettings overlays = new OverlaySettings();
        private readonly PathHistory travelledPath = new PathHistory();

        /// <summary>The serialized references that are not set (scene contract).</summary>
        public IEnumerable<string> MissingReferences()
        {
            if (robot == null) yield return nameof(robot);
            if (environment == null) yield return nameof(environment);
            if (site == null) yield return nameof(site);
            if (robotCamera == null) yield return nameof(robotCamera);
            if (freeCamera == null) yield return nameof(freeCamera);
            if (hud == null) yield return nameof(hud);
            if (input == null) yield return nameof(input);
            if (minimap == null) yield return nameof(minimap);
            if (goals == null) yield return nameof(goals);
            if (route == null) yield return nameof(route);
            if (planOverlay == null) yield return nameof(planOverlay);
            if (travelled == null) yield return nameof(travelled);
            if (lidar == null) yield return nameof(lidar);
            if (prediction == null) yield return nameof(prediction);
            if (legend == null) yield return nameof(legend);
            if (telemetry == null) yield return nameof(telemetry);
            if (consolePanel == null) yield return nameof(consolePanel);
        }

        public void Configure(PlantRobotView robotView, PlantEnvironmentView scenario, PlantViewerSite viewerSite,
            ThirdPersonRobotCamera orbit, FreeCamera free, PlantViewerHud status, PlantViewerInput keys)
        {
            robot = robotView;
            environment = scenario;
            site = viewerSite;
            robotCamera = orbit;
            freeCamera = free;
            hud = status;
            input = keys;
        }

        public void ConfigureInspection(PlantPlanOverlay plan, PlantTravelledPath path, PlantLidarOverlay points, PlantLegend key)
        {
            planOverlay = plan;
            travelled = path;
            lidar = points;
            legend = key;
        }

        /// <summary>4B-2: B5's prediction, overlay 7.</summary>
        public void ConfigurePrediction(PlantPredictionOverlay b5) => prediction = b5;

        /// <summary>The overlay an overlay key toggles (keys 1-7), or null for any other action.</summary>
        public static Overlay? OverlayOf(ViewerAction action) =>
            action >= ViewerAction.Overlay1 && action <= ViewerAction.Overlay7 ? (Overlay)(action - ViewerAction.Overlay1) : (Overlay?)null;

        public void ConfigurePanels(PlantTelemetryPanel values, PlantConsolePanel log)
        {
            telemetry = values;
            consolePanel = log;
        }

        /// <summary>The diagnostic console's store (4B-1 design 7.3).</summary>
        public ConsoleLog Console => consoleLog;

        private readonly ConsoleLog consoleLog = new ConsoleLog();
        private bool vsyncLock30;
        private bool inUnityLog;
        private List<string> lastWarnings = new List<string>();
        private ConnectionStatus lastStatus = ConnectionStatus.Connecting;

        public void ConfigureGoals(PlantMinimap overlay, PlantGoalPresenter marker, PlantRoutePresenter line)
        {
            minimap = overlay;
            goals = marker;
            route = line;
        }

        /// <summary>The bundled routing map (design 6.5), copied by <c>PlantViewerBuild.CopyMap</c>.</summary>
        public static string MapPath =>
            Path.Combine(Application.streamingAssetsPath, "PlantViewer", "construction_site_v2.yaml");

        void Awake()
        {
            var missing = string.Join(", ", MissingReferences());
            if (missing.Length > 0) throw new InvalidOperationException("PlantViewerBootstrap is missing " + missing);
            try
            {
                options = ViewerOptions.Parse(System.Environment.GetCommandLineArgs());
            }
            catch (ArgumentException e)
            {
                optionsError = e.Message;  // shown on the HUD; the viewer runs with the defaults
                options = ViewerOptions.Parse(new[] { "" });
            }
            if (options.Quality != null)
            {
                var level = Array.IndexOf(QualitySettings.names, options.Quality);
                if (level >= 0) QualitySettings.SetQualityLevel(level, true);
                else optionsError = $"-quality: no level {options.Quality} ({string.Join(", ", QualitySettings.names)})";
            }
            if (options.FrameTimes != null) frameTimes = new List<(double, double)>(1 << 16);
            if (options.VsyncLock30) SetPace(true);  // the steady 30: every second refresh (4B-1 design 7.5)
            else if (options.Fps > 0)  // another cap replaces vsync
            {
                QualitySettings.vSyncCount = 0;
                Application.targetFrameRate = options.Fps;
            }
            Application.logMessageReceived += OnUnityLog;
            if (options.Scenario != null) demo = Demo.WaitConnect;
            requestedSpeed = options.Speed;
            overlays.Apply(options.StartPreset);  // -preset (4B-1); Off is the 4A picture
            if (options.Overlays != null)  // -overlays: exactly these overlays on (the Task 6 measurement)
                foreach (Overlay o in Enum.GetValues(typeof(Overlay)))
                    if (overlays[o] != System.Linq.Enumerable.Contains(options.Overlays, o)) overlays.Toggle(o);
            var root = options.JournalDir ??
                Path.Combine(System.Environment.GetFolderPath(System.Environment.SpecialFolder.Personal), "Movies", "PlantViewer");
            journalFolder = Path.Combine(root, DateTime.UtcNow.ToString("yyyyMMdd'T'HHmmss'Z'"));
            commands = new ViewerCommands();
            client = new StreamClient(options.Host, options.Port, commands.Hello);
            journal = new JournalRecorder(OpenJournal, () => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0);
            freeCamera.enabled = false;
            robotCamera.Recentered += OnRecentered;
            if (options.Replay != null) OpenReplay(options.Replay);  // spec 4C
        }

        void Start()
        {
            consolePanel.Log = consoleLog;
            consolePanel.ExportFolder = () => journalFolder;
            consolePanel.Exported = path =>
            {
                Note(Severity.Info, "viewer", "console exported: " + path);
                Journal("console_export", new JObject { ["path"] = path });
            };
            try
            {
                map = MapFrame.Load(MapPath);
                minimap.Initialize(map);
                StartCoroutine(RenderMinimap());
            }
            catch (Exception e)
            {
                mapError = "no map: " + e.Message;
            }
            if (!Replaying && replayError == null) client.Start();
        }

        /// <summary>The top-down render waits one frame for HDRP, with the robot and scenario objects hidden.</summary>
        IEnumerator RenderMinimap()
        {
            yield return null;
            var robotShown = robot.gameObject.activeSelf;
            var scenarioShown = environment.gameObject.activeSelf;
            robot.gameObject.SetActive(false);
            environment.gameObject.SetActive(false);
            try
            {
                minimap.RenderTopDown();
            }
            finally
            {
                robot.gameObject.SetActive(robotShown);
                environment.gameObject.SetActive(scenarioShown);
            }
        }

        void Update()
        {
            var wall = Time.realtimeSinceStartupAsDouble;
            consoleLog.BeginFrame();
            RobotSample sample;
            bool hasSample;
            if (Replaying)
            {
                hasSample = UpdateReplay(wall, out sample);
                if (!ReferenceEquals(state.SessionHello, appliedHello)) OnSession(state.SessionHello);
                while (state.Events.Count > 0) OnEvent(state.Events.Dequeue());
            }
            else
            {
                if (client.Status != lastStatus)
                {
                    lastStatus = client.Status;
                    Note(lastStatus == ConnectionStatus.Connected ? Severity.Info : Severity.Warning, "stream", "connection: " + lastStatus);
                }
                for (var n = 0; n < MaxFramesPerUpdate && client.TryDequeue(out var frame); ++n) Route(frame, wall);
                if (!ReferenceEquals(state.SessionHello, appliedHello)) OnSession(state.SessionHello);
                // No state yet (a reset or a new server): hide the robot rather than show a stale pose (live check).
                hasSample = state.TrySample(DisplayLagS, wall, out sample);
                // spec 4C: F2's plans and B5's predictions at the displayed time, not about 0.1 s ahead of it
                gate.Release(hasSample ? sample.T : state.NewestStateT, f => state.Apply(f, wall));
                while (state.Events.Count > 0) OnEvent(state.Events.Dequeue());
            }
            if (robot.gameObject.activeSelf != hasSample) robot.gameObject.SetActive(hasSample);
            if (hasSample)
            {
                var dt = double.IsNaN(lastSampleT) ? 0.0 : Math.Max(0.0, sample.T - lastSampleT);
                robot.Apply(sample, (float)dt);
                travelledPath.Add(sample.X, sample.Y);
                lastSampleT = sample.T;
                displayedT = sample.T;
            }
            environment.ApplyState(state);
            UpdateGoals(hasSample ? (sample.X, sample.Y, sample.Yaw) : ((double, double, double)?)null, wall);
            UpdateOverlays();

            foreach (var action in input.Actions) Perform(action);
            input.Actions.Clear();
            if (hud.PickedScenario != null)
            {
                SendReset(hud.PickedScenario, 1);
                hud.PickedScenario = null;
            }
            UpdateFreeCameraJournal(wall);  // spec 4C: moving poses at 10 Hz, then the resting pose marked still

            fps = Mathf.Lerp(fps, 1f / Mathf.Max(Time.unscaledDeltaTime, 1e-4f), 0.1f);
            if (frameTimes != null && frameTimes.Count < 4_000_000) frameTimes.Add((wall, displayedT));
            StepDemo(wall);
            UpdateHud(wall);
        }

        void OnSession(JObject hello)
        {
            appliedHello = hello;
            lastSampleT = double.NaN;
            displayedT = double.NaN;
            goalsDisabled = GoalsDisabledReason(hello);
            message.Clear();  // a new session: the old session's messages no longer apply
            if (hello == null) return;  // a new connection: wait for the server's replay
            environment.ApplySession(hello);
            if (!state.SessionIsReplay)  // a new session: the old path and points go (a reconnect's replay keeps them)
            {
                travelledPath.Clear();
                lidar.Clear();
            }
            // One journal per session; only the server's replay after a reconnect continues it (review I3).
            var files = journal.Files;
            journal.OnSession(hello, state.SessionIsReplay, ReplayIdentity());
            if (journal.Files != files)  // a new journal file starts with the whole picture (Task 5 review I5)
            {
                var snapshot = overlays.Snapshot();
                snapshot["vsync_lock_30"] = vsyncLock30;
                Journal("overlay_state", snapshot);
                JournalCamera();  // spec 4C: and the camera as it is
            }
            if (demo == Demo.WaitSession && !state.SessionIsReplay) StartDemo();
        }

        /// <summary>
        /// The scripted demo resets once, a second after connecting: the server's replay of an old session arrives
        /// before then, so the next session Hello answers this reset.
        /// </summary>
        void StepDemo(double wall)
        {
            if (demo != Demo.WaitConnect || Replaying) return;
            if (client.Status != ConnectionStatus.Connected)
            {
                connectedAt = double.NaN;
                return;
            }
            if (double.IsNaN(connectedAt)) connectedAt = wall;
            if (wall - connectedAt >= 1.0 && SendReset(options.Scenario, options.Seed)) demo = Demo.WaitSession;
        }

        void StartDemo()
        {
            demo = Demo.Done;
            Journal("demo", new JObject { ["scenario"] = options.Scenario, ["seed"] = options.Seed, ["speed"] = options.Speed });
            if (options.Minimap) overlays.Set(Panel.Minimap, true);
            if (options.Goal is { } g) SubmitGoal(g);
            if (Send(commands.Run(requestedSpeed), "run"))
            {
                SetRunning(true);
                runStartedAt = Time.realtimeSinceStartupAsDouble;
            }
        }

        /// <summary>Opens journal file n; never overwrites one (Task 5 review I1, M3).</summary>
        TextWriter OpenJournal(int n)
        {
            Directory.CreateDirectory(journalFolder);
            var path = Path.Combine(journalFolder, $"journal-{n}.jsonl");
            return new StreamWriter(new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.Read));
        }

        /// <summary>Why minimap goals are off for this session (design 6.5), or null when they are on.</summary>
        string GoalsDisabledReason(JObject hello)
        {
            if (hello == null) return "no session";
            if (!state.GeneratedRoutes) return "this server replays recorded routes (start it with --routes lattice)";
            if (map == null) return mapError ?? "no map";
            return map.MatchesFingerprint(hello["routing"] as JObject, out var reason) ? null : reason;
        }

        /// <summary>Minimap gestures to goals, the marker and the route; the minimap is journaled (design 6.6).</summary>
        void UpdateGoals((double x, double y, double yaw)? robotPose, double wall)
        {
            minimap.Open = MinimapOpen;
            minimap.GoalsEnabled = goalsDisabled == null;
            minimap.Note = map == null ? mapError ?? "no map" : goalsDisabled ?? "";
            minimap.Robot = robotPose ?? (state.TryRobotPose(out var start) ? start : ((double, double, double)?)null);
            minimap.Goal = state.Goal;
            minimap.Route = state.RoutePath;
            foreach (var (kind, x, y) in minimap.Gestures)
            {
                if (kind == "drag")
                {
                    if (wall - lastDragJournal < 1.0 / JournalPoseHz) continue;
                    lastDragJournal = wall;
                }
                Journal("minimap_" + kind, new JObject { ["x"] = x, ["y"] = y });
                if (kind == "press") pressed = (x, y);
                else if (kind == "release" && pressed is { } p)
                {
                    pressed = null;
                    var from = state.TryRobotPose(out var r) ? (r.x, r.y) : p;  // the start pose before any State
                    SubmitGoal(MinimapGesture.Target(p, (x, y), from));
                }
            }
            minimap.Gestures.Clear();
            if (MinimapOpen && minimap.Cursor is { } c && !c.Equals(lastCursor) && wall - lastCursorJournal >= 1.0 / JournalPoseHz)
            {
                lastCursor = c;
                lastCursorJournal = wall;
                Journal("minimap_cursor", new JObject { ["x"] = c.x, ["y"] = c.y }, thinnable: true);
            }
            goals.Present(state.Goal);
            route.Present(state, overlays[Overlay.Route]);
            minimap.PlanSamples = overlays[Overlay.PlanPath] && state.Plan != null ? state.Plan.Samples : null;
        }

        void SubmitGoal((double x, double y, double yaw) target)
        {
            var command = commands.SubmitGoal(target.x, target.y, target.yaw);
            Journal("goal_submitted", new JObject
            {
                ["target"] = new JArray(target.x, target.y, target.yaw), ["client_request"] = command["client_request"],
            });
            if (Send(command, "submit_goal"))
                state.Goal.NoteSubmitted((long)command["client_request"], target.x, target.y, target.yaw);
        }

        /// <summary>The 4B-1 overlays and the legend (design 7.1, 7.4); hidden overlays do no work.</summary>
        void UpdateOverlays()
        {
            var stale = state.PlanIsStale(state.NewestStateT);
            planOverlay.Present(state.Plan, stale, overlays[Overlay.PlanPath], overlays[Overlay.Ghosts], overlays[Overlay.Sweep]);
            travelled.Present(travelledPath, overlays[Overlay.Travelled]);
            lidar.Present(state, overlays[Overlay.Lidar] && state.HasLidar);
            var predictionStale = state.PredictionIsStale(state.NewestStateT);
            prediction.Present(state.Prediction, predictionStale, overlays[Overlay.Prediction] && state.HasPrediction);
            legend.Visible = overlays[Panel.Legend];
            if (!legend.Visible) return;
            legend.Title = $"Preset {overlays.Current}{(overlays.Modified ? " (modified)" : "")}: P presets, 1-7 overlays, L legend";
            legend.Rows.Clear();
            var plans = !state.HasMediumPlans ? "no data" : state.Plan == null ? "no plan" : stale ? "stale" : "";
            void Row(string key, string name, Color colour, Overlay o, string note) => legend.Rows.Add((key, name, colour, overlays[o], note));
            Row("1", "Route", new Color(0.3f, 0.95f, 0.95f), Overlay.Route, "");
            Row("2", "Travelled path", new Color(0.95f, 0.95f, 0.95f), Overlay.Travelled, "");
            Row("3", "F2 plan path", new Color(0.35f, 0.75f, 1f), Overlay.PlanPath, plans);
            Row("4", "F2 ghosts", new Color(0.35f, 0.75f, 1f, 0.6f), Overlay.Ghosts,
                plans != "" ? plans : state.Plan.HasDisplaySamples ? "at plan times" : "spaced by samples (approximate)");
            Row("5", "Panel sweep, planned", new Color(1f, 0.55f, 0.1f), Overlay.Sweep,
                plans != "" ? plans
                : state.Plan.HasDisplaySamples ? "through the plan every 0.1 s; not a collision envelope"
                : "approximate; an arm turn with the base still is joined straight");  // design 7.1, 4B-2
            Row("6", "Lidar points", PlantLidarOverlay.ColourOf(1), Overlay.Lidar,
                !state.HasLidar ? "no data"
                : $"{(lidar.Cloud.Exact ? "exact" : "approximate")} placement; skipped {lidar.Cloud.Skipped}");
            if (overlays[Overlay.Lidar] && state.HasLidar)  // the hit classes' colours (Task 4 review I3)
                foreach (var (hit, name) in new[] { (0, "site"), (1, "scenario box"), (2, "mover"), (4, "robot"), (5, "panel"), (6, "other") })
                    legend.Rows.Add(("", "    hits: " + name, PlantLidarOverlay.ColourOf(hit), true, ""));
            var p = state.Prediction;  // 4B-2: B5's prediction, executed or held
            Row("7", "B5 prediction, 2 s", PlantPredictionOverlay.Colour(false, false), Overlay.Prediction,
                !state.HasPrediction ? "no data" : p == null ? "no prediction"
                : state.ExecutorHolding ? "executor holds (B5 not solving)" : predictionStale ? "stale"
                : p.Held ? $"held ({p.Status}): not executed" : "executed");
            if (overlays[Overlay.Prediction] && state.HasPrediction)
                legend.Rows.Add(("", "    held: B5 holds or brakes", PlantPredictionOverlay.Colour(true, false), true, ""));
            // spec 4C (a 4B-2 deferred item): the grey of a plan or prediction more than 1 s old
            if (overlays[Overlay.PlanPath] || overlays[Overlay.Ghosts] || overlays[Overlay.Sweep] || overlays[Overlay.Prediction])
                legend.Rows.Add(("", "    stale: older than 1 s", PlantPredictionOverlay.Colour(false, true), true, ""));
        }

        void OnEvent(JObject e)
        {
            var (severity, subsystem, text) = ConsoleLog.FromEvent(e);
            consoleLog.Add(severity, subsystem, (double?)e["t"], Time.realtimeSinceStartupAsDouble, text);
            switch ((string)e["event"])
            {
                case "scenarios":
                    hud.Scenarios.Clear();
                    if (e["scenarios"] is JArray list)
                        foreach (var s in list) hud.Scenarios.Add(s.Type == JTokenType.Object ? (string)s["name"] : (string)s);
                    break;
                case "error":
                    message.Set("server: " + (string)e["reason"], Time.realtimeSinceStartupAsDouble);  // the event is in the console
                    break;
                case "goal_ack":
                    if ((string)e["action"] != "accepted" && (string)e["action"] != "applied" && (string)e["action"] != "scheduled")
                        message.Set($"{(string)e["command"]}: {(string)e["action"]} {(string)e["reason"]}", Time.realtimeSinceStartupAsDouble);
                    break;
            }
        }

        /// <summary>Performs one hotkey action, as if its key were pressed (also for automation and 4C).</summary>
        public void Perform(ViewerAction action)
        {
            Journal("action", new JObject { ["action"] = action.ToString() });
            if (PerformReplay(action)) return;
            switch (action)
            {
                case ViewerAction.RunPause:
                    if (state.Running) Pause();
                    else if (state.SessionHello == null) OpenPicker();  // the server runs nothing before a reset
                    else if (Send(commands.Run(requestedSpeed), "run")) SetRunning(true);
                    break;
                case ViewerAction.Step:
                    if (Send(commands.Step(), "step")) SetRunning(false);
                    break;
                case ViewerAction.SpeedDown:
                case ViewerAction.SpeedUp:
                    requestedSpeed = action == ViewerAction.SpeedUp ? SpeedLadder.Up(requestedSpeed) : SpeedLadder.Down(requestedSpeed);
                    if (state.Running && Send(commands.Run(requestedSpeed), "run"))
                        state.NoteSpeedChange(requestedSpeed, Time.realtimeSinceStartupAsDouble);
                    break;
                case ViewerAction.NextCamera:
                    SetCamera((ViewerCamera)(((int)view + 1) % 4));
                    break;
                case ViewerAction.ToggleMinimap:
                    overlays.Set(Panel.Minimap, !MinimapOpen);  // the 4A key: not a change of preset
                    // spec 4C: journaled as a toggle, so a render opens and closes it at the same time
                    Journal("panel", new JObject { ["toggle"] = Panel.Minimap.ToString(), ["on"] = MinimapOpen });
                    break;
                case ViewerAction.CancelGoal:
                    if (!state.Has("cancel_goal")) Say("cancel: this server has no cancel_goal");
                    else Send(commands.CancelGoal(), "cancel_goal");
                    break;
                case ViewerAction.Reset:
                    var hello = state.SessionHello;
                    if (hello == null) OpenPicker();
                    else if (SendReset((string)hello["scenario"], (int?)hello["seed"] ?? 1)) SetRunning(false);
                    break;
                case ViewerAction.ScenarioPicker:
                    if (hud.PickerOpen) hud.PickerOpen = false;
                    else OpenPicker();
                    break;
                case ViewerAction.Overlay1:
                case ViewerAction.Overlay2:
                case ViewerAction.Overlay3:
                case ViewerAction.Overlay4:
                case ViewerAction.Overlay5:
                case ViewerAction.Overlay6:
                case ViewerAction.Overlay7:
                    Journal("overlay", overlays.Toggle(OverlayOf(action).Value));
                    break;
                case ViewerAction.NextPreset:
                    Journal("preset", overlays.Apply(OverlaySettings.Next(overlays.Current)));
                    break;
                case ViewerAction.Telemetry:
                    Journal("panel", overlays.Toggle(Panel.Telemetry));
                    break;
                case ViewerAction.Console:
                    Journal("panel", overlays.Toggle(Panel.Console));
                    break;
                case ViewerAction.Legend:
                    Journal("panel", overlays.Toggle(Panel.Legend));
                    break;
                case ViewerAction.Hud:
                    Journal("panel", overlays.Toggle(Panel.Hud));
                    break;
                case ViewerAction.Pace:
                    SetPace(!vsyncLock30);
                    Journal("pace", new JObject { ["vsync_lock_30"] = vsyncLock30 });
                    break;
            }
        }

        /// <summary>A reset; its answer starts a new session and journal, never a replay (Task 6 review I3).</summary>
        bool SendReset(string scenario, int seed)
        {
            if (!Send(commands.Reset(scenario, seed), "reset")) return false;
            state.NoteResetSent();
            return true;
        }

        void OpenPicker()
        {
            hud.PickerOpen = true;
            Send(commands.ListScenarios(), "list_scenarios");
        }

        void Pause()
        {
            if (Send(commands.Pause(), "pause")) SetRunning(false);
        }

        void SetRunning(bool value)
        {
            var wall = Time.realtimeSinceStartupAsDouble;
            state.NoteRunning(value, wall);
            if (value) state.NoteSpeedChange(requestedSpeed, wall);  // the clock follows the requested speed at once
        }

        /// <summary>Orbit, chase and payload are the scene camera's own views; free swaps in the fly camera.</summary>
        void SetCamera(ViewerCamera next)
        {
            view = next;
            var free = next == ViewerCamera.Free;
            freeCamera.enabled = free;
            robotCamera.enabled = !free;
            if (!free)
                robotCamera.SetView(next == ViewerCamera.Orbit ? RobotCameraView.Orbit
                    : next == ViewerCamera.Chase ? RobotCameraView.RearLeftChase : RobotCameraView.PayloadFirstPerson);
            Journal("camera", new JObject { ["view"] = next.ToString() });
        }

        bool Send(JObject command, string what)
        {
            if (Replaying)
            {
                Say($"{what}: off in replay");
                return false;
            }
            if (client.Send(command)) return true;
            Say($"{what}: not connected");
            return false;
        }

        /// <summary>Journals one viewer action at the displayed simulation time (design 6.6).</summary>
        public void Journal(string kind, JObject fields = null, bool thinnable = false) =>
            journal?.Write(double.IsNaN(displayedT) ? 0.0 : displayedT, kind, fields, thinnable);  // a session starts at 0

        /// <summary>A viewer message: the status line for 5 s, and the console.</summary>
        void Say(string text)
        {
            message.Set(text, Time.realtimeSinceStartupAsDouble);
            Note(Severity.Warning, "viewer", text);
        }

        void Note(Severity severity, string subsystem, string text) =>
            consoleLog.Add(severity, subsystem, double.IsNaN(displayedT) ? (double?)null : displayedT, Time.realtimeSinceStartupAsDouble, text);

        /// <summary>Unity's warnings and errors into the console, guarded so the console never logs itself.</summary>
        void OnUnityLog(string text, string stackTrace, LogType type)
        {
            if (inUnityLog || type == LogType.Log) return;
            inUnityLog = true;
            try
            {
                var first = text.Split('\n')[0];
                Note(type == LogType.Warning ? Severity.Warning : Severity.Error, "viewer", "unity: " + first);
            }
            finally
            {
                inUnityLog = false;
            }
        }

        /// <summary>60 FPS with vsync, or the steady 30: vsync every second refresh (4B-1 design 7.5).</summary>
        void SetPace(bool lock30)
        {
            vsyncLock30 = lock30;
            QualitySettings.vSyncCount = lock30 ? 2 : 1;
            Application.targetFrameRate = -1;
        }

        /// <summary>The telemetry panel's lines (design 7.2); "—" for values not received.</summary>
        void BuildTelemetry(double wall)
        {
            const string none = TelemetryText.None;
            var lines = telemetry.Lines;
            lines.Clear();
            // spec 4C: the State at the displayed time, not the newest (about 0.1 s ahead)
            lines.AddRange(TelemetryText.B5(state.StateHeaderAt(displayedT) ?? state.LastStateHeader));  // contacts are a count (Task 5 review I1)
            var plan = state.Plan;
            lines.Add("F2" + (state.HasMediumPlans ? "" : " (not running)"));
            if (plan == null) lines.Add("  plan " + none);
            else
            {
                var age = state.NewestStateT - plan.ReceivedAtSimT;
                lines.Add($"  plan {plan.PlanId} {plan.Status}, homotopy {plan.Homotopy}, seed {plan.Seed}, age {(double.IsNaN(age) ? none : age.ToString("0.0") + " s")}");
                lines.Add($"  capped {plan.Capped}, waiting {plan.Waiting}, full multistart {plan.FullMultistart}, homotopy switched {plan.HomotopySwitched}");
                foreach (var (name, merit, feasible, progress, merged) in plan.Seeds)
                    lines.Add($"    {name,-22} merit {(double.IsNaN(merit) ? none : merit.ToString("0.###")),8}  {(feasible ? "feasible" : "infeasible")}  progress {progress:0.00} m{(merged ? "  merged" : "")}");
            }
            var p = state.Prediction;  // 4B-2
            if (!state.HasPrediction) lines.Add("B5 prediction (not sent)");
            else if (p == null) lines.Add("B5 prediction " + none);
            else
            {
                var end = p.States.Count > 0 ? p.States[p.States.Count - 1] : null;
                var age = state.NewestStateT - p.T;
                lines.Add($"B5 prediction {p.Status} ({(p.Held ? "held, not executed" : "executed")}), following F2 plan {(p.PlanId == 0 ? none : p.PlanId.ToString())}, age {(double.IsNaN(age) ? none : age.ToString("0.0") + " s")}");
                lines.Add(end == null ? "  horizon end " + none
                    : $"  horizon end t {p.HorizonEndT:0.0} s at ({end[0]:0.00}, {end[1]:0.00}) m, yaw {end[2]:0.00} rad");
            }
            var g = state.Goal;
            lines.Add($"Goal  id {(g.GoalId == 0 ? none : g.GoalId.ToString())}, {g.State}{(string.IsNullOrEmpty(g.Reason) ? "" : ": " + g.Reason)}, route {(state.RouteId == 0 ? none : state.RouteId.ToString())}");
            lines.Add("Viewer (measured here, not planner metrics)");
            if (Replaying)
            {
                lines.Add($"  replay {replayLog.Name}{(replayLog.EndsEarly ? " (ends early)" : "")}, {fps:0} FPS, frame {Time.unscaledDeltaTime * 1000f:0.0} ms");
                return;
            }
            lines.Add($"  {client.Status}, achieved {state.AchievedSpeed(wall):0.00}x, stream age {(double.IsInfinity(state.SilentFor(wall)) ? none : state.SilentFor(wall).ToString("0.00") + " s")}");
            lines.Add($"  dropped {client.Dropped}, malformed {client.Malformed}, {fps:0} FPS, frame {Time.unscaledDeltaTime * 1000f:0.0} ms, pace {(vsyncLock30 ? "30 lock" : "60 vsync")}");
        }

        void UpdateHud(double wall)
        {
            hud.Visible = overlays[Panel.Hud];
            telemetry.Visible = overlays[Panel.Telemetry];
            consolePanel.Visible = overlays[Panel.Console];
            // open panels never overlap (Task 5 review I2): the console stops at the minimap, the left column at the console
            float sw = Screen.width, sh = Screen.height;
            var minimapLeft = MinimapOpen && map != null ? PanelLayout.Minimap(sw, sh, map.Width, map.Height).xMin : sw;
            consolePanel.Area = PanelLayout.Console(sw, sh, minimapLeft);
            var bottom = consolePanel.Visible ? consolePanel.Area.yMin : sh;
            telemetry.Area = PanelLayout.Telemetry(sw, sh, bottom, minimapLeft);
            legend.Area = PanelLayout.Legend(sw, sh, legend.Rows.Count, bottom, minimapLeft);
            UpdateCameraJournal(wall, new[]
            {
                MinimapOpen && map != null ? PanelLayout.Minimap(sw, sh, map.Width, map.Height) : default,
                consolePanel.Visible ? consolePanel.Area : default, telemetry.Visible ? telemetry.Area : default,
                legend.Visible ? legend.Area : default,
            });
            if (telemetry.Visible) BuildTelemetry(wall);
            var hello = state.SessionHello;
            var session = hello == null ? "no session" : $"{(string)hello["scenario"]} seed {(int?)hello["seed"]}";
            hud.Status = Replaying ? ReplayStatus() : $"{client.Status} {options.Host}:{options.Port} | {session} | t {(double.IsNaN(displayedT) ? "-" : displayedT.ToString("F2"))} s | " +
                $"speed {requestedSpeed:0.##}x, achieved {state.AchievedSpeed(wall):0.00}x | {fps:0} FPS | " +
                $"dropped {client.Dropped}, malformed {client.Malformed} | {view}" + (state.Running ? "" : " | paused");
            var warnings = new List<string>();
            if (state.IsStale(wall)) warnings.Add($"stale: no state for {state.SilentFor(wall):0.0} s");
            if (client.Status == ConnectionStatus.SchemaMismatch) warnings.Add("the server speaks another protocol version");
            if (hello != null && site != null && (string)hello["site_checksum"] != site.Checksum) warnings.Add("site differs");
            if (journal.Error != null) warnings.Add("journal off: " + journal.Error);
            if (mapError != null) warnings.Add("minimap goals off: " + mapError);  // Task 6 review I1
            if (optionsError != null) warnings.Add("command line: " + optionsError);
            if (replayError != null) warnings.Add(replayError);
            hud.Warning = string.Join(" | ", warnings);
            foreach (var w in StatusMessage.Added(lastWarnings, warnings)) Note(Severity.Warning, "viewer", w);  // review I6
            lastWarnings = warnings;
            hud.Message = message.Text(wall);
        }

        /// <summary>Frame times for the FPS measurement (plan 4A Task 7): one wall time per frame, written at exit.</summary>
        void WriteFrameTimes()
        {
            if (frameTimes == null || frameTimesWritten) return;
            frameTimesWritten = true;
            try
            {
                using var w = new StreamWriter(options.FrameTimes, false);
                w.WriteLine($"# screen {Screen.width}x{Screen.height}, quality {QualitySettings.names[QualitySettings.GetQualityLevel()]}, " +
                    $"fps cap {(vsyncLock30 ? "30 vsync lock" : options.Fps > 0 ? options.Fps.ToString() : "vsync")}, " +
                    $"run_started_s {(double.IsNaN(runStartedAt) ? "-" : runStartedAt.ToString("F4", System.Globalization.CultureInfo.InvariantCulture))}");
                w.WriteLine("wall_s,display_t");  // display_t: the simulation time drawn (empty before the first State)
                var inv = System.Globalization.CultureInfo.InvariantCulture;
                foreach (var (t, d) in frameTimes)
                    w.WriteLine(t.ToString("F5", inv) + "," + (double.IsNaN(d) ? "" : d.ToString("F5", inv)));
            }
            catch (Exception e)
            {
                Debug.LogWarning("frame times not written: " + e.Message);
            }
        }

        void OnApplicationQuit() => WriteFrameTimes();

        void OnDestroy()
        {
            Application.logMessageReceived -= OnUnityLog;
            if (robotCamera != null) robotCamera.Recentered -= OnRecentered;
            WriteFrameTimes();
            client?.Dispose();
            journal?.Dispose();
        }
    }
}
