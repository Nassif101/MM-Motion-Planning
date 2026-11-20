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
    public sealed class PlantViewerBootstrap : MonoBehaviour
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

        private readonly ViewerState state = new ViewerState();
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
        public bool MinimapOpen { get; private set; }

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
            if (options.Fps > 0)  // a frame-rate cap replaces vsync
            {
                QualitySettings.vSyncCount = 0;
                Application.targetFrameRate = options.Fps;
            }
            if (options.Scenario != null) demo = Demo.WaitConnect;
            requestedSpeed = options.Speed;
            var root = options.JournalDir ??
                Path.Combine(System.Environment.GetFolderPath(System.Environment.SpecialFolder.Personal), "Movies", "PlantViewer");
            journalFolder = Path.Combine(root, DateTime.UtcNow.ToString("yyyyMMdd'T'HHmmss'Z'"));
            commands = new ViewerCommands();
            client = new StreamClient(options.Host, options.Port, commands.Hello);
            journal = new JournalRecorder(OpenJournal, () => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0);
            freeCamera.enabled = false;
        }

        void Start()
        {
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
            client.Start();
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
            for (var n = 0; n < MaxFramesPerUpdate && client.TryDequeue(out var frame); ++n) state.Apply(frame, wall);
            if (!ReferenceEquals(state.SessionHello, appliedHello)) OnSession(state.SessionHello);
            while (state.Events.Count > 0) OnEvent(state.Events.Dequeue());

            // No state yet (a reset or a new server): hide the robot rather than show a stale pose (live check).
            var hasSample = state.TrySample(DisplayLagS, wall, out var sample);
            if (robot.gameObject.activeSelf != hasSample) robot.gameObject.SetActive(hasSample);
            if (hasSample)
            {
                var dt = double.IsNaN(lastSampleT) ? 0.0 : Math.Max(0.0, sample.T - lastSampleT);
                robot.Apply(sample, (float)dt);
                lastSampleT = sample.T;
                displayedT = sample.T;
            }
            environment.ApplyState(state);
            UpdateGoals(hasSample ? (sample.X, sample.Y, sample.Yaw) : ((double, double, double)?)null, wall);

            foreach (var action in input.Actions) Perform(action);
            input.Actions.Clear();
            if (hud.PickedScenario != null)
            {
                SendReset(hud.PickedScenario, 1);
                hud.PickedScenario = null;
            }
            if (view == ViewerCamera.Free && freeCamera.Moved && wall - lastPoseJournal >= 1.0 / JournalPoseHz)
            {
                lastPoseJournal = wall;
                var t = freeCamera.transform;
                Journal("free_camera", new JObject
                {
                    ["position"] = new JArray(t.position.x, t.position.y, t.position.z),
                    ["rotation"] = new JArray(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                }, thinnable: true);
            }

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
            if (hello == null) return;  // a new connection: wait for the server's replay
            environment.ApplySession(hello);
            // One journal per session; only the server's replay after a reconnect continues it (review I3).
            journal.OnSession(hello, state.SessionIsReplay);
            if (demo == Demo.WaitSession && !state.SessionIsReplay) StartDemo();
        }

        /// <summary>
        /// The scripted demo resets once, a second after connecting: the server's replay of an old session arrives
        /// before then, so the next session Hello answers this reset.
        /// </summary>
        void StepDemo(double wall)
        {
            if (demo != Demo.WaitConnect) return;
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
            if (options.Minimap) MinimapOpen = true;
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
            route.Present(state);
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

        void OnEvent(JObject e)
        {
            switch ((string)e["event"])
            {
                case "scenarios":
                    hud.Scenarios.Clear();
                    if (e["scenarios"] is JArray list)
                        foreach (var s in list) hud.Scenarios.Add(s.Type == JTokenType.Object ? (string)s["name"] : (string)s);
                    break;
                case "error":
                    hud.Message = "server: " + (string)e["reason"];
                    break;
                case "goal_ack":
                    if ((string)e["action"] != "accepted" && (string)e["action"] != "applied" && (string)e["action"] != "scheduled")
                        hud.Message = $"{(string)e["command"]}: {(string)e["action"]} {(string)e["reason"]}";
                    break;
            }
        }

        /// <summary>Performs one hotkey action, as if its key were pressed (also for automation and 4C).</summary>
        public void Perform(ViewerAction action)
        {
            Journal("action", new JObject { ["action"] = action.ToString() });
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
                    if (state.Running) Send(commands.Run(requestedSpeed), "run");
                    break;
                case ViewerAction.NextCamera:
                    SetCamera((ViewerCamera)(((int)view + 1) % 4));
                    break;
                case ViewerAction.ToggleMinimap:
                    MinimapOpen = !MinimapOpen;
                    break;
                case ViewerAction.CancelGoal:
                    if (!state.Has("cancel_goal")) hud.Message = "cancel: this server has no cancel_goal";
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

        void SetRunning(bool value) => state.NoteRunning(value, Time.realtimeSinceStartupAsDouble);

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
            if (client.Send(command)) return true;
            hud.Message = $"{what}: not connected";
            return false;
        }

        /// <summary>Journals one viewer action at the displayed simulation time (design 6.6).</summary>
        public void Journal(string kind, JObject fields = null, bool thinnable = false) =>
            journal?.Write(double.IsNaN(displayedT) ? 0.0 : displayedT, kind, fields, thinnable);  // a session starts at 0

        void UpdateHud(double wall)
        {
            var hello = state.SessionHello;
            var session = hello == null ? "no session" : $"{(string)hello["scenario"]} seed {(int?)hello["seed"]}";
            hud.Status = $"{client.Status} {options.Host}:{options.Port} | {session} | t {(double.IsNaN(displayedT) ? "-" : displayedT.ToString("F2"))} s | " +
                $"speed {requestedSpeed:0.##}x, achieved {state.AchievedSpeed(wall):0.00}x | {fps:0} FPS | " +
                $"dropped {client.Dropped}, malformed {client.Malformed} | {view}" + (state.Running ? "" : " | paused");
            var warnings = new List<string>();
            if (state.IsStale(wall)) warnings.Add($"stale: no state for {state.SilentFor(wall):0.0} s");
            if (client.Status == ConnectionStatus.SchemaMismatch) warnings.Add("the server speaks another protocol version");
            if (hello != null && site != null && (string)hello["site_checksum"] != site.Checksum) warnings.Add("site differs");
            if (journal.Error != null) warnings.Add("journal off: " + journal.Error);
            if (mapError != null) warnings.Add("minimap goals off: " + mapError);  // Task 6 review I1
            if (optionsError != null) warnings.Add("command line: " + optionsError);
            hud.Warning = string.Join(" | ", warnings);
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
                    $"fps cap {(options.Fps > 0 ? options.Fps.ToString() : "vsync")}, " +
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
            WriteFrameTimes();
            client?.Dispose();
            journal?.Dispose();
        }
    }
}
