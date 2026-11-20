using System;
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
    /// backend commands. It runs after the camera script so its camera choice wins. Arguments: <c>-host</c>
    /// (default localhost), <c>-port</c> (7700), <c>-journal-dir</c> (default ~/Movies/PlantViewer).
    /// </summary>
    [DefaultExecutionOrder(100)]
    public sealed class PlantViewerBootstrap : MonoBehaviour
    {
        const double DisplayLagS = 0.04;
        const int MaxFramesPerUpdate = 1024;
        const double JournalPoseHz = 10.0;

        [SerializeField] private PlantRobotView robot;
        [SerializeField] private PlantEnvironmentView environment;
        [SerializeField] private PlantViewerSite site;
        [SerializeField] private ThirdPersonRobotCamera robotCamera;
        [SerializeField] private FreeCamera freeCamera;
        [SerializeField] private PlantViewerHud hud;
        [SerializeField] private PlantViewerInput input;

        private readonly ViewerState state = new ViewerState();
        private ViewerCommands commands;
        private StreamClient client;
        private JournalRecorder journal;
        private string journalFolder;
        private JObject appliedHello;
        private ViewerCamera view = ViewerCamera.Orbit;
        private double requestedSpeed = 1.0;
        private double lastSampleT = double.NaN;
        private double displayedT;
        private double lastPoseJournal;
        private float fps;
        private string host = "localhost";
        private int port = 7700;

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

        void Awake()
        {
            var missing = string.Join(", ", MissingReferences());
            if (missing.Length > 0) throw new InvalidOperationException("PlantViewerBootstrap is missing " + missing);
            var args = System.Environment.GetCommandLineArgs();
            host = Argument(args, "-host") ?? host;
            if (int.TryParse(Argument(args, "-port"), out var p)) port = p;
            var root = Argument(args, "-journal-dir") ??
                Path.Combine(System.Environment.GetFolderPath(System.Environment.SpecialFolder.Personal), "Movies", "PlantViewer");
            journalFolder = Path.Combine(root, DateTime.UtcNow.ToString("yyyyMMdd'T'HHmmss'Z'"));
            commands = new ViewerCommands();
            client = new StreamClient(host, port, commands.Hello);
            journal = new JournalRecorder(OpenJournal, () => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0);
            freeCamera.enabled = false;
        }

        void Start() => client.Start();

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

            foreach (var action in input.Actions) Perform(action);
            input.Actions.Clear();
            if (hud.PickedScenario != null)
            {
                Send(commands.Reset(hud.PickedScenario, 1), "reset");
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
            UpdateHud(wall);
        }

        void OnSession(JObject hello)
        {
            appliedHello = hello;
            lastSampleT = double.NaN;
            displayedT = double.NaN;
            if (hello == null) return;  // a new connection: wait for the server's replay
            environment.ApplySession(hello);
            // One journal per session; only the server's replay after a reconnect continues it (review I3).
            journal.OnSession(hello, state.SessionIsReplay);
        }

        /// <summary>Opens journal file n; never overwrites one (Task 5 review I1, M3).</summary>
        TextWriter OpenJournal(int n)
        {
            Directory.CreateDirectory(journalFolder);
            var path = Path.Combine(journalFolder, $"journal-{n}.jsonl");
            return new StreamWriter(new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.Read));
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
                    else if (Send(commands.Reset((string)hello["scenario"], (int?)hello["seed"] ?? 1), "reset")) SetRunning(false);
                    break;
                case ViewerAction.ScenarioPicker:
                    if (hud.PickerOpen) hud.PickerOpen = false;
                    else OpenPicker();
                    break;
            }
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
            hud.Status = $"{client.Status} {host}:{port} | {session} | t {(double.IsNaN(displayedT) ? "-" : displayedT.ToString("F2"))} s | " +
                $"speed {requestedSpeed:0.##}x, achieved {state.AchievedSpeed(wall):0.00}x | {fps:0} FPS | " +
                $"dropped {client.Dropped}, malformed {client.Malformed} | {view}" + (state.Running ? "" : " | paused");
            var warnings = new List<string>();
            if (state.IsStale(wall)) warnings.Add($"stale: no state for {state.SilentFor(wall):0.0} s");
            if (client.Status == ConnectionStatus.SchemaMismatch) warnings.Add("the server speaks another protocol version");
            if (hello != null && site != null && (string)hello["site_checksum"] != site.Checksum) warnings.Add("site differs");
            if (journal.Error != null) warnings.Add("journal off: " + journal.Error);
            hud.Warning = string.Join(" | ", warnings);
        }

        static string Argument(string[] args, string name)
        {
            for (var i = 0; i + 1 < args.Length; ++i)
                if (args[i] == name) return args[i + 1];
            return null;
        }

        void OnDestroy()
        {
            client?.Dispose();
            journal?.Dispose();
        }
    }
}
