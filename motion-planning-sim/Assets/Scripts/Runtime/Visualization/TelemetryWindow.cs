using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MotionPlanningSim.Control;
using MotionPlanningSim.Environment;
using MotionPlanningSim.ROS;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MotionPlanningSim.Visualization
{
    /// <summary>
    /// Live telemetry window and path overlay, toggled by an on-screen button or the T key.
    /// </summary>
    /// <remarks>
    /// Unity-side values (base motion, applied command, arm state, contacts, clock) are read
    /// locally. ROS-side navigation state arrives as one small JSON message on /mm/telemetry
    /// (5 Hz) and a downsampled path on /mm/telemetry/path (on change, resent every 5 s), so
    /// the window adds about 2-3 KB/s to the ROS link. Text is rebuilt at 5 Hz, not per frame.
    /// </remarks>
    [DisallowMultipleComponent]
    public sealed class TelemetryWindow : MonoBehaviour
    {
        private const float RefreshPeriod = 0.2f;
        private const int UnityEventCapacity = 15;
        private const int WindowId = 0x4D4D5445;  // arbitrary unique IMGUI window id

        [SerializeField] private string telemetryTopic = "/mm/telemetry";
        [SerializeField] private string pathTopic = "/mm/telemetry/path";
        [SerializeField] private ArticulationBody baseLink;
        [SerializeField] private SkidSteerBaseController baseController;
        [SerializeField] private ArmActuatorController arm;
        [SerializeField] private Collider payload;
        [SerializeField] private SimulationClockPublisher clock;
        [SerializeField] private bool visible;
        [SerializeField, Min(0.01f)] private float trailSpacingMetres = 0.1f;
        [SerializeField, Min(10)] private int trailCapacity = 3000;

        private readonly TelemetryFeed feed = new TelemetryFeed();
        private readonly List<(double time, string text)> unityEvents = new List<(double, string)>();
        private readonly StringBuilder builder = new StringBuilder(2048);
        private readonly List<Vector3> trail = new List<Vector3>();
        private ROSConnection ros;
        private PathMsg pendingPath;
        private string body = "Waiting for data...";
        private float nextRefresh;
        private const float WindowWidth = 340f;
        private Rect windowRect;
        private bool windowPlaced;
        private Vector2 scroll;
        private GUIStyle textStyle;
        private int lastGoalSeq = -1;
        private int lastContactPairs;
        private bool lastWatchdog;
        private ArmActuatorState lastArmState;
        private long lastClockTicks;
        private double lastClockWall, lastClockSim, clockRateHz, realTimeFactor;
        private LineRenderer planLine, trailLine, goalLine;
        private GameObject overlay;

        public bool Visible => visible;
        public TelemetryFeed Feed => feed;

        public void Configure(ArticulationBody configuredBase, SkidSteerBaseController configuredController,
            ArmActuatorController configuredArm, Collider configuredPayload, SimulationClockPublisher configuredClock)
        {
            baseLink = configuredBase;
            baseController = configuredController;
            arm = configuredArm;
            payload = configuredPayload;
            clock = configuredClock;
        }

        public void SetVisible(bool value)
        {
            visible = value;
            if (overlay != null)
            {
                overlay.SetActive(value);
            }

            if (value && Application.isPlaying)
            {
                // Show current values immediately instead of text from the last time it was open.
                body = BuildText();
                nextRefresh = Time.unscaledTime + RefreshPeriod;
            }
        }

        private void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<StringMsg>(telemetryTopic, message => feed.Accept(message.data, WallTime));
            ros.Subscribe<PathMsg>(pathTopic, message => pendingPath = message);
            CreateOverlay();
            SetVisible(visible);
            lastArmState = arm != null ? arm.State : ArmActuatorState.INITIALIZING;
        }

        private static double WallTime => Time.realtimeSinceStartupAsDouble;

        private void Update()
        {
            if (Keyboard.current != null && Keyboard.current.tKey.wasPressedThisFrame)
            {
                SetVisible(!visible);
            }

            if (pendingPath != null)
            {
                ApplyPath(pendingPath);
                pendingPath = null;
            }

            SampleTrail();
            if (Time.unscaledTime >= nextRefresh)
            {
                nextRefresh = Time.unscaledTime + RefreshPeriod;
                TrackUnityEvents();
                if (visible)
                {
                    body = BuildText();
                }
            }
        }

        private void OnGUI()
        {
            // Top-right, clear of the ROS-TCP-Connector HUD in the top-left corner.
            var label = visible ? "Hide telemetry (T)" : "Telemetry (T)";
            if (GUI.Button(new Rect(Screen.width - 150, 8, 142, 22), label))
            {
                SetVisible(!visible);
            }

            if (!visible)
            {
                return;
            }

            textStyle ??= new GUIStyle(GUI.skin.label) { richText = true, fontSize = 11, wordWrap = true };
            if (!windowPlaced)
            {
                windowRect = new Rect(Screen.width - WindowWidth - 8, 36, WindowWidth,
                    Mathf.Min(Screen.height - 44, 560));
                windowPlaced = true;
            }

            windowRect = GUI.Window(WindowId, windowRect, DrawWindow, "Navigation telemetry");
        }

        private void DrawWindow(int id)
        {
            scroll = GUILayout.BeginScrollView(scroll);
            GUILayout.Label(body, textStyle);
            GUILayout.EndScrollView();
            GUI.DragWindow(new Rect(0, 0, 10000, 20));
        }

        private string BuildText()
        {
            var now = WallTime;
            var t = feed.Latest;
            var stale = feed.IsStale(now);
            builder.Clear();

            Section("Run");
            if (t == null)
            {
                Line("ROS telemetry", "<color=orange>no data (is navigation.launch.py running?)</color>");
            }
            else
            {
                Line("Telemetry", stale ? $"<color=orange>STALE {feed.Age(now):F1} s</color>" : $"{feed.Age(now):F1} s old");
                Line("Scenario", Or(t.scenario, "-"));
                Line("Profile / tree", $"{Or(t.footprint_profile, "-")} / {Or(t.behavior_tree, "-")}");
                Line("Planner / controller", $"{t.planner} / {t.controller}");
                Line("Goal", $"{Status(t.goal.status)}  recoveries {t.goal.recoveries}");
                Line("Elapsed / ETA", $"{t.goal.elapsed_s:F1} s / {Num(t.goal.eta_s, "F1")} s");
                Line("Distance remaining", $"{Num(t.goal.distance_remaining_m, "F2")} m");

                Section("Path");
                Line("Length / points", $"{t.path.length_m:F2} m / {t.path.points}");
                Line("Plan age", $"{Num(t.path.age_s, "F1")} s");
                Line("Cross-track now / max", $"{Num(t.path.cross_track_m, "F3")} / {t.path.cross_track_max_m:F3} m");
            }

            Section("Motion (Unity ground truth)");
            if (baseLink != null)
            {
                var forward = Vector3.Dot(baseLink.linearVelocity, baseLink.transform.forward);
                var yawRate = -baseLink.angularVelocity.y;  // ROS yaw is counter-clockwise
                Line("Actual v / w", $"{forward:F2} m/s / {yawRate:F2} rad/s");
            }

            if (baseController != null)
            {
                Line("Applied cmd v / w", $"{baseController.LimitedLinearMetresPerSecond:F2} / {baseController.LimitedAngularRadiansPerSecond:F2}");
                // With no command source the watchdog holding the base is the expected idle state;
                // flag it only while a navigation goal is executing.
                var executing = t != null && !stale && t.goal.status == "executing";
                var watchdog = !baseController.WatchdogActive ? "commanded" :
                    executing ? "<color=orange>STOPPED (no fresh cmd)</color>" : "holding (no commands)";
                Line("Watchdog / saturated", $"{watchdog} / {Flag(baseController.WheelCommandSaturated, "yes")}");
                Line("Rejected commands", baseController.RejectedCommandCount.ToString());
            }

            Section("Safety");
            if (t != null)
            {
                Line("Collision monitor", t.monitor.action == "none" ? "clear" :
                    $"<color=orange>{t.monitor.action}</color> ({t.monitor.polygon})");
            }

            var contacts = ContactMonitor();
            if (contacts != null)
            {
                var entries = contacts.Ledger.Entries;
                Line("Contacts since reset", entries.Count == 0 ? "none" :
                    $"<color=red>{entries.Count} pair(s), max {entries.Max(e => e.MaxPenetration):F3} m</color>");
            }
            else
            {
                Line("Contacts", "monitor not started (scenario_contacts_reset)");
            }

            Section("Arm");
            if (arm != null)
            {
                var error = 0.0;
                for (var i = 0; i < 6; i++)
                {
                    error = Math.Max(error, Math.Abs(arm.Position(i) - arm.DesiredPosition[i]));
                }

                Line("State / command age", $"{arm.State} / {(arm.CommandAge < 0 ? "-" : $"{arm.CommandAge:F2} s")}");
                Line("Max joint error", $"{error:F3} rad");
            }

            if (payload != null)
            {
                Line("Panel floor clearance", $"{payload.bounds.min.y:F2} m");
            }

            Section("Link and sensors");
            Line("ROS connection", ros == null ? "-" : ros.HasConnectionError ? "<color=red>error</color>" : "ok");
            Line("Sim time / clock", $"{RosTimeUtility.PhysicsTimeSeconds:F1} s / {clockRateHz:F0} Hz (x{realTimeFactor:F2} real time)");
            if (t != null)
            {
                Line("Filtered lidar", $"{t.lidar.rate_hz:F1} Hz, gaps > 0.5 s: {t.lidar.gaps_over_0p5s} (max {t.lidar.max_gap_s:F2} s)");
            }

            Section("Events (sim time)");
            foreach (var (time, text) in MergedEvents(t).Take(15))
            {
                builder.Append(time.ToString("F1")).Append("  ").Append(text).Append('\n');
            }

            return builder.ToString();
        }

        private IEnumerable<(double, string)> MergedEvents(NavigationTelemetry t)
        {
            var rosEvents = t?.events?.Select(e => ((double)e.t, e.text)) ?? Enumerable.Empty<(double, string)>();
            return rosEvents.Concat(unityEvents).OrderByDescending(e => e.Item1);
        }

        private void TrackUnityEvents()
        {
            var now = RosTimeUtility.PhysicsTimeSeconds;
            if (baseController != null && baseController.WatchdogActive != lastWatchdog)
            {
                lastWatchdog = baseController.WatchdogActive;
                AddUnityEvent(now, lastWatchdog ? "base watchdog stop" : "base commands resumed");
            }

            if (arm != null && arm.State != lastArmState)
            {
                lastArmState = arm.State;
                AddUnityEvent(now, $"arm {lastArmState}");
            }

            var contacts = ContactMonitor();
            var pairs = contacts?.Ledger.Entries.Count ?? 0;
            if (pairs > lastContactPairs)
            {
                var worst = contacts.Ledger.Entries.OrderByDescending(e => e.MaxPenetration).First();
                AddUnityEvent(now, $"<color=red>CONTACT {worst.RobotCollider} / {worst.Other}</color>");
            }

            lastContactPairs = pairs;
            if (clock != null)
            {
                var wall = WallTime;
                if (lastClockWall > 0 && wall > lastClockWall)
                {
                    clockRateHz = (clock.PublishedTicks - lastClockTicks) / (wall - lastClockWall);
                    realTimeFactor = (now - lastClockSim) / (wall - lastClockWall);
                }

                lastClockTicks = clock.PublishedTicks;
                lastClockWall = wall;
                lastClockSim = now;
            }

            var goalSeq = feed.Latest?.goal_seq ?? -1;
            if (goalSeq != lastGoalSeq)
            {
                lastGoalSeq = goalSeq;
                trail.Clear();
                if (trailLine != null)
                {
                    trailLine.positionCount = 0;
                }
            }
        }

        private void AddUnityEvent(double time, string text)
        {
            unityEvents.Add((time, text));
            if (unityEvents.Count > UnityEventCapacity)
            {
                unityEvents.RemoveAt(0);
            }
        }

        private RobotContactMonitor ContactMonitor() => arm != null ? arm.GetComponent<RobotContactMonitor>() : null;

        private void CreateOverlay()
        {
            overlay = new GameObject("TelemetryOverlay");
            overlay.transform.SetParent(transform, false);
            planLine = CreateLine("PlannedPath", new Color(0.1f, 0.8f, 1.0f), 0.05f);
            trailLine = CreateLine("ExecutedTrail", new Color(1.0f, 0.55f, 0.1f), 0.04f);
            goalLine = CreateLine("Goal", new Color(0.2f, 1.0f, 0.3f), 0.05f);
            goalLine.loop = true;
        }

        private LineRenderer CreateLine(string name, Color color, float width)
        {
            var child = new GameObject(name);
            child.transform.SetParent(overlay.transform, false);
            var line = child.AddComponent<LineRenderer>();
            var shader = Shader.Find("HDRP/Unlit") ?? Shader.Find("Sprites/Default");
            line.material = new Material(shader) { color = color };
            if (line.material.HasProperty("_UnlitColor"))
            {
                line.material.SetColor("_UnlitColor", color);
            }

            line.startColor = line.endColor = color;
            line.startWidth = line.endWidth = width;
            line.useWorldSpace = true;
            line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            line.receiveShadows = false;
            line.positionCount = 0;
            return line;
        }

        private void ApplyPath(PathMsg path)
        {
            var points = path.poses.Select(p => RosMapPose.ToUnityPosition(
                p.pose.position.x, p.pose.position.y, 0.06f)).ToArray();
            planLine.positionCount = points.Length;
            planLine.SetPositions(points);
            if (points.Length == 0)
            {
                goalLine.positionCount = 0;
                return;
            }

            var goal = points[points.Length - 1];
            const int segments = 24;
            goalLine.positionCount = segments;
            for (var i = 0; i < segments; i++)
            {
                var angle = i * Mathf.PI * 2f / segments;
                goalLine.SetPosition(i, goal + new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * 0.3f);
            }
        }

        private void SampleTrail()
        {
            if (baseLink == null || trailLine == null)
            {
                return;
            }

            var position = baseLink.transform.position;
            position.y = 0.05f;
            if (trail.Count > 0 && Vector3.Distance(trail[trail.Count - 1], position) < trailSpacingMetres)
            {
                return;
            }

            if (trail.Count >= trailCapacity)
            {
                trail.RemoveAt(0);
            }

            trail.Add(position);
            trailLine.positionCount = trail.Count;
            trailLine.SetPositions(trail.ToArray());
        }

        private void Section(string title) => builder.Append("\n<b>").Append(title).Append("</b>\n");

        private void Line(string name, string value) => builder.Append(name).Append(": ").Append(value).Append('\n');

        private static string Num(float value, string format) => value < 0 ? "-" : value.ToString(format);

        private static string Or(string value, string fallback) => string.IsNullOrEmpty(value) ? fallback : value;

        private static string Flag(bool value, string text) => value ? $"<color=orange>{text}</color>" : "no";

        private static string Status(string status) => status switch
        {
            "succeeded" => "<color=lime>succeeded</color>",
            "aborted" => "<color=red>aborted</color>",
            "canceled" => "<color=orange>canceled</color>",
            "executing" => "<color=cyan>executing</color>",
            _ => status
        };
    }
}
