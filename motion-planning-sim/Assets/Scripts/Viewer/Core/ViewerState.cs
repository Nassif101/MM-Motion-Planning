using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>One robot state from a State frame: block 0 (true base pose and arm joints) and block 2's base twist.</summary>
    public readonly struct RobotSample
    {
        public readonly double T, X, Y, Yaw, V, Omega;
        public readonly double[] Joints;

        public RobotSample(double t, double x, double y, double yaw, double v, double omega, double[] joints)
        {
            T = t; X = x; Y = y; Yaw = yaw; V = v; Omega = omega; Joints = joints;
        }
    }

    /// <summary>
    /// The viewer's picture of the session, built only from backend frames (design sections 5.3 and 6). Main thread
    /// only. A session Hello clears everything; Goal and Route frames of an older epoch are ignored; the robot is
    /// sampled behind the newest state and interpolated, never extrapolated.
    /// </summary>
    public sealed class ViewerState
    {
        const int MaxStates = 256;  // 5 s at 50 Hz: the lidar overlay places points from this history (4B-1)
        const double SpeedWindowS = 2.0;
        const double StaleAfterS = 0.25;

        readonly List<RobotSample> states = new List<RobotSample>();
        readonly List<double> stateWalls = new List<double>();  // each State's arrival wall time
        double lastDisplay = double.NegativeInfinity;
        double lastSampleWall = double.NaN;
        double rateSince = double.NegativeInfinity;  // States before this wall time ran at another speed
        double rateHint = double.NaN;                // the requested speed, used until the new rate is measured
        const double RateSettleS = 0.25;
        const double ClockGain = 2.0;      // 1/s: how fast the display clock is pulled toward newest - lag
        const double ClockSnapS = 0.5;     // further than this from its goal, the display clock jumps
        readonly Queue<(double wall, double sim)> speedSamples = new Queue<(double wall, double sim)>();
        readonly HashSet<string> capabilities = new HashSet<string>();
        readonly List<(string name, double x, double y)> movers = new List<(string, double, double)>();
        readonly List<(string name, double x, double y, double yaw)> bodies = new List<(string, double, double, double)>();
        readonly List<(double x, double y, double yaw)> routePath = new List<(double, double, double)>();
        double lastStateWall = double.NegativeInfinity;
        bool running;
        double runSince = double.NegativeInfinity;
        bool awaitingReplay;
        long routeGoalId;

        public JObject SessionHello { get; private set; }
        public long Epoch { get; private set; }
        public bool GeneratedRoutes { get; private set; }
        public JArray Boxes { get; private set; } = new JArray();
        public IReadOnlyList<(string name, double x, double y)> Movers => movers;
        public IReadOnlyList<(string name, double x, double y, double yaw)> Bodies => bodies;
        public IReadOnlyList<(double x, double y, double yaw)> RoutePath => routePath;
        public long RouteId { get; private set; }
        public GoalMarkerModel Goal { get; } = new GoalMarkerModel();

        /// <summary>F2's latest plan for the active goal (4B-1), or null.</summary>
        public MediumPlan Plan { get; private set; }

        /// <summary>The session runs F2 (Hello <c>config.medium</c>); otherwise there are no plans to draw.</summary>
        public bool HasMediumPlans { get; private set; }

        /// <summary>The session streams lidar scans (Hello <c>config.lidar</c>).</summary>
        public bool HasLidar { get; private set; }

        /// <summary>The plan is more than 1 s older than the newest State.</summary>
        public bool PlanIsStale(double newestSimT) => Plan != null && newestSimT - Plan.ReceivedAtSimT > 1.0;

        /// <summary>The base pose at a simulation time inside the State history, interpolated (yaw the short way).</summary>
        public bool TryPoseAt(double simT, out (double x, double y, double yaw) pose)
        {
            pose = default;
            if (states.Count == 0 || simT < states[0].T || simT > states[states.Count - 1].T) return false;
            for (var i = states.Count - 1; i >= 0; --i)
            {
                var b = states[i];
                if (b.T < simT) continue;
                if (b.T == simT || i == 0)
                {
                    pose = (b.X, b.Y, b.Yaw);
                    return true;
                }
                var a = states[i - 1];
                if (a.T > simT) continue;
                var f = (simT - a.T) / (b.T - a.T);
                pose = (a.X + f * (b.X - a.X), a.Y + f * (b.Y - a.Y), Angle(a.Yaw + f * Angle(b.Yaw - a.Yaw)));
                return true;
            }
            return false;
        }

        /// <summary>The latest event frames (scenarios, errors, goal acks) for presenters; drained by the caller.</summary>
        public Queue<JObject> Events { get; } = new Queue<JObject>();

        public bool Has(string capability) => capabilities.Contains(capability);
        /// <summary>
        /// Whether the backend runs, as far as the viewer knows: set by the viewer's own run requests, cleared by
        /// pause and step, by every Hello (a new connection and a reset both pause the server) and by run_stopped
        /// (Task 5 review I2).
        /// </summary>
        public bool Running => running;

        /// <summary>
        /// The session Hello is the server's replay after a connect, not a new session (a reset); Task 5 review I3.
        /// </summary>
        public bool SessionIsReplay { get; private set; }

        /// <summary>
        /// The viewer sent a reset: the session Hello that answers it is a new session even right after a connect
        /// (a restarted server with no session sends no replay); Task 6 review I3.
        /// </summary>
        public void NoteResetSent() => awaitingReplay = false;

        /// <summary>
        /// The robot's latest pose: the newest State, or the session's start pose before the first State (a reset
        /// pauses the server, which then sends none); Task 6 review I2. A replayed session has no start fallback: its
        /// Hello is from the last reset, and the robot may have moved since (Task 7 review I1).
        /// </summary>
        public bool TryRobotPose(out (double x, double y, double yaw) pose)
        {
            pose = default;
            if (states.Count > 0)
            {
                var last = states[states.Count - 1];
                pose = (last.X, last.Y, last.Yaw);
                return true;
            }
            if (!SessionIsReplay && SessionHello?["start"] is JArray start && start.Count >= 3)
            {
                pose = ((double)start[0], (double)start[1], (double)start[2]);
                return true;
            }
            return false;
        }

        /// <summary>
        /// The viewer asked for a new speed (a run or a speed change): the display clock uses it until 0.25 s of
        /// States at the new speed have arrived, then their measured rate. The 2 s rate window otherwise lagged the
        /// change and froze the display for about 2 s (final review M2).
        /// </summary>
        public void NoteSpeedChange(double speed, double wallNow)
        {
            rateSince = wallNow;
            rateHint = speed;
        }

        public void NoteRunning(bool isRunning, double wallNow)
        {
            running = isRunning;
            if (isRunning) runSince = wallNow;
        }

        /// <summary>Seconds without a State, counted from the later of the last State and the run request.</summary>
        public double SilentFor(double wallNow) => wallNow - Math.Max(lastStateWall, runSince);
        public bool IsStale(double wallNow) => running && SilentFor(wallNow) > StaleAfterS;

        /// <summary>Wall time of the last State frame; negative infinity before the first.</summary>
        public double LastStateWall => lastStateWall;

        public void Apply(Frame frame, double wallNow)
        {
            switch (frame.Type)
            {
                case FrameType.Hello:
                    // A session Hello starts a session; the connect-time server Hello means the old session may be
                    // gone (Task 3 review I1): only the server's replay may restore it.
                    running = false;
                    if (frame.Header["scenario"] != null)
                    {
                        var replay = awaitingReplay;
                        StartSession(frame.Header);
                        SessionIsReplay = replay;
                        awaitingReplay = false;
                    }
                    else
                    {
                        ClearSession();
                        awaitingReplay = true;
                    }
                    break;
                case FrameType.State:
                    ApplyState(frame, wallNow);
                    break;
                case FrameType.Goal:
                    if (FromOlderEpoch(frame.Header)) break;
                    Goal.ApplyGoalFrame(frame.Header);
                    if (frame.Header["goal"] is JObject g && (long)g["goal_id"] == routeGoalId &&
                        (string)g["state"] != "planning" && (string)g["state"] != "tracking")
                        ClearRoute();
                    // the plan's goal ended, or a newer goal became active: the old plan goes at once. A frame without
                    // a goal (a refused click, a scheduled submission) says nothing about the active goal (review I1).
                    if (Plan != null && frame.Header["goal"] is JObject pg)
                    {
                        var id = (long)pg["goal_id"];
                        var active = (string)pg["state"] == "planning" || (string)pg["state"] == "tracking";
                        if ((id == Plan.GoalId && !active) || (id > Plan.GoalId && active)) Plan = null;
                    }
                    break;
                case FrameType.Plans:
                    ApplyPlans(frame);
                    break;
                case FrameType.Route:
                    if (FromOlderEpoch(frame.Header)) break;
                    ApplyRoute(frame);
                    break;
                case FrameType.Event:
                    if ((string)frame.Header["event"] == "goal_ack") Goal.ApplyAck(frame.Header);
                    if ((string)frame.Header["event"] == "run_stopped") running = false;
                    if (Events.Count >= 256) Events.Dequeue();
                    Events.Enqueue(frame.Header);
                    break;
            }
        }

        /// <summary>
        /// A generated-route session keeps only the active goal's plans of the current epoch; a recorded-route
        /// session has no goal ids and keeps every plan.
        /// </summary>
        void ApplyPlans(Frame frame)
        {
            if (GeneratedRoutes)
            {
                if (FromOlderEpoch(frame.Header)) return;
                if (!ForCurrentGoal((long?)frame.Header["goal_id"] ?? 0)) return;
            }
            Plan = MediumPlan.FromFrame(frame);
        }

        /// <summary>
        /// A Plans or Route record belongs to the goal being driven: a goal newer than the last one known (a reconnect
        /// whose replayed Goal frame carried no goal, review I1), or the known goal while it is active. Records of an
        /// ended or replaced goal are refused.
        /// </summary>
        bool ForCurrentGoal(long goalId) => goalId > Goal.GoalId || (goalId == Goal.GoalId && Goal.HasActiveGoal);

        bool FromOlderEpoch(JObject header)
        {
            var epoch = (long?)header["epoch"];
            if (!epoch.HasValue) return false;
            if (epoch.Value < Epoch) return true;
            Epoch = epoch.Value;
            return false;
        }

        void StartSession(JObject hello)
        {
            ClearSession();
            SessionHello = hello;
            Epoch = (long?)hello["epoch"] ?? 0;
            GeneratedRoutes = (bool?)hello["config"]?["generated_routes"] ?? false;
            HasMediumPlans = (bool?)hello["config"]?["medium"] ?? (bool?)hello["medium"] ?? false;
            HasLidar = (bool?)hello["config"]?["lidar"] ?? false;
            if (hello["capabilities"] is JArray caps)
                foreach (var c in caps) capabilities.Add((string)c);
            Boxes = hello["boxes"] as JArray ?? new JArray();
        }

        void ClearSession()
        {
            SessionHello = null;
            SessionIsReplay = false;
            Plan = null;
            HasMediumPlans = false;
            HasLidar = false;
            Epoch = 0;
            GeneratedRoutes = false;
            capabilities.Clear();
            Boxes = new JArray();
            ClearStates();
            speedSamples.Clear();
            movers.Clear();
            bodies.Clear();
            ClearRoute();
            Goal.Clear();
            lastStateWall = double.NegativeInfinity;
        }

        void ClearStates()
        {
            states.Clear();
            stateWalls.Clear();
            lastDisplay = double.NegativeInfinity;
            lastSampleWall = double.NaN;
        }

        void ClearRoute()
        {
            routePath.Clear();
            RouteId = 0;
            routeGoalId = 0;
        }

        void ApplyState(Frame frame, double wallNow)
        {
            var t = (double)frame.Header["t"];
            if (frame.Blocks.Length > 0 && frame.Blocks[0].Length >= 9)
            {
                var b = frame.Blocks[0];
                var joints = new double[6];
                for (var j = 0; j < 6; ++j) joints[j] = b[3 + j];
                double v = 0, omega = 0;
                if (frame.Blocks.Length > 2 && frame.Blocks[2].Length >= 2)
                {
                    v = frame.Blocks[2][0];
                    omega = frame.Blocks[2][1];
                }
                if (states.Count > 0 && t < states[states.Count - 1].T) ClearStates();  // the clock restarted
                states.Add(new RobotSample(t, b[0], b[1], b[2], v, omega, joints));
                stateWalls.Add(wallNow);
                if (states.Count > MaxStates)
                {
                    states.RemoveAt(0);
                    stateWalls.RemoveAt(0);
                }
            }
            lastStateWall = wallNow;
            speedSamples.Enqueue((wallNow, t));
            while (speedSamples.Count > 0 && speedSamples.Peek().wall < wallNow - SpeedWindowS) speedSamples.Dequeue();

            if (frame.Header["movers"] is JArray m)
            {
                movers.Clear();
                foreach (var r in m) movers.Add(((string)r[0], (double)r[1], (double)r[2]));
            }
            if (frame.Header["bodies"] is JArray bs)
            {
                bodies.Clear();
                foreach (var r in bs) bodies.Add(((string)r[0], (double)r[1], (double)r[2], (double)r[3]));
            }
        }

        // Drawn only for the backend's active goal (Task 3 review I3): the reconnect replay sends the latest Route even
        // when its goal has ended.
        void ApplyRoute(Frame frame)
        {
            if (!ForCurrentGoal((long?)frame.Header["goal_id"] ?? 0)) return;
            routePath.Clear();
            var poses = frame.Blocks.Length > 0 ? frame.Blocks[0] : Array.Empty<float>();
            for (var i = 0; i + 2 < poses.Length; i += 3) routePath.Add((poses[i], poses[i + 1], poses[i + 2]));
            RouteId = (long?)frame.Header["route_id"] ?? 0;
            routeGoalId = (long?)frame.Header["goal_id"] ?? 0;
        }

        /// <summary>
        /// The robot at the display time, interpolated between the bracketing states. While running, the display
        /// time advances every frame with wall time at the rate the States arrived over the last two seconds, and is
        /// pulled toward <paramref name="lag"/> behind the newest State (it jumps there when more than 0.5 s off);
        /// paused, it is the newest State. Never past the newest State, never backwards.
        /// </summary>
        public bool TrySample(double lag, double wallNow, out RobotSample sample)
        {
            sample = default;
            if (states.Count == 0) return false;
            var newest = states.Count - 1;
            var oldest = newest;
            var windowStart = Math.Max(stateWalls[newest] - SpeedWindowS, rateSince);
            while (oldest > 0 && stateWalls[oldest - 1] >= windowStart) --oldest;
            var wallSpan = stateWalls[newest] - stateWalls[oldest];
            var rate = !double.IsNaN(rateHint) && Math.Max(wallNow, stateWalls[newest]) - rateSince < RateSettleS
                ? rateHint
                : wallSpan > 0 ? (states[newest].T - states[oldest].T) / wallSpan : 0.0;
            // The display clock runs at the arrival rate every frame and is pulled gently toward newest - lag, so
            // arrival jitter (8-45 ms gaps around 20 ms, measured) does not become jumps and freezes, as it did when
            // the clock was re-anchored on each arrival. Paused, the newest State is shown exactly (a step at once).
            double target;
            var goal = states[newest].T - lag;
            if (!running) target = states[newest].T;
            else if (double.IsNegativeInfinity(lastDisplay) || double.IsNaN(lastSampleWall) ||
                     Math.Abs(goal - lastDisplay) > ClockSnapS)
                target = goal;
            else
            {
                var dt = Math.Max(0.0, wallNow - lastSampleWall);
                target = lastDisplay + rate * dt + ClockGain * (goal - lastDisplay) * Math.Min(dt, 0.25);
            }
            if (!double.IsNaN(lastSampleWall) && wallNow < lastSampleWall) target = lastDisplay;  // wall time went back
            target = Math.Min(states[newest].T, Math.Max(target, lastDisplay));
            lastDisplay = target;
            lastSampleWall = double.IsNaN(lastSampleWall) ? wallNow : Math.Max(lastSampleWall, wallNow);
            if (target <= states[0].T)
            {
                sample = states[0];
                return true;
            }
            for (var i = states.Count - 1; i > 0; --i)
            {
                var a = states[i - 1];
                var b = states[i];
                if (target < a.T) continue;
                var span = b.T - a.T;
                var s = span > 0 ? Math.Min(1.0, (target - a.T) / span) : 1.0;
                var joints = new double[6];
                for (var j = 0; j < 6; ++j) joints[j] = a.Joints[j] + s * Angle(b.Joints[j] - a.Joints[j]);
                sample = new RobotSample(target, a.X + s * (b.X - a.X), a.Y + s * (b.Y - a.Y),
                    Angle(a.Yaw + s * Angle(b.Yaw - a.Yaw)), a.V + s * (b.V - a.V), a.Omega + s * (b.Omega - a.Omega), joints);
                return true;
            }
            sample = states[states.Count - 1];
            return true;
        }

        /// <summary>Simulation time over wall time across the last two wall seconds of State frames.</summary>
        public double AchievedSpeed(double wallNow)
        {
            while (speedSamples.Count > 0 && speedSamples.Peek().wall < wallNow - SpeedWindowS) speedSamples.Dequeue();
            if (speedSamples.Count < 2) return 0.0;
            var first = speedSamples.Peek();
            (double wall, double sim) last = default;
            foreach (var s in speedSamples) last = s;
            var wall = last.wall - first.wall;
            return wall > 0 ? (last.sim - first.sim) / wall : 0.0;
        }

        static double Angle(double a) => Math.IEEERemainder(a, 2 * Math.PI);
    }
}
