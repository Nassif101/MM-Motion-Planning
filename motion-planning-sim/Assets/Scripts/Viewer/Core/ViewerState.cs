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
        const int MaxStates = 64;
        const double SpeedWindowS = 2.0;
        const double StaleAfterS = 0.25;

        readonly List<RobotSample> states = new List<RobotSample>();
        readonly List<double> stateWalls = new List<double>();  // each State's arrival wall time
        double lastDisplay = double.NegativeInfinity;
        readonly Queue<(double wall, double sim)> speedSamples = new Queue<(double wall, double sim)>();
        readonly HashSet<string> capabilities = new HashSet<string>();
        readonly List<(string name, double x, double y)> movers = new List<(string, double, double)>();
        readonly List<(string name, double x, double y, double yaw)> bodies = new List<(string, double, double, double)>();
        readonly List<(double x, double y, double yaw)> routePath = new List<(double, double, double)>();
        double lastStateWall = double.NegativeInfinity;
        bool running;
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

        /// <summary>The latest event frames (scenarios, errors, goal acks) for presenters; drained by the caller.</summary>
        public Queue<JObject> Events { get; } = new Queue<JObject>();

        public bool Has(string capability) => capabilities.Contains(capability);
        public void NoteRunning(bool isRunning) => running = isRunning;
        public bool IsStale(double wallNow) => running && wallNow - lastStateWall > StaleAfterS;

        public void Apply(Frame frame, double wallNow)
        {
            switch (frame.Type)
            {
                case FrameType.Hello:
                    // A session Hello starts a session; the connect-time server Hello means the old session may be
                    // gone (Task 3 review I1): only the server's replay may restore it.
                    if (frame.Header["scenario"] != null) StartSession(frame.Header);
                    else ClearSession();
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
                    break;
                case FrameType.Route:
                    if (FromOlderEpoch(frame.Header)) break;
                    ApplyRoute(frame);
                    break;
                case FrameType.Event:
                    if ((string)frame.Header["event"] == "goal_ack") Goal.ApplyAck(frame.Header);
                    if (Events.Count >= 256) Events.Dequeue();
                    Events.Enqueue(frame.Header);
                    break;
            }
        }

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
            if (hello["capabilities"] is JArray caps)
                foreach (var c in caps) capabilities.Add((string)c);
            Boxes = hello["boxes"] as JArray ?? new JArray();
        }

        void ClearSession()
        {
            SessionHello = null;
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
            var goalId = (long?)frame.Header["goal_id"] ?? 0;
            if (!Goal.HasActiveGoal || goalId != Goal.GoalId) return;
            routePath.Clear();
            var poses = frame.Blocks.Length > 0 ? frame.Blocks[0] : Array.Empty<float>();
            for (var i = 0; i + 2 < poses.Length; i += 3) routePath.Add((poses[i], poses[i + 1], poses[i + 2]));
            RouteId = (long?)frame.Header["route_id"] ?? 0;
            routeGoalId = (long?)frame.Header["goal_id"] ?? 0;
        }

        /// <summary>
        /// The robot at the display time, interpolated between the bracketing states. The display time runs
        /// <paramref name="lag"/> behind the newest State and advances between frames with wall time at the rate the
        /// States arrived over the last two seconds (Task 3 review I2), never past the newest State and never
        /// backwards.
        /// </summary>
        public bool TrySample(double lag, double wallNow, out RobotSample sample)
        {
            sample = default;
            if (states.Count == 0) return false;
            var newest = states.Count - 1;
            var oldest = newest;
            while (oldest > 0 && stateWalls[oldest - 1] >= stateWalls[newest] - SpeedWindowS) --oldest;
            var wallSpan = stateWalls[newest] - stateWalls[oldest];
            var rate = wallSpan > 0 ? (states[newest].T - states[oldest].T) / wallSpan : 0.0;
            var target = states[newest].T + Math.Max(0.0, wallNow - stateWalls[newest]) * rate - lag;
            target = Math.Min(states[newest].T, Math.Max(target, lastDisplay));
            lastDisplay = target;
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
