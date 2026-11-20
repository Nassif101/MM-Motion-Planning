using System;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class ViewerStateTests
    {
        static Frame SessionHello(long epoch) => new Frame
        {
            Type = FrameType.Hello,
            Header = new JObject
            {
                ["schema"] = 3, ["scenario"] = "v2_open_yard_nav", ["seed"] = 1, ["epoch"] = epoch,
                ["config"] = new JObject { ["generated_routes"] = true },
                ["capabilities"] = new JArray("goal_frames", "route_frames", "cancel_goal"),
            },
            Blocks = Array.Empty<float[]>(),
        };

        static Frame State(double t, float x, float yaw = 0f, float v = 0f) => new Frame
        {
            Type = FrameType.State,
            Header = new JObject
            {
                ["t"] = t, ["movers"] = new JArray { new JArray("worker", 1.0, 2.0) },  // an initializer nests; the constructor would flatten
                ["bodies"] = new JArray { new JArray("reserved/0", 3.0, 4.0, 0.5) },
            },
            Blocks = new[]
            {
                new[] { x, 0f, yaw, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f },
                new float[9],
                new[] { v, 0.05f, 0f, 0f, 0f, 0f, 0f, 0f },
            },
        };

        static Frame Route(long epoch, long routeId) => new Frame
        {
            Type = FrameType.Route,
            Header = new JObject { ["epoch"] = epoch, ["goal_id"] = 4, ["route_id"] = routeId, ["path_poses"] = 2 },
            Blocks = new[] { new[] { 0f, 0f, 0f, 1f, 2f, 0.5f }, new[] { 0f, 0f, 1f, 2f } },
        };

        [Test]
        public void SessionHelloSetsIdentityAndCapabilities()
        {
            var s = new ViewerState();
            s.Apply(SessionHello(2), 0.0);
            Assert.AreEqual(2, s.Epoch);
            Assert.IsTrue(s.GeneratedRoutes);
            Assert.IsTrue(s.Has("cancel_goal"));
            Assert.IsFalse(s.Has("teleport"));
            var server = new Frame { Type = FrameType.Hello, Header = new JObject { ["schema"] = 3, ["server"] = "x" } };
            s.Apply(server, 0.1);  // the connect-time server Hello is not a session header
            Assert.AreEqual(2, s.Epoch);
        }

        [Test]
        public void IgnoresGoalAndRouteFromOlderEpoch()
        {
            var s = new ViewerState();
            s.Apply(SessionHello(2), 0.0);
            s.Apply(Route(1, 7), 0.1);
            Assert.AreEqual(0, s.RoutePath.Count);
            s.Apply(new Frame
            {
                Type = FrameType.Goal,
                Header = JObject.Parse("{\"event\":\"goal_accepted\",\"epoch\":1,\"client_request\":1," +
                                       "\"goal\":{\"goal_id\":3,\"state\":\"planning\",\"target\":[1,2,0]}}"),
            }, 0.1);
            Assert.AreEqual(MarkerState.None, s.Goal.State);
            s.Apply(Route(2, 8), 0.2);
            Assert.AreEqual(2, s.RoutePath.Count);
            Assert.AreEqual(8, s.RouteId);
            Assert.AreEqual((1.0, 2.0, 0.5), (s.RoutePath[1].x, s.RoutePath[1].y, Math.Round(s.RoutePath[1].yaw, 6)));
        }

        [Test]
        public void SessionHelloClearsSessionState()
        {
            var s = new ViewerState();
            s.Apply(SessionHello(2), 0.0);
            s.Apply(State(1.0, 0f), 0.0);
            s.Apply(Route(2, 8), 0.0);
            s.Apply(new Frame { Type = FrameType.Goal, Header = JObject.Parse(
                "{\"event\":\"goal_accepted\",\"epoch\":2,\"goal\":{\"goal_id\":4,\"state\":\"planning\",\"target\":[1,2,0]}}") }, 0.0);
            Assert.AreEqual(1, s.Movers.Count);
            s.Apply(SessionHello(3), 1.0);
            Assert.AreEqual(0, s.RoutePath.Count);
            Assert.AreEqual(MarkerState.None, s.Goal.State);
            Assert.IsFalse(s.TrySample(0.04, out _));
            Assert.AreEqual(0, s.Movers.Count);
            Assert.AreEqual(3, s.Epoch);
        }

        [Test]
        public void SampleInterpolatesAtLag()
        {
            var s = new ViewerState();
            s.Apply(State(1.00, 0f), 0.0);
            s.Apply(State(1.02, 1f, 0f, 0.4f), 0.0);
            s.Apply(State(1.06, 3f), 0.0);
            Assert.IsTrue(s.TrySample(0.04, out var sample));
            Assert.AreEqual(1.02, sample.T, 1e-9);
            Assert.AreEqual(1.0, sample.X, 1e-6);
            Assert.AreEqual(0.4, sample.V, 1e-6);
            Assert.AreEqual(6, sample.Joints.Length);
            Assert.AreEqual(0.6, sample.Joints[5], 1e-6);

            var w = new ViewerState();  // yaw interpolates the short way across +-pi
            w.Apply(State(0.00, 0f, 3.1f), 0.0);
            w.Apply(State(0.02, 0f, -3.1f), 0.0);
            w.Apply(State(0.05, 0f, -3.1f), 0.0);
            Assert.IsTrue(w.TrySample(0.04, out var across));
            Assert.AreEqual(0.01, across.T, 1e-9);
            Assert.AreEqual(0.0, Math.Abs(Math.IEEERemainder(across.Yaw - Math.PI, 2 * Math.PI)), 1e-3);
        }

        [Test]
        public void StaleOnlyWhileRunning()
        {
            var s = new ViewerState();
            s.Apply(State(1.0, 0f), 10.0);
            s.NoteRunning(true);
            Assert.IsTrue(s.IsStale(10.3));
            Assert.IsFalse(s.IsStale(10.2));
            s.NoteRunning(false);
            Assert.IsFalse(s.IsStale(10.3));
        }

        [Test]
        public void MoversBodiesAndAchievedSpeed()
        {
            var s = new ViewerState();
            for (var k = 0; k <= 100; ++k) s.Apply(State(k * 0.02, 0f), 10.0 + k * 0.01);  // 2 s sim in 1 s wall
            Assert.AreEqual(("worker", 1.0, 2.0), s.Movers[0]);
            Assert.AreEqual(("reserved/0", 3.0, 4.0, 0.5), s.Bodies[0]);
            Assert.AreEqual(2.0, s.AchievedSpeed(11.0), 1e-6);
        }
    }
}
