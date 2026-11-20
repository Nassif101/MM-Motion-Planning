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
            s.Apply(server, 0.1);  // the connect-time server Hello is not a session header: it ends the old session
            Assert.AreEqual(0, s.Epoch);
            Assert.IsFalse(s.Has("cancel_goal"));
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
            s.Apply(new Frame { Type = FrameType.Goal, Header = JObject.Parse(
                "{\"event\":\"goal_accepted\",\"epoch\":2,\"goal\":{\"goal_id\":4,\"state\":\"planning\",\"target\":[1,2,0]}}") }, 0.0);
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
            s.Apply(new Frame { Type = FrameType.Goal, Header = JObject.Parse(
                "{\"event\":\"goal_accepted\",\"epoch\":2,\"goal\":{\"goal_id\":4,\"state\":\"planning\",\"target\":[1,2,0]}}") }, 0.0);
            s.Apply(Route(2, 8), 0.0);
            Assert.AreEqual(2, s.RoutePath.Count);
            Assert.AreEqual(1, s.Movers.Count);
            s.Apply(SessionHello(3), 1.0);
            Assert.AreEqual(0, s.RoutePath.Count);
            Assert.AreEqual(MarkerState.None, s.Goal.State);
            Assert.IsFalse(s.TrySample(0.04, 1.0, out _));
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
            Assert.IsTrue(s.TrySample(0.04, 0.0, out var sample));
            Assert.AreEqual(1.02, sample.T, 1e-9);
            Assert.AreEqual(1.0, sample.X, 1e-6);
            Assert.AreEqual(0.4, sample.V, 1e-6);
            Assert.AreEqual(6, sample.Joints.Length);
            Assert.AreEqual(0.6, sample.Joints[5], 1e-6);

            var w = new ViewerState();  // yaw interpolates the short way across +-pi
            w.Apply(State(0.00, 0f, 3.1f), 0.0);
            w.Apply(State(0.02, 0f, -3.1f), 0.0);
            w.Apply(State(0.05, 0f, -3.1f), 0.0);
            Assert.IsTrue(w.TrySample(0.04, 0.0, out var across));
            Assert.AreEqual(0.01, across.T, 1e-9);
            Assert.AreEqual(0.0, Math.Abs(Math.IEEERemainder(across.Yaw - Math.PI, 2 * Math.PI)), 1e-3);
        }

        // Task 3 review I2: between State frames the display time advances with wall time at the achieved speed,
        // so motion is interpolated rather than stepped at the frame rate; it never passes the newest State
        [Test]
        public void SampleAdvancesBetweenStatesWithWallTime()
        {
            var s = new ViewerState();
            for (var k = 0; k <= 5; ++k) s.Apply(State(1.0 + 0.02 * k, k), 10.0 + 0.08 * k);  // 0.25x: 20 ms per 80 ms
            Assert.IsTrue(s.TrySample(0.04, 10.40, out var a));
            Assert.AreEqual(1.06, a.T, 1e-6);
            Assert.IsTrue(s.TrySample(0.04, 10.44, out var b));
            Assert.AreEqual(1.07, b.T, 1e-6);
            Assert.AreEqual(3.5, b.X, 1e-4);  // halfway between the States at 1.06 and 1.08
            Assert.IsTrue(s.TrySample(0.04, 20.0, out var late));
            Assert.AreEqual(1.10, late.T, 1e-6);  // clamped at the newest State: no extrapolation
            Assert.IsTrue(s.TrySample(0.04, 10.45, out var back));
            Assert.AreEqual(1.10, back.T, 1e-6);  // the display time never runs backwards
        }

        // Task 3 review I1: a reconnect's server Hello means the old session may be gone; only the server's replay
        // (a session Hello, then its latest Goal and Route) may restore it
        [Test]
        public void ServerHelloAfterReconnectClearsTheOldSession()
        {
            var s = new ViewerState();
            s.Apply(SessionHello(2), 0.0);
            s.Apply(State(1.0, 0f), 0.0);
            s.Apply(new Frame { Type = FrameType.Goal, Header = JObject.Parse(
                "{\"event\":\"goal_accepted\",\"epoch\":2,\"goal\":{\"goal_id\":4,\"state\":\"planning\",\"target\":[1,2,0]}}") }, 0.0);
            s.Apply(Route(2, 8), 0.0);
            s.Apply(new Frame { Type = FrameType.Hello, Header = new JObject { ["schema"] = 3, ["server"] = "x" } }, 5.0);
            Assert.IsNull(s.SessionHello);
            Assert.IsFalse(s.GeneratedRoutes);
            Assert.IsFalse(s.Has("cancel_goal"));
            Assert.AreEqual(0, s.RoutePath.Count);
            Assert.AreEqual(MarkerState.None, s.Goal.State);
            Assert.IsFalse(s.TrySample(0.04, 5.0, out _));
        }

        // Task 3 review I3: the replay sends the latest Goal frame, then the latest Route; a finished goal's route
        // is not drawn again
        [Test]
        public void RouteOfAFinishedGoalIsNotDrawn()
        {
            var s = new ViewerState();
            s.Apply(SessionHello(2), 0.0);
            s.Apply(new Frame { Type = FrameType.Goal, Header = JObject.Parse(
                "{\"event\":\"goal_reached\",\"epoch\":2,\"goal\":{\"goal_id\":4,\"state\":\"reached\",\"target\":[1,2,0]}}") }, 0.0);
            s.Apply(Route(2, 8), 0.0);
            Assert.AreEqual(0, s.RoutePath.Count);
            s.Apply(new Frame { Type = FrameType.Goal, Header = JObject.Parse(
                "{\"event\":\"goal_rejected\",\"epoch\":2,\"goal\":null,\"request\":{\"target\":[9,9,0]}}") }, 0.0);
            s.Apply(Route(2, 9), 0.0);  // no goal known to be active: nothing drawn
            Assert.AreEqual(0, s.RoutePath.Count);
        }

        [Test]
        public void StaleOnlyWhileRunning()
        {
            var s = new ViewerState();
            s.Apply(State(1.0, 0f), 10.0);
            s.NoteRunning(true, 10.0);
            Assert.IsTrue(s.IsStale(10.3));
            Assert.IsFalse(s.IsStale(10.2));
            s.NoteRunning(false, 10.3);
            Assert.IsFalse(s.IsStale(10.3));
        }

        // Task 5 review I2: a run that has not sent its first State is stale 0.25 s after the request, not forever
        [Test]
        public void StaleCountsFromTheRunRequest()
        {
            var s = new ViewerState();
            s.Apply(State(1.0, 0f), 1.0);
            s.NoteRunning(true, 5.0);
            Assert.IsFalse(s.IsStale(5.2));
            Assert.IsTrue(s.IsStale(5.3));
            Assert.AreEqual(0.3, s.SilentFor(5.3), 1e-9);
        }

        // Task 5 review I2: the server stops running on a new connection, a reset, and run_stopped
        [Test]
        public void HellosAndRunStoppedEndTheRun()
        {
            var s = new ViewerState();
            var server = new Frame { Type = FrameType.Hello, Header = new JObject { ["schema"] = 3, ["server"] = "x" } };
            var stopped = new Frame { Type = FrameType.Event, Header = new JObject { ["event"] = "run_stopped", ["reason"] = "goals_done" } };
            foreach (var f in new[] { server, SessionHello(1), stopped })
            {
                s.NoteRunning(true, 0.0);
                s.Apply(f, 0.1);
                Assert.IsFalse(s.Running, f.Type.ToString());
                Assert.IsFalse(s.IsStale(9.0));
            }
        }

        // Task 6 review I2: before the first State (a reset pauses the server), a click's heading needs the start pose
        [Test]
        public void RobotPoseFallsBackToTheSessionStart()
        {
            var s = new ViewerState();
            Assert.IsFalse(s.TryRobotPose(out _));
            var hello = SessionHello(1);
            hello.Header["start"] = new JArray(-6.0, -3.5, -0.5);
            s.Apply(hello, 0.0);
            Assert.IsTrue(s.TryRobotPose(out var p));
            Assert.AreEqual((-6.0, -3.5, -0.5), p);
            s.Apply(State(1.0, 2f, yaw: 0.25f), 0.1);
            Assert.IsTrue(s.TryRobotPose(out p));
            Assert.AreEqual(2.0, p.x, 1e-6);
            Assert.AreEqual(0.25, p.yaw, 1e-6);
        }

        // Task 7 review I1: a replayed session's start pose is where the robot was at the last reset, not where it is
        [Test]
        public void AReplayedSessionHasNoPoseUntilItsFirstState()
        {
            var s = new ViewerState();
            var server = new Frame { Type = FrameType.Hello, Header = new JObject { ["schema"] = 3, ["server"] = "x" } };
            var hello = SessionHello(1);
            hello.Header["start"] = new JArray(-6.0, -3.5, -0.5);
            s.Apply(server, 0.0);
            s.Apply(hello, 0.0);  // the server's replay after a (re)connect
            Assert.IsTrue(s.SessionIsReplay);
            Assert.IsFalse(s.TryRobotPose(out _));
            s.Apply(State(1.0, 2f), 0.1);
            Assert.IsTrue(s.TryRobotPose(out var p));
            Assert.AreEqual(2.0, p.x, 1e-6);
        }

        // Task 6 review I3: a restarted server with no session sends no replay; the Hello answering our reset is new
        [Test]
        public void AResetSentAfterAConnectIsNotAReplay()
        {
            var s = new ViewerState();
            var server = new Frame { Type = FrameType.Hello, Header = new JObject { ["schema"] = 3, ["server"] = "x" } };
            s.Apply(server, 0.0);
            s.NoteResetSent();
            s.Apply(SessionHello(1), 0.0);
            Assert.IsFalse(s.SessionIsReplay);
        }

        // Task 5 review I3: only the session Hello that follows a connect is a replay; a reset's is a new session
        [Test]
        public void OnlyTheSessionAfterAConnectIsAReplay()
        {
            var s = new ViewerState();
            var server = new Frame { Type = FrameType.Hello, Header = new JObject { ["schema"] = 3, ["server"] = "x" } };
            s.Apply(server, 0.0);
            s.Apply(SessionHello(1), 0.0);
            Assert.IsTrue(s.SessionIsReplay);
            s.Apply(SessionHello(1), 0.0);
            Assert.IsFalse(s.SessionIsReplay);
            s.Apply(server, 0.0);
            s.Apply(SessionHello(1), 0.0);
            Assert.IsTrue(s.SessionIsReplay);
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
