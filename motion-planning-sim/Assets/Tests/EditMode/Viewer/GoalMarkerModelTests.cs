using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class GoalMarkerModelTests
    {
        static JObject GoalFrame(string evt, long goalId, string state, long clientRequest, double x = 1.0) =>
            JObject.Parse($"{{\"event\":\"{evt}\",\"epoch\":1,\"client_request\":{clientRequest}," +
                          $"\"goal\":{{\"goal_id\":{goalId},\"state\":\"{state}\",\"target\":[{x},2.0,0.0]}}}}");

        static JObject Ack(string action, long clientRequest, string reason = "") =>
            JObject.Parse($"{{\"event\":\"goal_ack\",\"command\":\"submit_goal\",\"action\":\"{action}\"," +
                          $"\"reason\":\"{reason}\",\"client_request\":{clientRequest},\"goal_id\":0}}");

        [Test]
        public void MarkerPendingUntilBackendAnswers()
        {
            var m = new GoalMarkerModel();
            Assert.AreEqual(MarkerState.None, m.State);
            m.NoteSubmitted(1, 101.0, -2.25, 0.5);
            Assert.AreEqual(MarkerState.Pending, m.State);
            Assert.AreEqual((101.0, -2.25, 0.5), m.Target);
            m.ApplyAck(Ack("rejected", 1, "out_of_map"));
            Assert.AreEqual(MarkerState.Rejected, m.State);
            Assert.AreEqual("out_of_map", m.Reason);

            Assert.IsTrue(FrameCodec.TryDecode(ViewerFixtures.Read("goal_v3.bin"), out var goal, out _));
            m.ApplyGoalFrame(goal.Header);
            Assert.AreEqual(MarkerState.Planning, m.State);
            Assert.AreEqual(4, m.GoalId);
            Assert.AreEqual((1.5, -2.25, 0.5), m.Target);
        }

        [Test]
        public void AcceptedAckKeepsPendingUntilTheGoalFrame()
        {
            var m = new GoalMarkerModel();
            m.NoteSubmitted(1, 1.0, 2.0, 0.0);
            m.ApplyAck(Ack("accepted", 1));
            Assert.AreEqual(MarkerState.Pending, m.State);
            m.ApplyGoalFrame(GoalFrame("goal_accepted", 4, "planning", 1));
            Assert.AreEqual(MarkerState.Planning, m.State);
            m.ApplyGoalFrame(GoalFrame("route_accepted", 4, "tracking", 1));
            Assert.AreEqual(MarkerState.Tracking, m.State);
            m.ApplyGoalFrame(GoalFrame("goal_reached", 4, "reached", 1));
            Assert.AreEqual(MarkerState.Reached, m.State);
        }

        [Test]
        public void MarkerFollowsSupersedeNotTheClick()
        {
            var m = new GoalMarkerModel();
            m.NoteSubmitted(1, 1.0, 2.0, 0.0);
            m.ApplyGoalFrame(GoalFrame("route_accepted", 4, "tracking", 1));
            m.NoteSubmitted(2, 5.0, 2.0, 0.0);
            Assert.AreEqual(MarkerState.Tracking, m.State);  // the old goal is still the backend's
            Assert.AreEqual(4, m.GoalId);
            Assert.AreEqual((5.0, 2.0, 0.0), m.PendingTarget);
            m.ApplyGoalFrame(GoalFrame("goal_superseded", 4, "superseded", 1));
            Assert.AreEqual(MarkerState.Pending, m.State);
            m.ApplyGoalFrame(GoalFrame("goal_accepted", 5, "planning", 2, x: 5.0));
            Assert.AreEqual(MarkerState.Planning, m.State);
            Assert.AreEqual(5, m.GoalId);
            Assert.IsNull(m.PendingTarget);
        }

        [Test]
        public void RejectedReplacementLeavesTheActiveGoal()
        {
            var m = new GoalMarkerModel();
            m.ApplyGoalFrame(GoalFrame("route_accepted", 4, "tracking", 1));
            m.NoteSubmitted(2, 101.0, 2.0, 0.0);
            m.ApplyAck(Ack("rejected", 2, "out_of_map"));
            Assert.AreEqual(MarkerState.Tracking, m.State);
            Assert.AreEqual(4, m.GoalId);
            Assert.AreEqual("out_of_map", m.Reason);
            Assert.IsNull(m.PendingTarget);
        }

        [Test]
        public void CancelAndResetClearTheMarker()
        {
            var m = new GoalMarkerModel();
            m.ApplyGoalFrame(GoalFrame("route_accepted", 4, "tracking", 1));
            m.ApplyGoalFrame(GoalFrame("goal_canceled", 4, "canceled", 1));
            Assert.AreEqual(MarkerState.Canceled, m.State);
            m.ApplyGoalFrame(JObject.Parse("{\"event\":\"session_reset\",\"epoch\":2,\"goal\":null}"));
            Assert.AreEqual(MarkerState.None, m.State);
        }
    }
}
