using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    public enum MarkerState
    {
        None,
        Pending,
        Planning,
        Tracking,
        Rejected,
        Superseded,
        Reached,
        Failed,
        TimedOut,
        Canceled,
    }

    /// <summary>
    /// What the goal marker shows (design section 6.5). A click is only <see cref="MarkerState.Pending"/> until the
    /// backend answers; the displayed goal follows the backend's Goal frames, so a replacement shows only after the
    /// backend supersedes the old goal, and a rejected replacement leaves the active goal on screen.
    /// </summary>
    public sealed class GoalMarkerModel
    {
        long pendingRequest = -1;
        (double x, double y, double yaw)? pendingTarget;
        long goalId;
        MarkerState goalState = MarkerState.None;
        (double x, double y, double yaw)? goalTarget;

        public string Reason { get; private set; } = "";
        public long GoalId => goalId;

        /// <summary>The backend's current goal is planning or tracking.</summary>
        public bool HasActiveGoal => Active(goalState);

        /// <summary>The submitted destination the backend has not answered yet, if any.</summary>
        public (double x, double y, double yaw)? PendingTarget => pendingTarget;

        public MarkerState State =>
            Active(goalState) ? goalState : pendingTarget.HasValue ? MarkerState.Pending : goalState;

        public (double x, double y, double yaw)? Target =>
            Active(goalState) ? goalTarget : pendingTarget ?? goalTarget;

        static bool Active(MarkerState s) => s == MarkerState.Planning || s == MarkerState.Tracking;

        public void NoteSubmitted(long clientRequest, double x, double y, double yaw)
        {
            pendingRequest = clientRequest;
            pendingTarget = (x, y, yaw);
            Reason = "";
        }

        /// <summary>Applies a <c>goal_ack</c> event; only the answer to the pending submission matters.</summary>
        public void ApplyAck(JObject ack)
        {
            var command = (string)ack["command"];
            if (command != "submit_goal" && command != "schedule_goal") return;
            if (!pendingTarget.HasValue || (long?)ack["client_request"] != pendingRequest) return;
            var action = (string)ack["action"];
            if (action == "accepted" || action == "scheduled" || action == "duplicate") return;  // a Goal frame follows
            Refuse(action == "rejected" ? (string)ack["reason"] : $"{action}: {(string)ack["reason"]}");
        }

        public void ApplyGoalFrame(JObject header)
        {
            var evt = (string)header["event"];
            if (evt == "session_reset")
            {
                Clear();
                return;
            }
            var matchesPending = pendingTarget.HasValue && (long?)header["client_request"] == pendingRequest;
            if (header["goal"] is JObject goal)
            {
                var id = (long)goal["goal_id"];
                if (id < goalId) return;  // an older goal's late record
                goalId = id;
                goalState = Parse((string)goal["state"]);
                var t = goal["target"];
                goalTarget = ((double)t[0], (double)t[1], (double)t[2]);
                if (goalState == MarkerState.Failed || goalState == MarkerState.TimedOut) Reason = (string)header["reason"] ?? "";
                if (matchesPending) pendingTarget = null;
            }
            else if (matchesPending && evt == "goal_rejected")
            {
                Refuse((string)header["reason"]);
            }
            else if (matchesPending && evt == "goal_canceled")
            {
                pendingTarget = null;  // a scheduled submission canceled before it applied
            }
        }

        void Refuse(string reason)
        {
            Reason = reason ?? "";
            if (!Active(goalState))
            {
                goalState = MarkerState.Rejected;
                goalTarget = pendingTarget;
            }
            pendingTarget = null;
        }

        public void Clear()
        {
            pendingRequest = -1;
            pendingTarget = null;
            goalId = 0;
            goalState = MarkerState.None;
            goalTarget = null;
            Reason = "";
        }

        static MarkerState Parse(string state)
        {
            switch (state)
            {
                case "planning": return MarkerState.Planning;
                case "tracking": return MarkerState.Tracking;
                case "reached": return MarkerState.Reached;
                case "failed": return MarkerState.Failed;
                case "timed_out": return MarkerState.TimedOut;
                case "superseded": return MarkerState.Superseded;
                case "canceled": return MarkerState.Canceled;
                default: return MarkerState.None;
            }
        }
    }
}
