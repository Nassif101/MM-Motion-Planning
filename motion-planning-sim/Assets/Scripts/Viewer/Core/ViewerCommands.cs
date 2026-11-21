using System;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// The Control commands the viewer sends (stream-protocol.md). <c>client_request</c> increases by one for each
    /// goal command, so the backend's <c>goal_ack</c> can be matched to the request that caused it.
    /// It begins at Unix milliseconds, never at 1.
    /// </summary>
    public sealed class ViewerCommands
    {
        public const int Schema = 3;

        /// <summary>
        /// Starts at the clock's Unix milliseconds (Task 2 review I1): the server files every connection under one
        /// client and keeps its request watermark for the whole session, so a relaunched viewer must continue above
        /// the numbers an earlier process used.
        /// </summary>
        public ViewerCommands(Func<long> unixMilliseconds = null)
        {
            LastClientRequest = (unixMilliseconds ?? (() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()))();
        }

        public long LastClientRequest { get; private set; }

        /// <summary>4B-2: the hello accepts Prediction frames, which the demo server sends only to a client that asks.</summary>
        public JObject Hello() =>
            new JObject { ["cmd"] = "hello", ["schema"] = Schema, ["accepts"] = new JArray("prediction") };
        public JObject ListScenarios() => new JObject { ["cmd"] = "list_scenarios" };
        public JObject Reset(string scenario, int seed) =>
            new JObject { ["cmd"] = "reset", ["scenario"] = scenario, ["seed"] = seed };
        public JObject Run(double speed) => new JObject { ["cmd"] = "run", ["speed"] = speed };
        public JObject Pause() => new JObject { ["cmd"] = "pause" };
        public JObject Step(int n = 1) => new JObject { ["cmd"] = "step", ["n"] = n };

        public JObject SubmitGoal(double x, double y, double yaw) => new JObject
        {
            ["cmd"] = "submit_goal",
            ["target"] = new JArray(x, y, yaw),
            ["client_request"] = ++LastClientRequest,
        };

        public JObject CancelGoal() => new JObject { ["cmd"] = "cancel_goal", ["client_request"] = ++LastClientRequest };
    }

    /// <summary>The viewer's speed steps: 0.25, 0.5, 1, 2 and 4 times real time.</summary>
    public static class SpeedLadder
    {
        static readonly double[] Steps = { 0.25, 0.5, 1.0, 2.0, 4.0 };

        public static double Up(double speed)
        {
            foreach (var step in Steps)
                if (step > speed) return step;
            return Steps[Steps.Length - 1];
        }

        public static double Down(double speed)
        {
            for (var i = Steps.Length - 1; i >= 0; --i)
                if (Steps[i] < speed) return Steps[i];
            return Steps[0];
        }
    }
}
