using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// B5's solved plan as one Prediction frame carries it (spec 4B-2, stream-protocol.md, Prediction): the header's
    /// <c>t</c>, B5's cycle <c>status</c>, <c>stage_dt</c>, <c>stages</c> and the followed F2 <c>plan_id</c> (0 for
    /// none); block 0, (stages + 1) states of 9 floats (x, y, yaw, q0..q5), stage k at <c>t + k stage_dt</c>.
    /// </summary>
    public sealed class B5Prediction
    {
        public double T { get; private set; }
        public string Status { get; private set; } = "";
        public double StageDt { get; private set; }
        public int Stages { get; private set; }
        public long PlanId { get; private set; }
        public IReadOnlyList<double[]> States { get; private set; }

        /// <summary>The plan is not executed this cycle (B5 holds or brakes): it is drawn as held, not as the motion.</summary>
        public bool Held { get; private set; }

        /// <summary>The simulation time of the last stage.</summary>
        public double HorizonEndT => T + Stages * StageDt;

        /// <summary>
        /// B5 executes its solved plan only on <c>ok</c> and <c>fallback</c> (the replay's rule); at
        /// <c>goal_reached</c> there is nothing to hold. Any other status, including one this viewer does not know,
        /// is a hold.
        /// </summary>
        public static bool IsHeld(string status) => status != "ok" && status != "fallback" && status != "goal_reached";

        public static B5Prediction FromFrame(Frame frame)
        {
            var h = frame.Header;
            var states = new List<double[]>();
            if (frame.Blocks.Length > 0)
            {
                var b = frame.Blocks[0];
                for (var i = 0; i + 9 <= b.Length; i += 9)
                {
                    var row = new double[9];
                    for (var k = 0; k < 9; ++k) row[k] = b[i + k];
                    states.Add(row);
                }
            }
            var status = (string)h["status"] ?? "";
            var dt = (double?)h["stage_dt"] ?? 0.1;
            return new B5Prediction
            {
                T = (double?)h["t"] ?? 0.0,
                Status = status,
                Held = IsHeld(status),
                StageDt = dt > 0.0 ? dt : 0.1,
                Stages = Math.Max(0, states.Count - 1),  // the states as received, whatever the header says
                PlanId = (long?)h["plan_id"] ?? 0,
                States = states,
            };
        }

        /// <summary>The state <paramref name="tau"/> seconds after <see cref="T"/>, interpolated between stages and
        /// held at the ends; null without states.</summary>
        public double[] StateAt(double tau)
        {
            if (States.Count == 0) return null;
            var s = Math.Max(0.0, tau / StageDt);
            var k = (int)Math.Floor(s);
            if (k >= States.Count - 1) return (double[])States[States.Count - 1].Clone();
            return PlanState.Lerp(States[k], 0, States[k + 1], 0, s - k);
        }
    }

    /// <summary>Plan states of 9 values (x, y, yaw, q0..q5): the linear blend, with yaw the short way.</summary>
    static class PlanState
    {
        public static double[] Lerp(double[] a, int aAt, double[] b, int bAt, double f)
        {
            var r = new double[9];
            for (var k = 0; k < 9; ++k) r[k] = a[aAt + k] + f * (b[bAt + k] - a[aAt + k]);
            r[2] = a[aAt + 2] + f * Math.IEEERemainder(b[bAt + 2] - a[aAt + 2], 2 * Math.PI);
            return r;
        }
    }
}
