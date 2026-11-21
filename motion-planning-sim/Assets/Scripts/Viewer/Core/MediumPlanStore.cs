using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// F2's medium plan as one Plans frame carries it (stream-protocol.md, Plans): the decision trace in the header,
    /// the samples in block 0 (9 floats each: x, y, yaw, then the six arm joints). Samples are spaced by 0.1 m of arc
    /// length or 0.1 rad of yaw, without times. Since 4B-2 (capability <c>plan_display_samples</c>), block 1 carries
    /// the same plan every 0.1 s for display, 10 floats each: t from the plan's start, then the 9 values.
    /// </summary>
    public sealed class MediumPlan
    {
        public long PlanId { get; private set; }
        public long RouteId { get; private set; }
        public long Epoch { get; private set; }
        public long GoalId { get; private set; }
        public string Status { get; private set; } = "";
        public string Homotopy { get; private set; } = "";
        public string Seed { get; private set; } = "";
        public bool Capped { get; private set; }
        public bool Waiting { get; private set; }
        public bool FullMultistart { get; private set; }
        public bool HomotopySwitched { get; private set; }
        public IReadOnlyList<(string name, double merit, bool feasible, double progress, bool merged)> Seeds { get; private set; }
        public IReadOnlyList<double[]> Samples { get; private set; }

        /// <summary>The display samples (block 1), 10 values each (t, x, y, yaw, q0..q5), or empty (an older server, or
        /// a plan reused unchanged).</summary>
        public IReadOnlyList<double[]> DisplaySamples { get; private set; } = Array.Empty<double[]>();

        public bool HasDisplaySamples => DisplaySamples.Count > 0;

        /// <summary>The display samples' last time (s from the plan's start), or 0 without them.</summary>
        public double DisplayHorizonS => HasDisplaySamples ? DisplaySamples[DisplaySamples.Count - 1][0] : 0.0;

        /// <summary>
        /// The states the sweep is drawn through, 9 values each: the display samples without their times when present
        /// (an arm turn with the base at rest is drawn as it moves), else F2's samples (4B-1's approximation).
        /// </summary>
        public IReadOnlyList<double[]> SweepSamples { get; private set; } = Array.Empty<double[]>();

        /// <summary>The plan's state <paramref name="tau"/> s after its start, from the display samples (interpolated,
        /// held at the ends), or null without them.</summary>
        public double[] StateAt(double tau)
        {
            var d = DisplaySamples;
            if (d.Count == 0) return null;
            if (tau <= d[0][0]) return PlanState.Lerp(d[0], 1, d[0], 1, 0.0);
            for (var i = 1; i < d.Count; ++i)
                if (tau <= d[i][0])
                {
                    var span = d[i][0] - d[i - 1][0];
                    return PlanState.Lerp(d[i - 1], 1, d[i], 1, span > 0.0 ? (tau - d[i - 1][0]) / span : 1.0);
                }
            return PlanState.Lerp(d[d.Count - 1], 1, d[d.Count - 1], 1, 0.0);
        }

        static List<double[]> Rows(Frame frame, int block, int width)
        {
            var rows = new List<double[]>();
            if (frame.Blocks.Length <= block) return rows;
            var b = frame.Blocks[block];
            for (var i = 0; i + width <= b.Length; i += width)
            {
                var row = new double[width];
                for (var k = 0; k < width; ++k) row[k] = b[i + k];
                rows.Add(row);
            }
            return rows;
        }

        static double Number(JToken v) =>
            v != null && (v.Type == JTokenType.Float || v.Type == JTokenType.Integer) ? (double)v : double.NaN;

        /// <summary>The simulation time the plan was made at (the header's <c>t</c>).</summary>
        public double ReceivedAtSimT { get; private set; }

        public static MediumPlan FromFrame(Frame frame)
        {
            var h = frame.Header;
            var seeds = new List<(string, double, bool, double, bool)>();
            if (h["seeds"] is JArray rows)
                foreach (var r in rows)
                    if (r is JArray a && a.Count >= 5)  // a non-finite number arrives as null (nlohmann): NaN
                        seeds.Add(((string)a[0], Number(a[1]), Number(a[2]) != 0, Number(a[3]), Number(a[4]) != 0));
            var samples = Rows(frame, 0, 9);
            var display = Rows(frame, 1, 10);
            var sweep = new List<double[]>(display.Count);
            foreach (var row in display)
            {
                var q = new double[9];
                Array.Copy(row, 1, q, 0, 9);
                sweep.Add(q);
            }
            return new MediumPlan
            {
                PlanId = (long?)h["plan_id"] ?? 0,
                RouteId = (long?)h["route_id"] ?? 0,
                Epoch = (long?)h["epoch"] ?? 0,
                GoalId = (long?)h["goal_id"] ?? 0,
                Status = (string)h["status"] ?? "",
                Homotopy = (string)h["homotopy"] ?? "",
                Seed = (string)h["seed"] ?? "",
                Capped = (bool?)h["capped"] ?? false,
                Waiting = (bool?)h["waiting"] ?? false,
                FullMultistart = (bool?)h["full_multistart"] ?? false,
                HomotopySwitched = (bool?)h["homotopy_switched"] ?? false,
                Seeds = seeds,
                Samples = samples,
                DisplaySamples = display,
                SweepSamples = display.Count > 0 ? sweep : samples,
                ReceivedAtSimT = (double?)h["t"] ?? 0.0,
            };
        }
    }
}
