using System.Collections.Generic;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// F2's medium plan as one Plans frame carries it (stream-protocol.md, Plans): the decision trace in the header,
    /// the samples in block 0 (9 floats each: x, y, yaw, then the six arm joints). Samples are spaced by 0.1 m of arc
    /// length or 0.1 rad of yaw; the stream carries no sample times (4B-1 design 6).
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

        /// <summary>The simulation time the plan was made at (the header's <c>t</c>).</summary>
        public double ReceivedAtSimT { get; private set; }

        public static MediumPlan FromFrame(Frame frame)
        {
            var h = frame.Header;
            var seeds = new List<(string, double, bool, double, bool)>();
            if (h["seeds"] is JArray rows)
                foreach (var r in rows)
                    if (r is JArray a && a.Count >= 5)
                        seeds.Add(((string)a[0], (double)a[1], (double)a[2] != 0, (double)a[3], (double)a[4] != 0));
            var samples = new List<double[]>();
            if (frame.Blocks.Length > 0)
            {
                var b = frame.Blocks[0];
                for (var i = 0; i + 9 <= b.Length; i += 9)
                {
                    var row = new double[9];
                    for (var k = 0; k < 9; ++k) row[k] = b[i + k];
                    samples.Add(row);
                }
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
                ReceivedAtSimT = (double?)h["t"] ?? 0.0,
            };
        }
    }
}
