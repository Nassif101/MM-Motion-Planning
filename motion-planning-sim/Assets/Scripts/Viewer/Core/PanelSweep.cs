using System;
using System.Collections.Generic;
using System.Numerics;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// The panel's planned sweep (4B-1 overlay 5, approximate): rings of the panel's four corners along F2's plan.
    /// Between two samples the base (x, y, yaw the short way) and the joints are interpolated linearly, in substeps
    /// until no corner moves more than <c>maxCornerStepM</c> from one ring to the next. Not a collision envelope: F2's
    /// curve between samples is not linear, and B5 tracks the plan with an error (design 7.1).
    /// </summary>
    public static class PanelSweep
    {
        const int MaxRefinements = 6;

        /// <param name="samples">Plan samples, 9 values each: x, y, yaw, q0..q5 (map frame, ROS).</param>
        /// <param name="panelCorners">The panel's four corners in the map frame at a sample's configuration.</param>
        public static IReadOnlyList<Vector3[]> Rings(IReadOnlyList<double[]> samples, Func<double[], Vector3[]> panelCorners,
            double maxCornerStepM = 0.05, int maxRings = 2000)
        {
            var rings = new List<Vector3[]>();
            if (samples == null || samples.Count == 0) return rings;
            rings.Add(panelCorners(samples[0]));
            for (var i = 1; i < samples.Count && rings.Count < maxRings; ++i)
            {
                var a = samples[i - 1];
                var b = samples[i];
                var start = rings[rings.Count - 1];
                var end = panelCorners(b);
                var n = Math.Max(1, (int)Math.Ceiling(MaxDistance(start, end) / maxCornerStepM));
                List<Vector3[]> segment = null;
                for (var refine = 0; refine <= MaxRefinements; ++refine)
                {
                    segment = new List<Vector3[]>(n);
                    var prev = start;
                    var worst = 0.0;
                    for (var k = 1; k <= n; ++k)
                    {
                        var ring = k == n ? end : panelCorners(Lerp(a, b, (double)k / n));
                        worst = Math.Max(worst, MaxDistance(prev, ring));
                        segment.Add(ring);
                        prev = ring;
                    }
                    // a turn moves corners along arcs, longer than the end-to-end chord: refine until the steps hold
                    if (worst <= maxCornerStepM + 1e-9 || rings.Count + 2 * n > maxRings) break;
                    n *= 2;
                }
                foreach (var ring in segment)
                {
                    if (rings.Count >= maxRings) break;
                    rings.Add(ring);
                }
            }
            return rings;
        }

        static double MaxDistance(Vector3[] p, Vector3[] q)
        {
            var max = 0.0;
            for (var k = 0; k < p.Length; ++k) max = Math.Max(max, Vector3.Distance(p[k], q[k]));
            return max;
        }

        static double[] Lerp(double[] a, double[] b, double f)
        {
            var r = new double[a.Length];
            for (var k = 0; k < a.Length; ++k) r[k] = a[k] + f * (b[k] - a[k]);
            r[2] = a[2] + f * Math.IEEERemainder(b[2] - a[2], 2 * Math.PI);  // yaw the short way
            return r;
        }
    }
}
