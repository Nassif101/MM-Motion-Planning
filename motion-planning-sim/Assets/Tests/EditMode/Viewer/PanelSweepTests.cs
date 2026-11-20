using System;
using System.Numerics;
using MotionPlanningSim.Viewer.Core;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>4B-1 Task 2: the panel's planned sweep (approximate) from F2's plan samples.</summary>
    public sealed class PanelSweepTests
    {
        // A 1.2 m square panel centred on the base, turned by the first arm joint (sample index 3)
        static Vector3[] Corners(double[] s)
        {
            var c = new Vector3[4];
            var a = s[2] + s[3];
            for (var k = 0; k < 4; ++k)
            {
                double lx = (k == 0 || k == 3) ? 0.6 : -0.6, ly = k < 2 ? 0.6 : -0.6;
                c[k] = new Vector3((float)(s[0] + lx * Math.Cos(a) - ly * Math.Sin(a)), (float)(s[1] + lx * Math.Sin(a) + ly * Math.Cos(a)), 1f);
            }
            return c;
        }

        static double[] Sample(double x, double q0) => new[] { x, 0, 0, q0, 0, 0, 0, 0, 0 };

        static double MaxCornerStep(System.Collections.Generic.IReadOnlyList<Vector3[]> rings)
        {
            var max = 0.0;
            for (var i = 1; i < rings.Count; ++i)
                for (var k = 0; k < 4; ++k) max = Math.Max(max, Vector3.Distance(rings[i][k], rings[i - 1][k]));
            return max;
        }

        [Test]
        public void SweepInterpolatesUntilCornersMoveAtMostFiveCentimetres()
        {
            var rings = PanelSweep.Rings(new[] { Sample(0, 0), Sample(0.1, 0.5) }, Corners);
            Assert.Greater(rings.Count, 2);
            Assert.LessOrEqual(MaxCornerStep(rings), 0.05 + 1e-6);
            rings = PanelSweep.Rings(new[] { Sample(0, 0), Sample(0, Math.PI) }, Corners);  // a half turn: arcs, not chords
            Assert.LessOrEqual(MaxCornerStep(rings), 0.05 + 1e-6);
            Assert.AreEqual(Corners(Sample(0, Math.PI))[0].X, rings[rings.Count - 1][0].X, 1e-5);  // ends on the last sample
        }

        // The known gap (design 7.1): a reconfiguration with the base still has no samples between its ends, so the
        // band joins them by interpolating the joints
        [Test]
        public void SweepJoinsAStillBaseReconfigurationDirectly()
        {
            var rings = PanelSweep.Rings(new[] { Sample(1, 0), Sample(1, 1.0) }, Corners);
            var mid = rings[rings.Count / 2];
            var expected = Corners(Sample(1, 1.0 * (rings.Count / 2) / (rings.Count - 1)));
            Assert.AreEqual(expected[0].X, mid[0].X, 0.02);
            Assert.AreEqual(expected[0].Y, mid[0].Y, 0.02);
        }

        [Test]
        public void SweepIsCappedAtMaxRings()
        {
            var samples = new double[100][];
            for (var i = 0; i < 100; ++i) samples[i] = Sample(0, 0.5 * i);  // 50 rad of turning
            var rings = PanelSweep.Rings(samples, Corners, maxRings: 300);
            Assert.LessOrEqual(rings.Count, 300);
            Assert.AreEqual(0, PanelSweep.Rings(Array.Empty<double[]>(), Corners).Count);
            Assert.AreEqual(1, PanelSweep.Rings(new[] { Sample(0, 0) }, Corners).Count);
        }
    }
}
