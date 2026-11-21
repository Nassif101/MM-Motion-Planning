using System;
using System.Collections.Generic;
using System.Numerics;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// Lidar points for the 4B-1 overlay, placed approximately: point i of n in a scan takes the time
    /// <c>t_start + (i / n)(t_end - t_start)</c>, the base pose interpolated at that time, and the lidar mount. The
    /// plant keeps points in ray-time order but drops rays without a hit, so the time can be off by part of the scan
    /// period (the legend says "approximate"). Since 4B-2 (capability <c>scan_point_times</c>), a scan may carry each
    /// point's own time offset from <c>t_start</c>; then point i takes <c>t_start + dt[i]</c> (exact).
    /// A scan any of whose kept points has no pose is skipped and counted.
    /// <para>Budget (Task 2 review I1, I2): each scan is thinned when it is added to its share of <c>maxPoints</c>
    /// (<c>maxPoints</c> times its period over <c>keepS</c>; 4,000 at 10 Hz), only the kept points are placed, hidden
    /// ground hits are left out before placement, whole scans are dropped by age (and the oldest first if still over
    /// the budget), and scan buffers are reused.</para>
    /// Hit classes: 0 site, 1 scenario, 2 mover, 3 ground, 4 robot, 5 panel, 6 other.
    /// </summary>
    public sealed class LidarCloud
    {
        public const int Ground = 3;

        sealed class Scan
        {
            public double TEnd;
            public readonly List<(Vector3 position, int hitClass)> Points = new List<(Vector3, int)>();
        }

        readonly double keepS;
        readonly int maxPoints;
        readonly List<Scan> scans = new List<Scan>();
        readonly Stack<Scan> free = new Stack<Scan>();
        bool showGround;

        public LidarCloud(double keepS = 0.5, int maxPoints = 20000)
        {
            this.keepS = keepS;
            this.maxPoints = maxPoints;
        }

        /// <summary>Ground hits are drawn; applies from the next scan.</summary>
        public bool ShowGround
        {
            get => showGround;
            set
            {
                if (value == showGround) return;
                showGround = value;
                ++Version;
            }
        }

        public int Skipped { get; private set; }

        /// <summary>The last scan added was placed with its points' own times (4B-2), not the approximation.</summary>
        public bool Exact { get; private set; }
        public int Count { get; private set; }
        public int Scans => scans.Count;

        /// <summary>Incremented on every change, so a presenter rebuilds its mesh only when the cloud changed.</summary>
        public int Version { get; private set; }

        public static double PointTime(double tStart, double tEnd, int i, int n) =>
            n <= 0 ? tStart : tStart + (double)i / n * (tEnd - tStart);

        /// <param name="xyz">3 floats per point in the lidar frame.</param>
        /// <param name="baseAt">The base pose (map frame, ROS) at a simulation time, or null outside the history.</param>
        /// <param name="livoxToBase">A point in the lidar frame to the base frame (ROS).</param>
        /// <param name="pointTimes">Each point's time offset from <paramref name="tStart"/> (Scan block 2), or null:
        /// used only when it has one per point.</param>
        public bool AddScan(double tStart, double tEnd, float[] xyz, float[] hitClass,
            Func<double, (double x, double y, double yaw)?> baseAt, Func<Vector3, Vector3> livoxToBase,
            float[] pointTimes = null)
        {
            var n = Math.Min(xyz.Length / 3, hitClass.Length);
            var exact = pointTimes != null && pointTimes.Length == n;
            var visible = 0;
            for (var i = 0; i < n; ++i)
                if (showGround || (int)hitClass[i] != Ground) ++visible;
            var period = Math.Max(1e-3, tEnd - tStart);
            var budget = Math.Max(1, (int)(maxPoints * Math.Min(1.0, period / keepS)));
            var stride = Math.Max(1, (visible + budget - 1) / budget);
            var scan = free.Count > 0 ? free.Pop() : new Scan();
            scan.Points.Clear();
            scan.TEnd = tEnd;
            for (int i = 0, v = 0; i < n; ++i)
            {
                var hit = (int)hitClass[i];
                if (!showGround && hit == Ground) continue;
                if (v++ % stride != 0) continue;
                // within [t_start, t_end]: the float offsets of the last rays land just past t_end (Task 3 review I1)
                var pose = baseAt(exact ? Math.Min(tEnd, Math.Max(tStart, tStart + pointTimes[i])) : PointTime(tStart, tEnd, i, n));
                if (pose == null)
                {
                    free.Push(scan);
                    ++Skipped;
                    return false;
                }
                var (x, y, yaw) = pose.Value;
                var p = livoxToBase(new Vector3(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]));
                double c = Math.Cos(yaw), s = Math.Sin(yaw);
                scan.Points.Add((new Vector3((float)(x + c * p.X - s * p.Y), (float)(y + s * p.X + c * p.Y), p.Z), hit));
            }
            scans.Add(scan);
            Exact = exact;
            Count += scan.Points.Count;
            for (var k = scans.Count - 1; k >= 0; --k)
                if (scans[k].TEnd < tEnd - keepS) Drop(k);
            while (Count > maxPoints && scans.Count > 1) Drop(0);
            ++Version;
            return true;
        }

        void Drop(int k)
        {
            Count -= scans[k].Points.Count;
            free.Push(scans[k]);
            scans.RemoveAt(k);
        }

        public IEnumerable<(Vector3 position, int hitClass)> Visible()
        {
            foreach (var scan in scans)
                foreach (var p in scan.Points)
                    if (showGround || p.hitClass != Ground) yield return p;
        }

        public void Clear()
        {
            while (scans.Count > 0) Drop(scans.Count - 1);
            ++Version;
        }
    }
}
