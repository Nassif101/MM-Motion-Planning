using System;
using System.Collections.Generic;
using System.Numerics;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// Lidar points for the 4B-1 overlay, placed approximately: point i of n in a scan takes the time
    /// <c>t_start + (i / n)(t_end - t_start)</c> (the plant emits points in ray-time order but does not stream each
    /// point's time), the base pose interpolated at that time, and the lidar mount. A scan any of whose times has no
    /// pose is skipped and counted. The cloud keeps <c>keepS</c> of simulation time and at most <c>maxPoints</c>,
    /// thinning the oldest scan first. Hit classes: 0 site, 1 scenario, 2 mover, 3 ground, 4 robot, 5 panel, 6 other.
    /// </summary>
    public sealed class LidarCloud
    {
        public const int Ground = 3;

        sealed class Scan
        {
            public double TEnd;
            public List<(Vector3 position, int hitClass)> Points;
        }

        readonly double keepS;
        readonly int maxPoints;
        readonly List<Scan> scans = new List<Scan>();

        public LidarCloud(double keepS = 0.5, int maxPoints = 20000)
        {
            this.keepS = keepS;
            this.maxPoints = maxPoints;
        }

        public bool ShowGround { get; set; }
        public int Skipped { get; private set; }
        public int Count { get; private set; }

        /// <summary>Incremented on every change, so a presenter rebuilds its mesh only when the cloud changed.</summary>
        public int Version { get; private set; }

        public static double PointTime(double tStart, double tEnd, int i, int n) =>
            n <= 0 ? tStart : tStart + (double)i / n * (tEnd - tStart);

        /// <param name="xyz">3 floats per point in the lidar frame.</param>
        /// <param name="baseAt">The base pose (map frame, ROS) at a simulation time, or null outside the history.</param>
        /// <param name="livoxToBase">A point in the lidar frame to the base frame (ROS).</param>
        public bool AddScan(double tStart, double tEnd, float[] xyz, float[] hitClass,
            Func<double, (double x, double y, double yaw)?> baseAt, Func<Vector3, Vector3> livoxToBase)
        {
            var n = Math.Min(xyz.Length / 3, hitClass.Length);
            var points = new List<(Vector3, int)>(n);
            for (var i = 0; i < n; ++i)
            {
                var pose = baseAt(PointTime(tStart, tEnd, i, n));
                if (pose == null)
                {
                    ++Skipped;
                    return false;
                }
                var (x, y, yaw) = pose.Value;
                var p = livoxToBase(new Vector3(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]));
                double c = Math.Cos(yaw), s = Math.Sin(yaw);
                points.Add((new Vector3((float)(x + c * p.X - s * p.Y), (float)(y + s * p.X + c * p.Y), p.Z), (int)hitClass[i]));
            }
            scans.Add(new Scan { TEnd = tEnd, Points = points });
            Count += points.Count;
            scans.RemoveAll(sc =>
            {
                var old = sc.TEnd < tEnd - keepS;
                if (old) Count -= sc.Points.Count;
                return old;
            });
            for (var k = 0; Count > maxPoints && k < scans.Count; )
            {
                var oldest = scans[k];
                var kept = new List<(Vector3, int)>(oldest.Points.Count / 2 + 1);
                for (var i = 0; i < oldest.Points.Count; i += 2) kept.Add(oldest.Points[i]);  // a stride of 2
                Count -= oldest.Points.Count - kept.Count;
                oldest.Points = kept;
                if (kept.Count <= 1) ++k;  // thinned out: move to the next scan
            }
            ++Version;
            return true;
        }

        public IEnumerable<(Vector3 position, int hitClass)> Visible()
        {
            foreach (var scan in scans)
                foreach (var p in scan.Points)
                    if (ShowGround || p.hitClass != Ground) yield return p;
        }

        public void Clear()
        {
            scans.Clear();
            Count = 0;
            ++Version;
        }
    }
}
