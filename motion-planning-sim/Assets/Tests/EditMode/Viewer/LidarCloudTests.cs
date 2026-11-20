using System.Linq;
using System.Numerics;
using MotionPlanningSim.Viewer.Core;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>4B-1 Task 2: lidar points at their approximate time, placed with the base pose of that time.</summary>
    public sealed class LidarCloudTests
    {
        static (double x, double y, double yaw)? Moving(double t) => (t - 1.0, 0.0, 0.0);  // 1 m/s along x from t 1
        static Vector3 Mount(Vector3 p) => p + new Vector3(0.3f, 0f, 0.5f);

        static float[] Origins(int n) => new float[3 * n];
        static float[] Classes(int n, float c = 0f) => Enumerable.Repeat(c, n).ToArray();

        [Test]
        public void LidarPointTimesAreSpreadOverTheScan()
        {
            for (var i = 0; i < 5; ++i) Assert.AreEqual(1.0 + 0.02 * i, LidarCloud.PointTime(1.0, 1.1, i, 5), 1e-12);
        }

        [Test]
        public void LidarPlacementUsesTheBasePoseAtEachPointsTime()
        {
            var cloud = new LidarCloud();
            Assert.IsTrue(cloud.AddScan(1.0, 1.1, Origins(5), Classes(5), Moving, Mount));
            var points = cloud.Visible().ToList();
            Assert.AreEqual(5, points.Count);
            Assert.AreEqual(0.3, points[0].position.X, 1e-5);
            Assert.AreEqual(0.08, points[4].position.X - points[0].position.X, 1e-5);
            Assert.AreEqual(0.5, points[0].position.Z, 1e-5);
        }

        [Test]
        public void AScanWithoutPoseHistoryIsSkipped()
        {
            var cloud = new LidarCloud();
            Assert.IsFalse(cloud.AddScan(1.0, 1.1, Origins(5), Classes(5), t => null, Mount));
            Assert.AreEqual(1, cloud.Skipped);
            Assert.AreEqual(0, cloud.Visible().Count());
        }

        [Test]
        public void CloudKeepsHalfASecondAndAtMostTwentyThousandPoints()
        {
            var cloud = new LidarCloud();
            cloud.AddScan(1.0, 1.1, Origins(10), Classes(10), t => (0, 0, 0), Mount);
            cloud.AddScan(1.3, 1.4, Origins(10), Classes(10), t => (0, 0, 0), Mount);
            cloud.AddScan(1.6, 1.7, Origins(10), Classes(10), t => (0, 0, 0), Mount);  // 1.1 < 1.7 - 0.5: the first goes
            Assert.AreEqual(20, cloud.Visible().Count());
            var big = new LidarCloud(maxPoints: 20000);
            for (var k = 0; k < 4; ++k) big.AddScan(2.0 + 0.1 * k, 2.1 + 0.1 * k, Origins(8000), Classes(8000), t => (0, 0, 0), Mount);
            Assert.LessOrEqual(big.Count, 20000);
            Assert.Greater(big.Count, 15000, "the oldest scan is thinned, not everything dropped");
        }

        [Test]
        public void GroundHitsAreHiddenByDefault()
        {
            var cloud = new LidarCloud();
            var classes = new[] { 0f, 3f, 3f, 5f };  // site, ground, ground, panel
            cloud.AddScan(1.0, 1.1, Origins(4), classes, t => (0, 0, 0), Mount);
            Assert.AreEqual(2, cloud.Visible().Count());
            cloud.ShowGround = true;
            Assert.AreEqual(4, cloud.Visible().Count());
        }
    }
}
