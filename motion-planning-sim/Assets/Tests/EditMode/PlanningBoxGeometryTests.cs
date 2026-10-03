using MotionPlanningSim.Environment;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests
{
    public class PlanningBoxGeometryTests
    {
        [Test]
        public void UnityBoundsMapToRosAxes()
        {
            var box = PlanningBoxGeometry.UnityBoundsToRos(
                new Bounds(new Vector3(1, 2, 3), new Vector3(0.5f, 1.0f, 2.0f)));
            Assert.That(box.Center, Is.EqualTo(new Vector3(3, -1, 2)));
            Assert.That(box.Size, Is.EqualTo(new Vector3(2.0f, 0.5f, 1.0f)));
            Assert.That(box.Yaw, Is.EqualTo(0.0f));
        }

        [Test]
        public void BoxRotatedAboutUnityUpKeepsItsSizeAndGetsARosYaw()
        {
            // Unity yaw +30 deg turns forward (+z, ROS +x) toward right (+x, ROS -y): ROS yaw -30 deg.
            Assert.That(PlanningBoxGeometry.TryUnityOrientedBoxToRos(
                new Vector3(1, 2, 3), new Vector3(0.5f, 1.0f, 2.0f), Quaternion.Euler(0, 30, 0), out var box), Is.True);
            Assert.That(Vector3.Distance(box.Center, new Vector3(3, -1, 2)), Is.LessThan(1e-5f));
            Assert.That(box.Size, Is.EqualTo(new Vector3(2.0f, 0.5f, 1.0f)));
            Assert.That(box.Yaw, Is.EqualTo(-30.0f * Mathf.Deg2Rad).Within(1e-5f));
        }

        [Test]
        public void TiltedBoxIsNotOriented()
        {
            Assert.That(PlanningBoxGeometry.TryUnityOrientedBoxToRos(
                Vector3.zero, Vector3.one, Quaternion.Euler(10, 30, 0), out _), Is.False);
        }
    }
}
