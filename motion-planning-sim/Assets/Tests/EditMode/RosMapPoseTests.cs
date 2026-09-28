using System;
using MotionPlanningSim.ROS;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

namespace MotionPlanningSim.Tests
{
    public class RosMapPoseTests
    {
        [Test]
        public void OpenFixtureMatchesQualificationPlacement()
        {
            // arm_test_place_open: Unity (0, 0.21, 12), yaw 180 <-> ROS (12, 0, pi).
            Assert.That(RosMapPose.ToUnityPosition(12, 0, 0.21f),
                Is.EqualTo(new Vector3(0, 0.21f, 12)).Using(Vector3EqualityComparer.Instance));
            Assert.That(Quaternion.Angle(RosMapPose.ToUnityRotation(Math.PI), Quaternion.Euler(0, 180, 0)),
                Is.LessThan(1e-3f));
        }

        [Test]
        public void GateFixtureMatchesQualificationPlacement()
        {
            // arm_test_place_gate: Unity (7.725, 0.21, -5.3) <-> ROS (-5.3, -7.725).
            Assert.That(RosMapPose.ToUnityPosition(-5.3, -7.725, 0.21f),
                Is.EqualTo(new Vector3(7.725f, 0.21f, -5.3f)).Using(Vector3EqualityComparer.Instance));
        }

        [TestCase(0.0, 0f, 1f)]
        [TestCase(Math.PI / 2, -1f, 0f)]
        [TestCase(-Math.PI / 2, 1f, 0f)]
        public void ForwardAxisFollowsRosHeading(double yaw, float unityX, float unityZ)
        {
            // ROS heading (cos yaw, sin yaw) maps to Unity (x = -sin yaw, z = cos yaw).
            var forward = RosMapPose.ToUnityRotation(yaw) * Vector3.forward;
            Assert.That(forward, Is.EqualTo(new Vector3(unityX, 0, unityZ)).Using(Vector3EqualityComparer.Instance));
        }

        [Test]
        public void RejectsNonFinitePoses()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => RosMapPose.ToUnityPosition(double.NaN, 0, 0.21f));
            Assert.Throws<ArgumentOutOfRangeException>(() => RosMapPose.ToUnityRotation(double.PositiveInfinity));
        }
    }
}
