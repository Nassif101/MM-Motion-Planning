using MotionPlanningSim.ROS;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

namespace MotionPlanningSim.Tests
{
    public class GroundTruthOdometryTests
    {
        private static readonly Vector3EqualityComparer Near = new Vector3EqualityComparer(1e-5f);

        [Test]
        public void ForwardMotionIsPositiveBodyXAtAnyHeading()
        {
            // Robot faces Unity -Z (yaw 180, as at the open fixture) and drives forward at 0.3 m/s.
            var rotation = Quaternion.Euler(0, 180, 0);
            GroundTruthBaseTfPublisher.ComputeFootprintTwist(
                new Vector3(0, 0, -0.3f), Vector3.zero, new Vector3(0, 0.21f, 12), new Vector3(0, 0, 12),
                rotation, out var linear, out var angular);
            Assert.That(linear, Is.EqualTo(new Vector3(0.3f, 0, 0)).Using(Near));
            Assert.That(angular, Is.EqualTo(Vector3.zero).Using(Near));
        }

        [Test]
        public void CounterClockwiseRosYawIsPositiveAngularZ()
        {
            // Positive ROS yaw is a negative rotation about Unity +Y (left-handed, Y up).
            GroundTruthBaseTfPublisher.ComputeFootprintTwist(
                Vector3.zero, new Vector3(0, -0.4f, 0), new Vector3(0, 0.21f, 0), Vector3.zero,
                Quaternion.identity, out var linear, out var angular);
            Assert.That(angular, Is.EqualTo(new Vector3(0, 0, 0.4f)).Using(Near));
            // base_footprint lies on the vertical axis through base_link: pure yaw has no translation.
            Assert.That(linear, Is.EqualTo(Vector3.zero).Using(Near));
        }

        [Test]
        public void FootprintVelocityIncludesTheLeverArmFromTheBodyOrigin()
        {
            // Body pitching forward (ROS +pitch = nose down): Unity angular velocity +X.
            // base_footprint is 0.21 m below base_link, so it moves backward at w * 0.21.
            GroundTruthBaseTfPublisher.ComputeFootprintTwist(
                Vector3.zero, new Vector3(1, 0, 0), new Vector3(0, 0.21f, 0), Vector3.zero,
                Quaternion.identity, out var linear, out _);
            Assert.That(linear, Is.EqualTo(new Vector3(-0.21f, 0, 0)).Using(Near));
        }
    }
}
