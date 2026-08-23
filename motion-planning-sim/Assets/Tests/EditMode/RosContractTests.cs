using MotionPlanningSim.ROS;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests
{
    public class RosContractTests
    {
        [Test]
        public void RosTimeSplitsSecondsAndNanoseconds()
        {
            var time = RosTimeUtility.FromSeconds(12.345678901);

            Assert.That(time.sec, Is.EqualTo(12));
            Assert.That(time.nanosec, Is.EqualTo(345678901u).Within(1u));
        }

        [Test]
        public void PublicationScheduleRecoversFromBackwardTimeJump()
        {
            var next = 5.0;
            var previous = 4.0;

            var due = PublicationSchedule.IsDue(
                0.0,
                50.0,
                ref next,
                ref previous);

            Assert.That(due, Is.True);
            Assert.That(next, Is.EqualTo(0.02).Within(1e-9));
        }

        [Test]
        public void PublicationScheduleAcceptsFixedStepFloatJitter()
        {
            var next = 0.02;
            var previous = 0.0;

            var due = PublicationSchedule.IsDue(
                0.0199999809,
                50.0,
                ref next,
                ref previous);

            Assert.That(due, Is.True);
            Assert.That(next, Is.EqualTo(0.04).Within(1e-9));
        }

        [Test]
        public void BasePoseRemovesFixedFootprintOffset()
        {
            GroundTruthBaseTfPublisher.ComputeFootprintWorldPose(
                new Vector3(10.0f, 0.21f, 5.0f),
                Quaternion.identity,
                new Vector3(0.0f, 0.21f, 0.0f),
                Quaternion.identity,
                out var footprintPosition,
                out var footprintRotation);

            Assert.That(footprintPosition.x, Is.EqualTo(10.0f).Within(1e-5f));
            Assert.That(footprintPosition.y, Is.EqualTo(0.0f).Within(1e-5f));
            Assert.That(footprintPosition.z, Is.EqualTo(5.0f).Within(1e-5f));
            Assert.That(
                Quaternion.Angle(footprintRotation, Quaternion.identity),
                Is.LessThan(1e-4f));
        }
    }
}
