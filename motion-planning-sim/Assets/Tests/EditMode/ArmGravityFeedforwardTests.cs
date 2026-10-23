using MotionPlanningSim.Control;
using NUnit.Framework;
namespace MotionPlanningSim.Tests
{
    public class ArmGravityFeedforwardTests
    {
        [Test] public void FeedforwardLeavesTheDriveTheRestOfTheTorqueLimit()
        {
            var (feedforward, driveLimit) = ArmGravityFeedforward.Split(-30f, 150f);
            Assert.That(feedforward, Is.EqualTo(-30f));
            Assert.That(driveLimit, Is.EqualTo(120f));
        }
        [Test] public void GravityBeyondTheLimitIsClampedAndTheDriveGetsNothing()
        {
            var (feedforward, driveLimit) = ArmGravityFeedforward.Split(200f, 150f);
            Assert.That(feedforward, Is.EqualTo(150f));
            Assert.That(driveLimit, Is.EqualTo(0f));
        }
        [Test] public void MissingGravityEntriesReadAsNonFinite()
        {
            var forces = new System.Collections.Generic.List<float> {1f, 2f, 3f};
            var starts = new System.Collections.Generic.List<int> {0, 1, 5};
            Assert.That(ArmGravityFeedforward.Gravity(forces, starts, 1), Is.EqualTo(2f));
            Assert.That(ArmGravityFeedforward.Gravity(forces, starts, 2), Is.NaN);
            Assert.That(ArmGravityFeedforward.Gravity(forces, starts, 3), Is.NaN);
        }
        [Test] public void NonFiniteGravityFallsBackToTheFeedbackDrive()
        {
            var (feedforward, driveLimit) = ArmGravityFeedforward.Split(float.NaN, 150f);
            Assert.That(feedforward, Is.EqualTo(0f));
            Assert.That(driveLimit, Is.EqualTo(150f));
        }
    }
}
