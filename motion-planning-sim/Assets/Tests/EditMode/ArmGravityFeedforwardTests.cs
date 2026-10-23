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
        [Test] public void NonFiniteGravityFallsBackToTheFeedbackDrive()
        {
            var (feedforward, driveLimit) = ArmGravityFeedforward.Split(float.NaN, 150f);
            Assert.That(feedforward, Is.EqualTo(0f));
            Assert.That(driveLimit, Is.EqualTo(150f));
        }
    }
}
