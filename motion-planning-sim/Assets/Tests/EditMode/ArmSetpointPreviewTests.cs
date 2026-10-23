using MotionPlanningSim.Control;
using NUnit.Framework;
namespace MotionPlanningSim.Tests
{
    public class ArmSetpointPreviewTests
    {
        [Test] public void ExtrapolatesTheCommandToTheEndOfTheStep()
        {
            var (q, v) = ArmSetpointPreview.Ahead(1.0, 0.5, 2.0, 0.02, 0.1);
            Assert.That(q, Is.EqualTo(1.0 + 0.5 * 0.02 + 0.5 * 2.0 * 0.0004).Within(1e-12));
            Assert.That(v, Is.EqualTo(0.5 + 2.0 * 0.02).Within(1e-12));
        }
        [Test] public void NeverLooksBackOrBeyondTheCap()
        {
            Assert.That(ArmSetpointPreview.Ahead(1.0, 0.5, 2.0, -0.01, 0.1), Is.EqualTo((1.0, 0.5)));
            var (q, _) = ArmSetpointPreview.Ahead(1.0, 0.5, 0.0, 0.5, 0.1);
            Assert.That(q, Is.EqualTo(1.05).Within(1e-12));
        }
        [Test] public void StopsWhereTheVelocityWouldReverse()
        {
            // v 0.01 rad/s decelerating at 1 rad/s²: at rest after 0.01 s, never driven backwards.
            var (q, v) = ArmSetpointPreview.Ahead(1.0, 0.01, -1.0, 0.04, 0.1);
            Assert.That(v, Is.EqualTo(0.0).Within(1e-12));
            Assert.That(q, Is.EqualTo(1.0 + 0.01 * 0.01 - 0.5 * 0.0001).Within(1e-12));
        }
        [Test] public void AHoldNeedsZeroVelocityAndAcceleration()
        {
            Assert.That(ArmCommandValidation.IsHold(new[]{0.0,0.0}, new[]{0.0,0.0}), Is.True);
            Assert.That(ArmCommandValidation.IsHold(new[]{0.0,0.0}, new[]{0.0,0.3}), Is.False);
            Assert.That(ArmCommandValidation.IsHold(new[]{0.0,0.1}, new[]{0.0,0.0}), Is.False);
        }
        [Test] public void LeavesANonFiniteHorizonAlone()
        {
            Assert.That(ArmSetpointPreview.Ahead(1.0, 0.5, 2.0, double.NaN, 0.1), Is.EqualTo((1.0, 0.5)));
        }
    }
}
