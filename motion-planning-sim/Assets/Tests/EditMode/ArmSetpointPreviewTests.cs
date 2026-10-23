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
        [Test] public void AStillCommandIsAHoldWhateverItsAcceleration()
        {
            // The JTC keeps a finished trajectory's last point, acceleration included: a hold all the same.
            Assert.That(ArmCommandValidation.IsHold(new[]{0.0,0.0}), Is.True);
            Assert.That(ArmCommandValidation.IsHold(new[]{0.0,0.1}), Is.False);
        }
        [Test] public void AHoldIgnoresTheCommandedAcceleration()
        {
            Assert.That(ArmCommandValidation.HoldAcceleration(true, 0.3), Is.EqualTo(0.0));
            Assert.That(ArmCommandValidation.HoldAcceleration(false, 0.3), Is.EqualTo(0.3));
        }
        [Test] public void TheRealizedAccelerationIsTheCommandedVelocitysChange()
        {
            Assert.That(ArmCommandValidation.RealizedAcceleration(0.20, 0.21, 0.02, 4.0), Is.EqualTo(0.5).Within(1e-9));
            Assert.That(ArmCommandValidation.RealizedAcceleration(0.0, 0.5, 0.02, 4.0), Is.EqualTo(4.0));
            Assert.That(ArmCommandValidation.RealizedAcceleration(0.2, 0.3, 0.0, 4.0), Is.EqualTo(0.0));
            Assert.That(ArmCommandValidation.RealizedAcceleration(0.2, 0.3, 0.5, 4.0), Is.EqualTo(0.0));
            Assert.That(ArmCommandValidation.RealizedAcceleration(0.2, 0.3, double.NaN, 4.0), Is.EqualTo(0.0));
        }
        [Test] public void LeavesANonFiniteHorizonAlone()
        {
            Assert.That(ArmSetpointPreview.Ahead(1.0, 0.5, 2.0, double.NaN, 0.1), Is.EqualTo((1.0, 0.5)));
        }
    }
}
