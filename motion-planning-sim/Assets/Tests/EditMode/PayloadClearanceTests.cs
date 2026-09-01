using MotionPlanningSim.Environment;
using NUnit.Framework;

namespace MotionPlanningSim.Tests
{
    public class PayloadClearanceTests
    {
        [Test]
        public void InitialPanelRequiresOnePointEightMetreNominalPassage()
        {
            Assert.That(
                PayloadClearance.RequiredStraightPassage(1.2f, 0.3f),
                Is.EqualTo(1.8f).Within(1e-5f));
        }

        [Test]
        public void InitialSquarePanelSweepsOnePointSevenMetresWhenTurning()
        {
            Assert.That(
                PayloadClearance.RectangularSweptDiameter(1.2f, 1.2f),
                Is.EqualTo(1.697056f).Within(1e-5f));
        }

        [Test]
        public void MainLaneFitsButManipulationGateRejectsInitialPose()
        {
            Assert.That(PayloadClearance.FitsStraightPassage(1.2f, 2.4f, 0.3f), Is.True);
            Assert.That(PayloadClearance.FitsStraightPassage(1.2f, 1.05f, 0.0f), Is.False);
        }
    }
}
