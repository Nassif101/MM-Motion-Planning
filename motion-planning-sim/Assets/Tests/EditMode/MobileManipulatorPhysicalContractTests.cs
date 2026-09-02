using MotionPlanningSim.Environment;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests
{
    public class MobileManipulatorPhysicalContractTests
    {
        [Test]
        public void ReferencePanelHasExpectedMassCentreAndInertia()
        {
            Assert.That(MobileManipulatorPhysicalContract.PayloadMassKg, Is.EqualTo(3.0f));
            Assert.That(
                MobileManipulatorPhysicalContract.PayloadCentreOfMassAtTool,
                Is.EqualTo(new Vector3(0.0f, 0.035f, 0.0f)));

            var inertia = MobileManipulatorPhysicalContract.ReferencePayloadInertia;
            Assert.That(inertia.x, Is.EqualTo(0.3604f).Within(1e-5f));
            Assert.That(inertia.y, Is.EqualTo(0.72f).Within(1e-5f));
            Assert.That(inertia.z, Is.EqualTo(0.3604f).Within(1e-5f));
        }

        [Test]
        public void BareFootprintContainsWheelExtentsAndAllowance()
        {
            var operationalLength =
                MobileManipulatorPhysicalContract.BareFootprintLengthMetres +
                (2.0f * MobileManipulatorPhysicalContract.BareFootprintAllowanceMetres);
            var operationalWidth =
                MobileManipulatorPhysicalContract.BareFootprintWidthMetres +
                (2.0f * MobileManipulatorPhysicalContract.BareFootprintAllowanceMetres);

            Assert.That(operationalLength, Is.EqualTo(0.92f).Within(1e-6f));
            Assert.That(operationalWidth, Is.EqualTo(0.77f).Within(1e-6f));
        }

        [Test]
        public void MaximumBaseCommandStaysBelowHardWheelSpeed()
        {
            const float maximumForwardVelocity = 0.8f;
            const float maximumYawVelocity = 0.8f;
            var fasterWheelVelocity =
                (maximumForwardVelocity +
                 maximumYawVelocity * MobileManipulatorPhysicalContract.WheelTrackMetres * 0.5f) /
                MobileManipulatorPhysicalContract.WheelRadiusMetres;

            Assert.That(fasterWheelVelocity, Is.EqualTo(7.542857f).Within(1e-5f));
            Assert.That(fasterWheelVelocity, Is.LessThan(18.0f));
        }
    }
}
