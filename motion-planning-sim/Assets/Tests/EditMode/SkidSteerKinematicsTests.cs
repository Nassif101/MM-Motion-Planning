using MotionPlanningSim.Control;
using MotionPlanningSim.Environment;
using NUnit.Framework;

namespace MotionPlanningSim.Tests
{
    public sealed class SkidSteerKinematicsTests
    {
        private const float Tolerance = 1e-5f;

        [Test]
        public void StraightCommand_GivesEqualWheelTargets()
        {
            var result = SkidSteerKinematics.ComputeSkidSteerWheelTargets(
                new PlanarVelocity(0.7f, 0.0f),
                MobileManipulatorPhysicalContract.WheelRadiusMetres,
                MobileManipulatorPhysicalContract.WheelTrackMetres);

            Assert.That(result.LeftRadiansPerSecond, Is.EqualTo(5.0f).Within(Tolerance));
            Assert.That(result.RightRadiansPerSecond, Is.EqualTo(5.0f).Within(Tolerance));
        }

        [Test]
        public void PurePositiveYaw_GivesOppositeWheelTargets()
        {
            var result = SkidSteerKinematics.ComputeSkidSteerWheelTargets(
                new PlanarVelocity(0.0f, 0.8f),
                MobileManipulatorPhysicalContract.WheelRadiusMetres,
                MobileManipulatorPhysicalContract.WheelTrackMetres);

            Assert.That(result.LeftRadiansPerSecond, Is.EqualTo(-1.8285714f).Within(Tolerance));
            Assert.That(result.RightRadiansPerSecond, Is.EqualTo(1.8285714f).Within(Tolerance));
        }

        [Test]
        public void ChassisClamp_UsesSeparateForwardAndReverseLimits()
        {
            var forward = SkidSteerKinematics.ClampChassisCommand(
                new PlanarVelocity(4.0f, 2.0f),
                0.8f,
                0.5f,
                0.8f);
            var reverse = SkidSteerKinematics.ClampChassisCommand(
                new PlanarVelocity(-4.0f, -2.0f),
                0.8f,
                0.5f,
                0.8f);

            Assert.That(forward.LinearMetresPerSecond, Is.EqualTo(0.8f));
            Assert.That(forward.AngularRadiansPerSecond, Is.EqualTo(0.8f));
            Assert.That(reverse.LinearMetresPerSecond, Is.EqualTo(-0.5f));
            Assert.That(reverse.AngularRadiansPerSecond, Is.EqualTo(-0.8f));
        }

        [Test]
        public void RateLimiter_UsesAccelerationAndDecelerationRates()
        {
            var accelerating = SkidSteerKinematics.RateLimitChassisCommand(
                new PlanarVelocity(0.0f, 0.0f),
                new PlanarVelocity(0.8f, 0.8f),
                0.5f,
                0.8f,
                0.8f,
                1.2f,
                0.02f);
            var braking = SkidSteerKinematics.RateLimitChassisCommand(
                new PlanarVelocity(0.8f, 0.8f),
                new PlanarVelocity(0.0f, 0.0f),
                0.5f,
                0.8f,
                0.8f,
                1.2f,
                0.02f);

            Assert.That(accelerating.LinearMetresPerSecond, Is.EqualTo(0.01f).Within(Tolerance));
            Assert.That(accelerating.AngularRadiansPerSecond, Is.EqualTo(0.016f).Within(Tolerance));
            Assert.That(braking.LinearMetresPerSecond, Is.EqualTo(0.784f).Within(Tolerance));
            Assert.That(braking.AngularRadiansPerSecond, Is.EqualTo(0.776f).Within(Tolerance));
        }

        [Test]
        public void RateLimiter_BrakesToZeroBeforeReversing()
        {
            var result = SkidSteerKinematics.RateLimitAxis(
                0.01f,
                -0.5f,
                0.5f,
                0.8f,
                0.02f);

            Assert.That(result, Is.Zero.Within(Tolerance));
        }

        [Test]
        public void WheelSaturation_PreservesRatioAndCurvature()
        {
            var result = SkidSteerKinematics.SaturateWheelTargets(
                new WheelVelocityTargets(10.0f, -5.0f),
                8.0f);

            Assert.That(result.Saturated, Is.True);
            Assert.That(result.LeftRadiansPerSecond, Is.EqualTo(8.0f).Within(Tolerance));
            Assert.That(result.RightRadiansPerSecond, Is.EqualTo(-4.0f).Within(Tolerance));
            Assert.That(
                result.RightRadiansPerSecond / result.LeftRadiansPerSecond,
                Is.EqualTo(-0.5f).Within(Tolerance));
        }

        [Test]
        public void NominalMaximumCombinedCommand_IsRatioSaturatedAtOperatingWheelLimit()
        {
            var requested = SkidSteerKinematics.ComputeSkidSteerWheelTargets(
                new PlanarVelocity(
                    MobileManipulatorPhysicalContract.MaximumForwardMetresPerSecond,
                    MobileManipulatorPhysicalContract.MaximumAngularRadiansPerSecond),
                MobileManipulatorPhysicalContract.WheelRadiusMetres,
                MobileManipulatorPhysicalContract.EffectiveSkidSteerTrackMetres);
            var result = SkidSteerKinematics.SaturateWheelTargets(
                requested,
                MobileManipulatorPhysicalContract.MaximumOperatingWheelRadiansPerSecond);

            Assert.That(requested.LeftRadiansPerSecond, Is.EqualTo(1.428571f).Within(Tolerance));
            Assert.That(requested.RightRadiansPerSecond, Is.EqualTo(10.0f).Within(Tolerance));
            Assert.That(result.Saturated, Is.True);
            Assert.That(result.LeftRadiansPerSecond, Is.EqualTo(1.142857f).Within(Tolerance));
            Assert.That(result.RightRadiansPerSecond, Is.EqualTo(8.0f).Within(Tolerance));
            Assert.That(
                result.LeftRadiansPerSecond / result.RightRadiansPerSecond,
                Is.EqualTo(requested.LeftRadiansPerSecond / requested.RightRadiansPerSecond)
                    .Within(Tolerance));
        }

        [Test]
        public void ArticulationDriveBoundary_ConvertsRadiansPerSecondToDegreesPerSecond()
        {
            var result = SkidSteerKinematics.ToArticulationDriveAngularVelocity(1.0f);

            Assert.That(result, Is.EqualTo(57.29578f).Within(Tolerance));
        }

        [TestCase(false, 0.0, true)]
        [TestCase(true, 0.49, false)]
        [TestCase(true, 0.50, false)]
        [TestCase(true, 0.5001, true)]
        public void Watchdog_UsesStrictConfiguredTimeout(
            bool hasCommand,
            double ageSeconds,
            bool expectedStale)
        {
            Assert.That(
                SkidSteerKinematics.IsCommandStale(
                    hasCommand,
                    ageSeconds,
                    MobileManipulatorPhysicalContract.CommandWatchdogSeconds),
                Is.EqualTo(expectedStale));
        }
    }
}
