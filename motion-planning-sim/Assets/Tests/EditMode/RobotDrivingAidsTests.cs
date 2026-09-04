using System;
using MotionPlanningSim.Control;
using MotionPlanningSim.Visualization;
using NUnit.Framework;

namespace MotionPlanningSim.Tests
{
    public sealed class RobotDrivingAidsTests
    {
        [Test]
        public void KeyboardMapping_UsesSeparateForwardAndReverseSpeeds()
        {
            var forward = KeyboardTeleopMapping.ComputeCommand(1.0f, 0.0f, 0.35f, 0.25f, 0.6f);
            var reverse = KeyboardTeleopMapping.ComputeCommand(-1.0f, 0.0f, 0.35f, 0.25f, 0.6f);

            Assert.That(forward.LinearMetresPerSecond, Is.EqualTo(0.35f));
            Assert.That(reverse.LinearMetresPerSecond, Is.EqualTo(-0.25f));
        }

        [Test]
        public void KeyboardMapping_ClampsAxesAndMapsLeftToPositiveRosYaw()
        {
            var command = KeyboardTeleopMapping.ComputeCommand(3.0f, 4.0f, 0.35f, 0.25f, 0.6f);

            Assert.That(command.LinearMetresPerSecond, Is.EqualTo(0.35f));
            Assert.That(command.AngularRadiansPerSecond, Is.EqualTo(0.6f));
        }

        [TestCase(0.5f, 2.0f)]
        [TestCase(7.0f, 7.0f)]
        [TestCase(20.0f, 12.0f)]
        public void CameraDistance_IsClampedToConfiguredRange(float requested, float expected)
        {
            Assert.That(
                ThirdPersonCameraMath.ClampDistance(requested, 2.0f, 12.0f),
                Is.EqualTo(expected));
        }

        [Test]
        public void CameraDistance_RejectsInvalidBounds()
        {
            Assert.Throws<ArgumentOutOfRangeException>(
                () => ThirdPersonCameraMath.ClampDistance(4.0f, 5.0f, 2.0f));
        }

        [TestCase(float.PositiveInfinity, 6.0f)]
        [TestCase(8.0f, 6.0f)]
        [TestCase(2.0f, 1.9f)]
        [TestCase(0.1f, 0.2f)]
        public void CameraObstruction_ShortensBoomWithPadding(
            float obstructionDistance,
            float expectedDistance)
        {
            Assert.That(
                ThirdPersonCameraMath.ResolveObstructedDistance(
                    6.0f,
                    obstructionDistance,
                    0.1f,
                    0.2f),
                Is.EqualTo(expectedDistance).Within(1e-5f));
        }

        [Test]
        public void CameraViews_CycleInDeclaredOrderAndWrap()
        {
            Assert.That(
                RobotCameraViewCycler.Next(RobotCameraView.Orbit),
                Is.EqualTo(RobotCameraView.RearLeftChase));
            Assert.That(
                RobotCameraViewCycler.Next(RobotCameraView.RearLeftChase),
                Is.EqualTo(RobotCameraView.PayloadFirstPerson));
            Assert.That(
                RobotCameraViewCycler.Next(RobotCameraView.PayloadFirstPerson),
                Is.EqualTo(RobotCameraView.Orbit));
        }

        [Test]
        public void CameraViews_RejectUnknownValue()
        {
            Assert.Throws<ArgumentOutOfRangeException>(
                () => RobotCameraViewCycler.Next((RobotCameraView)99));
        }
    }
}
