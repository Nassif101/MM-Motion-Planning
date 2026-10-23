using MotionPlanningSim.Control;
using NUnit.Framework;
namespace MotionPlanningSim.Tests
{
    public class ArmGravityFeedforwardTests
    {
        [Test] public void TheDriveTargetShiftsByTorqueOverStiffness()
        {
            Assert.That(ArmGravityFeedforward.TargetOffset(-30f, 150f, 3000f), Is.EqualTo(-0.01f).Within(1e-7));
        }
        [Test] public void TorqueBeyondTheLimitIsClamped()
        {
            Assert.That(ArmGravityFeedforward.TargetOffset(200f, 150f, 3000f), Is.EqualTo(0.05f).Within(1e-7));
        }
        static readonly UnityEngine.Vector3 G = new UnityEngine.Vector3(0f, -9.81f, 0f);
        [Test] public void HoldingTorqueOfALeverIsWeightTimesArm()
        {
            // 1 kg one metre out along x, joint axis along z through the origin.
            float torque = ArmGravityFeedforward.HoldingTorque(UnityEngine.Vector3.forward, UnityEngine.Vector3.zero,
                new[] {1f}, new[] {UnityEngine.Vector3.right}, G);
            Assert.That(torque, Is.EqualTo(9.81f).Within(1e-4));
        }
        [Test] public void HoldingTorqueFlipsWithTheAxisAndSumsTheCarriedBodies()
        {
            float torque = ArmGravityFeedforward.HoldingTorque(UnityEngine.Vector3.back, new UnityEngine.Vector3(0f, 0f, 3f),
                new[] {1f, 2f}, new[] {UnityEngine.Vector3.right, new UnityEngine.Vector3(-0.25f, 1f, 0f)}, G);
            Assert.That(torque, Is.EqualTo(-(9.81f - 2f * 9.81f * 0.25f)).Within(1e-4));
        }
        [Test] public void BodiesOnTheAxisNeedNoTorque()
        {
            float torque = ArmGravityFeedforward.HoldingTorque(UnityEngine.Vector3.up, UnityEngine.Vector3.zero,
                new[] {5f}, new[] {new UnityEngine.Vector3(1f, 2f, 0f)}, G);
            Assert.That(torque, Is.EqualTo(0f).Within(1e-5));
        }
        [Test] public void NonFiniteTorqueLeavesTheTargetAlone()
        {
            Assert.That(ArmGravityFeedforward.TargetOffset(float.NaN, 150f, 3000f), Is.EqualTo(0f));
        }
    }
}
