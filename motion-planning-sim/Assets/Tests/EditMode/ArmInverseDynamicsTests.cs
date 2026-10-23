using MotionPlanningSim.Control;
using NUnit.Framework;
using UnityEngine;
namespace MotionPlanningSim.Tests
{
    public class ArmInverseDynamicsTests
    {
        static readonly Vector3 G = new Vector3(0f, -9.81f, 0f);
        static ArmLinkBody Point(float mass, Vector3 center, int joint) =>
            new ArmLinkBody {mass = mass, center = center, principalRotation = Quaternion.identity, joint = joint};

        static float[] Torques(Vector3[] axes, Vector3[] points, float[] qd, float[] qdd, ArmLinkBody[] bodies,
                               Vector3 gravity, ArmBaseMotion root = default)
        {
            var torques = new float[axes.Length];
            ArmInverseDynamics.JointTorques(axes, points, qd, qdd, bodies, gravity, root, torques);
            return torques;
        }

        [Test] public void AtRestItIsTheGravityStatics()
        {
            var bodies = new[] {Point(2f, new Vector3(0.5f, 0.3f, 0f), 0)};
            float statics = ArmGravityFeedforward.HoldingTorque(Vector3.forward, Vector3.zero,
                new[] {2f}, new[] {new Vector3(0.5f, 0.3f, 0f)}, G);
            var t = Torques(new[] {Vector3.forward}, new[] {Vector3.zero}, new[] {0f}, new[] {0f}, bodies, G);
            Assert.That(t[0], Is.EqualTo(statics).Within(1e-4));
        }

        [Test] public void APendulumNeedsItsInertiaTimesTheAcceleration()
        {
            // 3 kg at 0.5 m plus 0.2 kg m² about its own centre on the joint axis: (I + m L²) α.
            var body = Point(3f, new Vector3(0.5f, 0f, 0f), 0);
            body.principalInertia = new Vector3(0f, 0f, 0.2f);
            var t = Torques(new[] {Vector3.forward}, new[] {Vector3.zero}, new[] {0f}, new[] {2f}, new[] {body}, Vector3.zero);
            Assert.That(t[0], Is.EqualTo((0.2f + 3f * 0.25f) * 2f).Within(1e-4));
        }

        [Test] public void SpinningAtConstantSpeedNeedsNoTorque()
        {
            var t = Torques(new[] {Vector3.forward}, new[] {Vector3.zero}, new[] {3f}, new[] {0f},
                new[] {Point(3f, new Vector3(0.5f, 0f, 0f), 0)}, Vector3.zero);
            Assert.That(t[0], Is.EqualTo(0f).Within(1e-4));
        }

        [Test] public void TwoLinksMatchTheirClosedForm()
        {
            const float m1 = 2f, m2 = 1.5f, l1 = 0.6f, l2 = 0.4f, q1 = 0.3f, q2 = 0.7f;
            float qd1 = 0.8f, qd2 = -0.5f, qdd1 = 1.2f, qdd2 = 0.9f;
            var elbow = new Vector3(l1 * Mathf.Cos(q1), l1 * Mathf.Sin(q1), 0f);
            var tip = elbow + new Vector3(l2 * Mathf.Cos(q1 + q2), l2 * Mathf.Sin(q1 + q2), 0f);
            var t = Torques(new[] {Vector3.forward, Vector3.forward}, new[] {Vector3.zero, elbow},
                new[] {qd1, qd2}, new[] {qdd1, qdd2}, new[] {Point(m1, elbow, 0), Point(m2, tip, 1)}, Vector3.zero);
            float c2 = Mathf.Cos(q2), s2 = Mathf.Sin(q2);
            float tau2 = m2 * l2 * l2 * (qdd1 + qdd2) + m2 * l1 * l2 * c2 * qdd1 + m2 * l1 * l2 * s2 * qd1 * qd1;
            float tau1 = (m1 * l1 * l1 + m2 * (l1 * l1 + l2 * l2 + 2f * l1 * l2 * c2)) * qdd1
                         + m2 * (l2 * l2 + l1 * l2 * c2) * qdd2 - m2 * l1 * l2 * s2 * (2f * qd1 * qd2 + qd2 * qd2);
            Assert.That(t[0], Is.EqualTo(tau1).Within(1e-3));
            Assert.That(t[1], Is.EqualTo(tau2).Within(1e-3));
        }

        [Test] public void ABaseAccelerationActsAsAnOppositeGravity()
        {
            var bodies = new[] {Point(2f, new Vector3(0.5f, 0.3f, 0f), 0)};
            var accelerating = new Vector3(0.5f, 0f, 0f);
            var t = Torques(new[] {Vector3.forward}, new[] {Vector3.zero}, new[] {0f}, new[] {0f}, bodies, Vector3.zero,
                new ArmBaseMotion {linearAcceleration = accelerating});
            float statics = ArmGravityFeedforward.HoldingTorque(Vector3.forward, Vector3.zero,
                new[] {2f}, new[] {new Vector3(0.5f, 0.3f, 0f)}, -accelerating);
            Assert.That(t[0], Is.EqualTo(statics).Within(1e-4));
        }

        [Test] public void TheBaseCommandBecomesTheMountsMotion()
        {
            // Facing +z, turning left (ROS yaw rate +0.5 rad/s, counter-clockwise) at 0.4 m/s, speeding up at 0.3 m/s².
            var m = ArmBaseMotion.FromBaseCommand(Vector3.one, Vector3.forward, Vector3.up, 0.4f, 0.3f, 0.5f, 0.2f);
            Assert.That(m.origin, Is.EqualTo(Vector3.one));
            Assert.That(Vector3.Distance(m.angularVelocity, new Vector3(0f, -0.5f, 0f)), Is.LessThan(1e-6f));
            Assert.That(Vector3.Distance(m.angularAcceleration, new Vector3(0f, -0.2f, 0f)), Is.LessThan(1e-6f));
            // Forward 0.3 m/s² and the centripetal 0.4 x 0.5 = 0.2 m/s² to the left (ROS +y is Unity -x).
            Assert.That(Vector3.Distance(m.linearAcceleration, new Vector3(-0.2f, 0f, 0.3f)), Is.LessThan(1e-6f));
        }
        [Test] public void TheLimitedCommandsChangeIsTheBasesAcceleration()
        {
            var a = SkidSteerKinematics.CommandAcceleration(new PlanarVelocity(0.1f, -0.2f), new PlanarVelocity(0.11f, -0.216f), 0.02f);
            Assert.That(a.LinearMetresPerSecond, Is.EqualTo(0.5f).Within(1e-4));
            Assert.That(a.AngularRadiansPerSecond, Is.EqualTo(-0.8f).Within(1e-4));
        }
        [Test] public void ATurningBaseSwingsTheArmOutwards()
        {
            // Base yawing at a constant 1 rad/s about +y, joint axis +y through the origin: the centrifugal force on a
            // mass is radial, so the joint needs no torque.
            var t = Torques(new[] {Vector3.up}, new[] {Vector3.zero}, new[] {0f}, new[] {0f},
                new[] {Point(2f, new Vector3(0.5f, 0f, 0f), 0)}, Vector3.zero,
                new ArmBaseMotion {angularVelocity = Vector3.up});
            Assert.That(t[0], Is.EqualTo(0f).Within(1e-4));
        }
    }
}
