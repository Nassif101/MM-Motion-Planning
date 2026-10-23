using System;
using UnityEngine;

namespace MotionPlanningSim.Control
{
    /// <summary>A rigid body the arm moves, at the current configuration (world): mass, centre of mass, principal
    /// inertia and its rotation, and the deepest arm joint that moves it (its link).</summary>
    public struct ArmLinkBody
    {
        public float mass;
        public Vector3 center, principalInertia;
        public Quaternion principalRotation;
        public int joint;
    }

    /// <summary>The arm mount's reference motion (world): a point of the base, its acceleration, and the base's
    /// angular velocity and acceleration.</summary>
    public struct ArmBaseMotion
    {
        public Vector3 origin, linearAcceleration, angularVelocity, angularAcceleration;
    }

    /// <summary>Joint torques that make the arm follow given joint rates (Phase 4B-B motion feedforward, ADR 0014):
    /// recursive Newton-Euler at the current configuration, τ = M(q) q̈ + C(q, q̇) q̇ + G(q), with the base's motion as
    /// the root's. Joints are revolute and ordered from the base; axes point along each positive reduced coordinate.
    /// At rest it is the gravity statics (ArmGravityFeedforward.HoldingTorque).</summary>
    public static class ArmInverseDynamics
    {
        public static void JointTorques(Vector3[] axes, Vector3[] points, float[] qd, float[] qdd, ArmLinkBody[] bodies,
                                        Vector3 gravity, ArmBaseMotion root, float[] torques)
        {
            int n = axes.Length;
            Span<Vector3> omega = stackalloc Vector3[n], alpha = stackalloc Vector3[n], pointAcceleration = stackalloc Vector3[n];
            Vector3 w = root.angularVelocity, al = root.angularAcceleration, a = root.linearAcceleration, previous = root.origin;
            for (int l = 0; l < n; ++l)
            {
                // The joint's point rides on the previous link; the joint adds its rate about an axis that link turns.
                var r = points[l] - previous;
                a += Vector3.Cross(al, r) + Vector3.Cross(w, Vector3.Cross(w, r));
                al += axes[l] * qdd[l] + Vector3.Cross(w, axes[l] * qd[l]);
                w += axes[l] * qd[l];
                omega[l] = w; alpha[l] = al; pointAcceleration[l] = a; previous = points[l];
            }
            Array.Clear(torques, 0, n);
            foreach (var body in bodies)
            {
                int l = body.joint;
                var r = body.center - points[l];
                var centerAcceleration = pointAcceleration[l] + Vector3.Cross(alpha[l], r) +
                                         Vector3.Cross(omega[l], Vector3.Cross(omega[l], r));
                var force = body.mass * (centerAcceleration - gravity);
                var moment = Inertia(body, alpha[l]) + Vector3.Cross(omega[l], Inertia(body, omega[l]));
                for (int i = 0; i <= l; ++i)
                    torques[i] += Vector3.Dot(axes[i], Vector3.Cross(body.center - points[i], force) + moment);
            }
        }

        private static Vector3 Inertia(in ArmLinkBody body, Vector3 v)
        {
            var local = Quaternion.Inverse(body.principalRotation) * v;
            return body.principalRotation * Vector3.Scale(body.principalInertia, local);
        }
    }
}
