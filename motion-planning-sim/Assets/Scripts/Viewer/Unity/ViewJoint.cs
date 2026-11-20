using System;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// A display-only joint of the viewer's robot (design 6.1). The setup command records the joint's name, its
    /// local axis (the articulation anchor's X axis) and its rest rotation before removing the ArticulationBody; the
    /// link then turns only through <see cref="SetAngle"/>. A positive angle is the URDF's positive joint angle.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ViewJoint : MonoBehaviour
    {
        [SerializeField] private string jointName = "";
        [SerializeField] private Vector3 axis = Vector3.right;
        [SerializeField] private Quaternion rest = Quaternion.identity;

        public string JointName => jointName;
        public Vector3 Axis => axis;

        /// <summary>The last angle set, in radians, wrapped to (-pi, pi].</summary>
        public float Angle { get; private set; }

        public void Configure(string name, Vector3 localAxis, Quaternion restRotation)
        {
            if (string.IsNullOrEmpty(name)) throw new ArgumentException("a joint needs a name", nameof(name));
            if (localAxis.sqrMagnitude < 1e-12f) throw new ArgumentException("a joint needs an axis", nameof(localAxis));
            jointName = name;
            axis = localAxis.normalized;
            rest = restRotation;
        }

        public void SetAngle(float radians)
        {
            if (!float.IsFinite(radians)) return;
            Angle = (float)Math.IEEERemainder(radians, 2 * Math.PI);
            transform.localRotation = rest * Quaternion.AngleAxis(Angle * Mathf.Rad2Deg, axis);
        }
    }
}
