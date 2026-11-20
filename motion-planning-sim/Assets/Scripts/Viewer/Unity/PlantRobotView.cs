using System;
using System.Linq;
using MotionPlanningSim.ROS;
using MotionPlanningSim.Viewer.Core;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// Poses the physics-free robot from the backend's state (design 6.2): the base from x, y and yaw, the six arm
    /// joints by name, and the wheels from the measured base twist with the plant profile's skid-steer mapping.
    /// The wheels are display only; nothing here feeds the backend.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class PlantRobotView : MonoBehaviour
    {
        public static readonly string[] ArmJointNames =
        {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        };

        [SerializeField] private float wheelRadius = 0.14f;
        [SerializeField] private float track = 3.0f;
        [SerializeField] private Transform payloadAnchor;
        [SerializeField] private ViewJoint[] arm = new ViewJoint[6];
        [SerializeField] private ViewJoint[] leftWheels = Array.Empty<ViewJoint>();
        [SerializeField] private ViewJoint[] rightWheels = Array.Empty<ViewJoint>();

        public Transform PayloadAnchor => payloadAnchor;
        public float WheelRadius => wheelRadius;
        public float Track => track;

        /// <summary>Binds the joints by name (setup command); throws when one is missing.</summary>
        public void Configure(float radius, float effectiveTrack, Transform payload)
        {
            if (!(radius > 0) || !(effectiveTrack > 0)) throw new ArgumentOutOfRangeException(nameof(radius));
            wheelRadius = radius;
            track = effectiveTrack;
            payloadAnchor = payload != null ? payload : throw new ArgumentNullException(nameof(payload));
            var joints = GetComponentsInChildren<ViewJoint>(true);
            arm = ArmJointNames.Select(n => joints.SingleOrDefault(j => j.JointName == n)
                ?? throw new InvalidOperationException($"the view robot has no {n}")).ToArray();
            leftWheels = joints.Where(j => j.JointName.EndsWith("left_wheel_joint", StringComparison.Ordinal)).ToArray();
            rightWheels = joints.Where(j => j.JointName.EndsWith("right_wheel_joint", StringComparison.Ordinal)).ToArray();
            if (leftWheels.Length == 0 || rightWheels.Length == 0) throw new InvalidOperationException("the view robot has no wheels");
        }

        public void Apply(in RobotSample s, float dt)
        {
            transform.SetPositionAndRotation(RosMapPose.ToUnityPosition(s.X, s.Y, 0f), RosMapPose.ToUnityRotation(s.Yaw));
            for (var i = 0; i < arm.Length && i < s.Joints.Length; ++i) arm[i].SetAngle((float)s.Joints[i]);
            if (!(dt > 0)) return;
            var left = (float)((s.V - s.Omega * track / 2) / wheelRadius * dt);
            var right = (float)((s.V + s.Omega * track / 2) / wheelRadius * dt);
            foreach (var w in leftWheels) w.SetAngle(w.Angle + left);
            foreach (var w in rightWheels) w.SetAngle(w.Angle + right);
        }
    }
}
