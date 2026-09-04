using System;
using System.Collections.Generic;
using UnityEngine;

namespace MotionPlanningSim.Control
{
    /// <summary>
    /// Torque-limited commissioning hold for otherwise passive arm joints.
    /// Disable it before enabling a real ROS arm controller.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ArmJointHoldController : MonoBehaviour
    {
        [Header("Optional Arm Hold")]
        [SerializeField, Tooltip(
            "Hold the six arm joints at their pose when Play Mode starts. " +
            "Disable this before using a real arm controller.")]
        private bool holdArmJoints = true;

        [SerializeField]
        private ArticulationBody[] armJoints = Array.Empty<ArticulationBody>();

        [SerializeField, Min(0.0f)]
        private float holdStiffness = 800.0f;

        [SerializeField, Min(0.0f)]
        private float holdDamping = 80.0f;

        private ArticulationDrive[] originalDrives = Array.Empty<ArticulationDrive>();
        private bool initialized;
        private bool holdApplied;

        public bool HoldArmJoints
        {
            get => holdArmJoints;
            set => holdArmJoints = value;
        }

        public IReadOnlyList<ArticulationBody> ArmJoints => armJoints;
        public bool HoldApplied => holdApplied;

        public void Configure(ArticulationBody[] configuredArmJoints)
        {
            armJoints = configuredArmJoints != null
                ? (ArticulationBody[])configuredArmJoints.Clone()
                : throw new ArgumentNullException(nameof(configuredArmJoints));
            ValidateConfiguration();
        }

        private void Awake()
        {
            ValidateConfiguration();
            originalDrives = new ArticulationDrive[armJoints.Length];
            for (var index = 0; index < armJoints.Length; index++)
            {
                originalDrives[index] = armJoints[index].xDrive;
            }

            initialized = true;
        }

        private void Start()
        {
            SynchronizeHoldState();
        }

        private void FixedUpdate()
        {
            SynchronizeHoldState();
        }

        private void OnEnable()
        {
            if (initialized)
            {
                SynchronizeHoldState();
            }
        }

        private void OnDisable()
        {
            RestoreOriginalDrives();
        }

        private void OnValidate()
        {
            holdStiffness = Mathf.Max(0.0f, holdStiffness);
            holdDamping = Mathf.Max(0.0f, holdDamping);
        }

        private void SynchronizeHoldState()
        {
            if (holdArmJoints == holdApplied)
            {
                return;
            }

            if (holdArmJoints)
            {
                ApplyHoldAtCurrentPose();
            }
            else
            {
                RestoreOriginalDrives();
            }
        }

        private void ApplyHoldAtCurrentPose()
        {
            for (var index = 0; index < armJoints.Length; index++)
            {
                var joint = armJoints[index];
                var drive = joint.xDrive;
                drive.driveType = ArticulationDriveType.Force;
                drive.target = joint.jointPosition[0] * Mathf.Rad2Deg;
                drive.targetVelocity = 0.0f;
                drive.stiffness = holdStiffness;
                drive.damping = holdDamping;
                // Preserve the URDF effort-derived forceLimit.
                joint.xDrive = drive;
            }

            holdApplied = true;
        }

        private void RestoreOriginalDrives()
        {
            if (!initialized || !holdApplied)
            {
                return;
            }

            for (var index = 0; index < armJoints.Length; index++)
            {
                if (armJoints[index] != null)
                {
                    armJoints[index].xDrive = originalDrives[index];
                }
            }

            holdApplied = false;
        }

        private void ValidateConfiguration()
        {
            if (armJoints == null || armJoints.Length != 6)
            {
                throw new InvalidOperationException(
                    "Arm hold requires exactly the six movable arm ArticulationBody joints.");
            }

            var uniqueJoints = new HashSet<ArticulationBody>();
            foreach (var joint in armJoints)
            {
                if (joint == null ||
                    joint.jointType != ArticulationJointType.RevoluteJoint ||
                    joint.dofCount != 1 ||
                    !uniqueJoints.Add(joint))
                {
                    throw new InvalidOperationException(
                        "Arm hold references must be six unique one-DOF revolute joints.");
                }
            }

            if (holdStiffness <= 0.0f || holdDamping <= 0.0f)
            {
                throw new InvalidOperationException(
                    "Arm hold stiffness and damping must be positive.");
            }
        }
    }
}
