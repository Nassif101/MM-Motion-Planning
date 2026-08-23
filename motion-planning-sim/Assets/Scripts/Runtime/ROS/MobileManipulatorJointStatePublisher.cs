using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    [DisallowMultipleComponent]
    public sealed class MobileManipulatorJointStatePublisher : MonoBehaviour
    {
        [SerializeField]
        private string topicName = "/joint_states";

        [SerializeField, Min(1.0f)]
        private float frequencyHz = 50.0f;

        [SerializeField]
        private ArticulationBody[] joints = Array.Empty<ArticulationBody>();

        [SerializeField]
        private string[] jointNames = Array.Empty<string>();

        private ROSConnection ros;
        private JointStateMsg message;
        private double[] positions;
        private double[] velocities;
        private double nextPublishTime;
        private double previousTime;

        public int JointCount => joints?.Length ?? 0;
        public string[] JointNames => jointNames;

        public void Configure(
            ArticulationBody[] configuredJoints,
            string[] configuredJointNames)
        {
            joints = configuredJoints ?? throw new ArgumentNullException(
                nameof(configuredJoints));
            jointNames = configuredJointNames ?? throw new ArgumentNullException(
                nameof(configuredJointNames));
            ValidateConfiguration();
        }

        private void Awake()
        {
            ValidateConfiguration();
            positions = new double[joints.Length];
            velocities = new double[joints.Length];
        }

        private void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<JointStateMsg>(topicName);

            message = new JointStateMsg(
                RosTimeUtility.Header(Time.timeAsDouble),
                jointNames,
                positions,
                velocities,
                Array.Empty<double>());

            var now = Time.timeAsDouble;
            nextPublishTime = now;
            previousTime = now;
        }

        private void FixedUpdate()
        {
            var now = Time.fixedTimeAsDouble;
            if (!PublicationSchedule.IsDue(
                    now,
                    frequencyHz,
                    ref nextPublishTime,
                    ref previousTime))
            {
                return;
            }

            for (var index = 0; index < joints.Length; index++)
            {
                var joint = joints[index];
                positions[index] = joint.jointPosition[0];
                velocities[index] = joint.jointVelocity[0];
            }

            message.header.stamp = RosTimeUtility.FromSeconds(now);
            ros.Publish(topicName, message);
        }

        private void ValidateConfiguration()
        {
            if (joints == null || jointNames == null || joints.Length != 10 ||
                jointNames.Length != joints.Length)
            {
                throw new InvalidOperationException(
                    "The mobile manipulator joint-state publisher requires " +
                    "exactly ten paired joints and URDF joint names.");
            }

            var uniqueNames = new System.Collections.Generic.HashSet<string>(
                StringComparer.Ordinal);
            for (var index = 0; index < joints.Length; index++)
            {
                if (joints[index] == null)
                {
                    throw new InvalidOperationException(
                        $"Joint reference {index} is missing.");
                }

                if (joints[index].jointType != ArticulationJointType.RevoluteJoint ||
                    joints[index].dofCount != 1)
                {
                    throw new InvalidOperationException(
                        $"{joints[index].name} must be a one-DOF revolute articulation.");
                }

                if (string.IsNullOrWhiteSpace(jointNames[index]) ||
                    !uniqueNames.Add(jointNames[index]))
                {
                    throw new InvalidOperationException(
                        $"Joint name {index} is empty or duplicated.");
                }
            }
        }
    }
}
