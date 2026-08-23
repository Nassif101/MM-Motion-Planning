using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    [DisallowMultipleComponent]
    public sealed class GroundTruthBaseTfPublisher : MonoBehaviour
    {
        [SerializeField]
        private string topicName = "/tf";

        [SerializeField]
        private string parentFrameId = "odom";

        [SerializeField]
        private string childFrameId = "base_footprint";

        [SerializeField, Min(1.0f)]
        private float frequencyHz = 50.0f;

        [SerializeField]
        private Transform baseLink;

        private ROSConnection ros;
        private TFMessageMsg message;
        private TransformStampedMsg transformMessage;
        private Vector3 basePositionInFootprint;
        private Quaternion baseRotationInFootprint;
        private double nextPublishTime;
        private double previousTime;

        public Transform BaseLink => baseLink;

        public void Configure(Transform configuredBaseLink)
        {
            baseLink = configuredBaseLink != null
                ? configuredBaseLink
                : throw new ArgumentNullException(nameof(configuredBaseLink));
            ValidateConfiguration();
        }

        public static void ComputeFootprintWorldPose(
            Vector3 baseWorldPosition,
            Quaternion baseWorldRotation,
            Vector3 basePositionRelativeToFootprint,
            Quaternion baseRotationRelativeToFootprint,
            out Vector3 footprintWorldPosition,
            out Quaternion footprintWorldRotation)
        {
            footprintWorldRotation =
                baseWorldRotation * Quaternion.Inverse(baseRotationRelativeToFootprint);
            footprintWorldRotation.Normalize();
            footprintWorldPosition =
                baseWorldPosition -
                footprintWorldRotation * basePositionRelativeToFootprint;
        }

        private void Awake()
        {
            ValidateConfiguration();
            basePositionInFootprint = baseLink.localPosition;
            baseRotationInFootprint = baseLink.localRotation;
        }

        private void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<TFMessageMsg>(topicName);

            transformMessage = new TransformStampedMsg
            {
                header = RosTimeUtility.Header(Time.timeAsDouble, parentFrameId),
                child_frame_id = childFrameId,
                transform = new TransformMsg()
            };
            message = new TFMessageMsg(new[] { transformMessage });

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

            ComputeFootprintWorldPose(
                baseLink.position,
                baseLink.rotation,
                basePositionInFootprint,
                baseRotationInFootprint,
                out var footprintPosition,
                out var footprintRotation);

            transformMessage.header.stamp = RosTimeUtility.FromSeconds(now);
            transformMessage.transform.translation = footprintPosition.To<FLU>();
            transformMessage.transform.rotation = footprintRotation.To<FLU>();
            ros.Publish(topicName, message);
        }

        private void ValidateConfiguration()
        {
            if (baseLink == null)
            {
                throw new InvalidOperationException(
                    "A physical base_link Transform is required.");
            }

            if (baseLink.parent == null || baseLink.parent.name != childFrameId)
            {
                throw new InvalidOperationException(
                    $"base_link must be a direct child of {childFrameId}.");
            }

            var articulation = baseLink.GetComponent<ArticulationBody>();
            if (articulation == null || !articulation.isRoot)
            {
                throw new InvalidOperationException(
                    "base_link must be the physical root ArticulationBody.");
            }
        }
    }
}
