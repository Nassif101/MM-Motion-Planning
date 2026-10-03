using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    /// <summary>
    /// Publishes Unity ground truth for the base: the dynamic <c>odom -> base_footprint</c>
    /// transform and <c>nav_msgs/Odometry</c> on <c>/odom</c>, both from the same physics
    /// sample and canonical stamp.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class GroundTruthBaseTfPublisher : MonoBehaviour
    {
        // Ground truth: a small fixed variance keeps consumers that invert covariances finite.
        private const double GroundTruthVariance = 1e-6;

        [SerializeField]
        private string topicName = "/tf";

        [SerializeField]
        private string odometryTopicName = "/odom";

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
        private OdometryMsg odometryMessage;
        private ArticulationBody baseBody;
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

        /// <summary>
        /// Twist of the base_footprint point expressed in the base_footprint frame (ROS FLU).
        /// Inputs are Unity world-frame velocities of the base_link body and world poses.
        /// </summary>
        public static void ComputeFootprintTwist(
            Vector3 baseLinearVelocityWorld,
            Vector3 baseAngularVelocityWorld,
            Vector3 baseWorldPosition,
            Vector3 footprintWorldPosition,
            Quaternion footprintWorldRotation,
            out Vector3 linearFlu,
            out Vector3 angularFlu)
        {
            var footprintVelocityWorld = baseLinearVelocityWorld +
                Vector3.Cross(baseAngularVelocityWorld, footprintWorldPosition - baseWorldPosition);
            var inverse = Quaternion.Inverse(footprintWorldRotation);
            linearFlu = FLU.ConvertFromRUF(inverse * footprintVelocityWorld);
            angularFlu = FLU.ConvertAngularVelocityFromRUF(inverse * baseAngularVelocityWorld);
        }

        private void Awake()
        {
            ValidateConfiguration();
            baseBody = baseLink.GetComponent<ArticulationBody>();
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

            ros.RegisterPublisher<OdometryMsg>(odometryTopicName);
            odometryMessage = new OdometryMsg
            {
                header = RosTimeUtility.Header(Time.timeAsDouble, parentFrameId),
                child_frame_id = childFrameId,
                pose = new PoseWithCovarianceMsg { pose = new PoseMsg(), covariance = DiagonalCovariance() },
                twist = new TwistWithCovarianceMsg { twist = new TwistMsg(), covariance = DiagonalCovariance() }
            };

            var now = Time.timeAsDouble;
            nextPublishTime = now;
            previousTime = now;
        }

        private void FixedUpdate()
        {
            // SimulationClockPublisher runs first and owns the canonical tick.
            var now = RosTimeUtility.PhysicsTimeSeconds;
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

            var stamp = RosTimeUtility.FromSeconds(now);
            var translation = footprintPosition.To<FLU>();
            var rotation = footprintRotation.To<FLU>();
            transformMessage.header.stamp = stamp;
            transformMessage.transform.translation = translation;
            transformMessage.transform.rotation = rotation;
            ros.Publish(topicName, message);

            // A sleeping articulation is not integrated but keeps reporting its last residual
            // velocity (a constant 0.013 m/s was seen at rest), which would read as a moving base.
            var sleeping = baseBody.IsSleeping();
            ComputeFootprintTwist(
                sleeping ? Vector3.zero : baseBody.linearVelocity,
                sleeping ? Vector3.zero : baseBody.angularVelocity,
                baseLink.position,
                footprintPosition,
                footprintRotation,
                out var linear,
                out var angular);
            odometryMessage.header.stamp = stamp;
            odometryMessage.pose.pose.position = new PointMsg(translation.x, translation.y, translation.z);
            odometryMessage.pose.pose.orientation = rotation;
            odometryMessage.twist.twist.linear = new Vector3Msg(linear.x, linear.y, linear.z);
            odometryMessage.twist.twist.angular = new Vector3Msg(angular.x, angular.y, angular.z);
            ros.Publish(odometryTopicName, odometryMessage);
        }

        private static double[] DiagonalCovariance()
        {
            var covariance = new double[36];
            for (var index = 0; index < 6; index++)
            {
                covariance[index * 7] = GroundTruthVariance;
            }

            return covariance;
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
