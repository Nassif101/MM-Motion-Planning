using System.Linq;
using MotionPlanningSim.Environment;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    /// <summary>
    /// Publishes the ground-truth positions of the scenario movers under this object as a
    /// <c>geometry_msgs/PoseArray</c> in the ROS <c>map</c> frame (box centres, sorted by name),
    /// stamped with physics time. Scenario metrics only; navigation never reads it.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ScenarioMoverPublisher : MonoBehaviour
    {
        public const string Topic = "/scenario/movers";
        private const double FrequencyHz = 20.0;

        private ROSConnection ros;
        private double nextPublishTime;
        private double previousTime;

        private void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<PoseArrayMsg>(Topic);
        }

        private void FixedUpdate()
        {
            var now = RosTimeUtility.PhysicsTimeSeconds;
            if (ros == null || !PublicationSchedule.IsDue(now, FrequencyHz, ref nextPublishTime, ref previousTime))
            {
                return;
            }

            var poses = GetComponentsInChildren<ScenarioMover>()
                .Where(m => m.Motion != null)
                .OrderBy(m => m.name, System.StringComparer.Ordinal)
                .Select(m => new PoseMsg(new PointMsg(m.Motion.X, m.Motion.Y, m.Height / 2.0), new QuaternionMsg(0, 0, 0, 1)))
                .ToArray();
            ros.Publish(Topic, new PoseArrayMsg(RosTimeUtility.Header(now, "map"), poses));
        }
    }
}
