using System.Collections.Generic;
using MotionPlanningSim.Environment;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Visualization;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    /// <summary>
    /// Publishes the enabled <see cref="DraggableObstacle"/> boxes as a <c>visualization_msgs/MarkerArray</c> of
    /// cubes in the ROS <c>map</c> frame (centre, yaw, full size), stamped with physics time, at 20 Hz. Created by
    /// the first draggable box and only then: scenes and scenario runs never publish the topic.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class DraggableObstaclePublisher : MonoBehaviour
    {
        public const string Topic = "/demo/draggables";
        private const double FrequencyHz = 20.0;
        private const string Namespace = "demo";

        private static readonly List<DraggableObstacle> Obstacles = new();
        private static DraggableObstaclePublisher instance;
        private static int nextId;

        private ROSConnection ros;
        private double nextPublishTime;
        private double previousTime;

        public static void Register(DraggableObstacle obstacle)
        {
            if (!Obstacles.Contains(obstacle))
            {
                Obstacles.Add(obstacle);
            }

            if (obstacle.Id == 0)
            {
                obstacle.Id = ++nextId;
            }

            if (instance == null && Application.isPlaying)
            {
                instance = new GameObject(nameof(DraggableObstaclePublisher)).AddComponent<DraggableObstaclePublisher>();
            }
        }

        public static void Unregister(DraggableObstacle obstacle) => Obstacles.Remove(obstacle);

        /// <summary>One cube marker for a box (RViz drops it 0.5 s after the last one).</summary>
        public static MarkerMsg Marker(RosBox box, int id, HeaderMsg header)
        {
            return new MarkerMsg
            {
                header = header,
                ns = Namespace,
                id = id,
                type = MarkerMsg.CUBE,
                action = MarkerMsg.ADD,
                pose = new PoseMsg(new PointMsg(box.Center.x, box.Center.y, box.Center.z),
                    new QuaternionMsg(0, 0, Mathf.Sin(box.Yaw / 2), Mathf.Cos(box.Yaw / 2))),
                scale = new Vector3Msg(box.Size.x, box.Size.y, box.Size.z),
                color = new ColorRGBAMsg(1.0f, 0.5f, 0.0f, 0.8f),
                lifetime = new DurationMsg(0, 500_000_000),
            };
        }

        private void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<MarkerArrayMsg>(Topic);
        }

        private void OnDestroy()
        {
            if (instance == this)
            {
                instance = null;
            }
        }

        private void FixedUpdate()
        {
            var now = RosTimeUtility.PhysicsTimeSeconds;
            if (ros == null || Obstacles.Count == 0 ||
                !PublicationSchedule.IsDue(now, FrequencyHz, ref nextPublishTime, ref previousTime))
            {
                return;
            }

            var header = RosTimeUtility.Header(now, "map");
            var markers = new MarkerMsg[Obstacles.Count];
            for (var i = 0; i < Obstacles.Count; i++)
            {
                markers[i] = Marker(Obstacles[i].ToRos(), Obstacles[i].Id, header);
            }

            ros.Publish(Topic, new MarkerArrayMsg(markers));
        }
    }
}
