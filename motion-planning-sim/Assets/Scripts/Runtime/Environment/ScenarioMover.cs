using System;
using System.Linq;
using MotionPlanningSim.ROS;
using UnityEngine;

namespace MotionPlanningSim.Environment
{
    /// <summary>
    /// A kinematic box (worker or cart) that crosses the robot's route for a navigation
    /// scenario, moved every physics tick by <see cref="ScenarioMoverMotion"/>. It is visible
    /// to the lidar like any obstacle and is not part of the static map. Like a person, it
    /// waits while the robot is within <see cref="StandOff"/> of its next position instead of
    /// pushing through it; the waiting time is reported.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody), typeof(BoxCollider))]
    public sealed class ScenarioMover : MonoBehaviour
    {
        public const float StandOff = 0.1f;

        private Rigidbody body;
        private BoxCollider box;
        private Transform robotBase;
        private Transform robotRoot;
        private readonly Collider[] overlaps = new Collider[32];

        public ScenarioMoverMotion Motion { get; private set; }
        public float SizeX { get; private set; }
        public float SizeY { get; private set; }
        public float Height { get; private set; }

        public void Configure(ScenarioMoverMotion motion, float sizeX, float sizeY, float height, Transform configuredRobotBase)
        {
            Motion = motion ?? throw new ArgumentNullException(nameof(motion));
            robotBase = configuredRobotBase != null ? configuredRobotBase : throw new ArgumentNullException(nameof(configuredRobotBase));
            robotRoot = robotBase.root;
            (SizeX, SizeY, Height) = (sizeX, sizeY, height);
            body = GetComponent<Rigidbody>();
            body.isKinematic = true;
            body.interpolation = RigidbodyInterpolation.None;
            box = GetComponent<BoxCollider>();
            // ROS x (forward) is Unity z and ROS y (left) is -Unity x; axis-aligned in the map.
            transform.localScale = new Vector3(sizeY, height, sizeX);
            transform.position = RosMapPose.ToUnityPosition(motion.X, motion.Y, height / 2f);
            Physics.SyncTransforms();
        }

        private void FixedUpdate()
        {
            if (Motion == null || robotBase == null)
            {
                return;
            }

            var time = RosTimeUtility.PhysicsTimeSeconds;
            var robot = robotBase.position;
            var (x, y) = Motion.Next(time, robot.z, -robot.x);
            if (x == Motion.X && y == Motion.Y)
            {
                Motion.Commit(time, x, y);
                return;
            }

            var target = RosMapPose.ToUnityPosition(x, y, Height / 2f);
            if (RobotWithin(target))
            {
                Motion.Hold(time);
                return;
            }

            body.MovePosition(target);
            Motion.Commit(time, x, y);
        }

        private bool RobotWithin(Vector3 centre)
        {
            var half = Vector3.Scale(box.size, transform.lossyScale) / 2f + Vector3.one * StandOff;
            var count = Physics.OverlapBoxNonAlloc(centre, half, overlaps, transform.rotation, ~0, QueryTriggerInteraction.Ignore);
            return overlaps.Take(count).Any(c => c.transform.root == robotRoot);
        }
    }
}
