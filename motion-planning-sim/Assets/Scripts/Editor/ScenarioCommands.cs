using System;
using System.Linq;
using MotionPlanningSim.Control;
using MotionPlanningSim.Environment;
using MotionPlanningSim.ROS;
using Unity.Pipeline.Commands;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    /// <summary>
    /// Experiment fixtures for the navigation scenario runner (tools/run_nav_scenario.py).
    /// Compiled commands avoid Roslyn eval stalls during runs; the saved scene is never modified.
    /// </summary>
    public static class ScenarioCommands
    {
        private const float BaseLinkHeight = 0.21f;
        private const double PoseTolerance = 0.04;

        private static readonly (string name, double[] q)[] ArmPoses =
        {
            ("home", new double[] { 0, 0, 0, 0, 0, 0 }),
            ("vertical_carry", new[] { Math.PI / 2, 0, 0, 0, Math.PI / 2, 0 }),
        };

        [CliCommand("scenario_place",
            "Teleport the stopped robot to a ROS map pose (x, y, yaw) in Play only; " +
            "requires fresh HOLD in the named arm pose",
            MainThreadRequired = true)]
        public static object Place(double x, double y, double yaw, string arm_pose = "home")
        {
            if (!Application.isPlaying)
            {
                throw new InvalidOperationException("Enter Play before placing a scenario fixture.");
            }

            var expected = ArmPoses.FirstOrDefault(p => p.name == arm_pose).q
                ?? throw new ArgumentException(
                    $"Unknown arm_pose '{arm_pose}'; expected {string.Join(", ", ArmPoses.Select(p => p.name))}.");
            var arm = UnityEngine.Object.FindFirstObjectByType<ArmActuatorController>()
                ?? throw new InvalidOperationException("Arm missing.");
            var baseLink = arm.GetComponentsInChildren<ArticulationBody>().Single(b => b.name == "base_link");
            if (arm.State != ArmActuatorState.HOLD || arm.CommandAge < 0 || arm.CommandAge > 0.5 ||
                baseLink.linearVelocity.magnitude > 0.05 || baseLink.angularVelocity.magnitude > 0.05 ||
                Enumerable.Range(0, 6).Any(i => Math.Abs(arm.Position(i) - expected[i]) > PoseTolerance))
            {
                throw new InvalidOperationException($"Requires fresh, stopped {arm_pose} HOLD.");
            }

            baseLink.TeleportRoot(
                RosMapPose.ToUnityPosition(x, y, BaseLinkHeight),
                RosMapPose.ToUnityRotation(yaw));
            baseLink.linearVelocity = Vector3.zero;
            baseLink.angularVelocity = Vector3.zero;
            return new { placed = true, x, y, yaw, arm_pose, physicsTime = RosTimeUtility.PhysicsTimeSeconds };
        }

        [CliCommand("scenario_contacts_reset",
            "Start (or restart) recording robot-environment contacts in Play only",
            MainThreadRequired = true)]
        public static object ResetContacts()
        {
            var monitor = ContactMonitor(create: true);
            monitor.ResetMonitor();
            return new { reset = true, physicsTime = RosTimeUtility.PhysicsTimeSeconds };
        }

        [CliCommand("scenario_contacts",
            "Report robot-environment contacts recorded since scenario_contacts_reset",
            MainThreadRequired = true)]
        public static object Contacts()
        {
            var monitor = ContactMonitor(create: false);
            var contacts = monitor.Ledger.Entries
                .OrderByDescending(e => e.MaxPenetration)
                .Select(e => new
                {
                    robot = e.RobotCollider, other = e.Other,
                    maxPenetration = Math.Round(e.MaxPenetration, 4), firstTime = e.FirstTime, ticks = e.Ticks
                })
                .ToArray();
            return new
            {
                contact = contacts.Length > 0,
                ticks = monitor.Ticks,
                bufferOverflowed = monitor.BufferOverflowed,
                contacts,
                physicsTime = RosTimeUtility.PhysicsTimeSeconds
            };
        }

        private static RobotContactMonitor ContactMonitor(bool create)
        {
            if (!Application.isPlaying)
            {
                throw new InvalidOperationException("Enter Play before recording contacts.");
            }

            var arm = UnityEngine.Object.FindFirstObjectByType<ArmActuatorController>()
                ?? throw new InvalidOperationException("Robot missing.");
            var monitor = arm.GetComponent<RobotContactMonitor>();
            if (monitor == null)
            {
                monitor = create
                    ? arm.gameObject.AddComponent<RobotContactMonitor>()
                    : throw new InvalidOperationException("Call scenario_contacts_reset first.");
            }

            return monitor;
        }

        private const string ObstacleRootName = "ScenarioObstacles";

        [CliCommand("scenario_obstacle",
            "Create or replace a named box obstacle at a ROS map position (x, y), sized in ROS " +
            "x/y/height metres, in Play only; it is not part of the static map",
            MainThreadRequired = true)]
        public static object PlaceObstacle(string name, double x, double y,
            float size_x = 0.5f, float size_y = 0.5f, float height = 0.5f)
        {
            if (!Application.isPlaying)
            {
                throw new InvalidOperationException("Enter Play before placing scenario obstacles.");
            }

            if (string.IsNullOrWhiteSpace(name) || name.Any(c => !char.IsLetterOrDigit(c) && c != '-' && c != '_'))
            {
                throw new ArgumentException("Obstacle name must be alphanumeric with '-' or '_'.");
            }

            if (!(size_x > 0 && size_y > 0 && height > 0 && size_x <= 5 && size_y <= 5 && height <= 3))
            {
                throw new ArgumentOutOfRangeException(nameof(size_x), "Obstacle sizes must be in (0, 5] m, height in (0, 3] m.");
            }

            var root = GameObject.Find(ObstacleRootName) ?? new GameObject(ObstacleRootName);
            var existing = root.transform.Find(name);
            if (existing != null)
            {
                UnityEngine.Object.Destroy(existing.gameObject);
            }

            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name;
            box.transform.SetParent(root.transform, false);
            // ROS x (forward) is Unity z and ROS y (left) is -Unity x; axis-aligned in the map.
            box.transform.position = RosMapPose.ToUnityPosition(x, y, height / 2f);
            box.transform.localScale = new Vector3(size_y, height, size_x);
            Physics.SyncTransforms();
            return new { name, x, y, size_x, size_y, height, physicsTime = RosTimeUtility.PhysicsTimeSeconds };
        }

        [CliCommand("scenario_mover",
            "Create or replace a named mover (worker or cart box) that crosses from (start_x, start_y) " +
            "to (end_x, end_y) in ROS map metres at speed m/s, starting when the robot comes within " +
            "trigger_distance m of the segment midpoint, in Play only",
            MainThreadRequired = true)]
        public static object PlaceMover(string name, double start_x, double start_y, double end_x, double end_y,
            float size_x = 0.5f, float size_y = 0.5f, float height = 1.8f, double speed = 0.8,
            double trigger_distance = 2.5, int crossings = 1)
        {
            if (!Application.isPlaying)
            {
                throw new InvalidOperationException("Enter Play before placing scenario movers.");
            }

            if (string.IsNullOrWhiteSpace(name) || name.Any(c => !char.IsLetterOrDigit(c) && c != '-' && c != '_'))
            {
                throw new ArgumentException("Mover name must be alphanumeric with '-' or '_'.");
            }

            if (!(size_x > 0 && size_y > 0 && height > 0 && size_x <= 5 && size_y <= 5 && height <= 3))
            {
                throw new ArgumentOutOfRangeException(nameof(size_x), "Mover sizes must be in (0, 5] m, height in (0, 3] m.");
            }

            var motion = new ScenarioMoverMotion(start_x, start_y, end_x, end_y, speed, trigger_distance, crossings);
            var arm = UnityEngine.Object.FindFirstObjectByType<ArmActuatorController>()
                ?? throw new InvalidOperationException("Robot missing.");
            var baseLink = arm.GetComponentsInChildren<ArticulationBody>().Single(b => b.name == "base_link");

            var root = GameObject.Find(ObstacleRootName) ?? new GameObject(ObstacleRootName);
            if (root.GetComponent<ScenarioMoverPublisher>() == null)
            {
                root.AddComponent<ScenarioMoverPublisher>();
            }

            var existing = root.transform.Find(name);
            if (existing != null)
            {
                existing.gameObject.SetActive(false);
                UnityEngine.Object.Destroy(existing.gameObject);
            }

            var mover = GameObject.CreatePrimitive(PrimitiveType.Cube);
            mover.name = name;
            mover.transform.SetParent(root.transform, false);
            mover.GetComponent<Renderer>().material.SetColor("_BaseColor", new Color(1f, 0.45f, 0f));  // hi-vis
            mover.AddComponent<Rigidbody>();
            mover.AddComponent<ScenarioMover>().Configure(motion, size_x, size_y, height, baseLink.transform);
            return new
            {
                name, start_x, start_y, end_x, end_y, size_x, size_y, height, speed, trigger_distance, crossings,
                physicsTime = RosTimeUtility.PhysicsTimeSeconds
            };
        }

        [CliCommand("scenario_movers",
            "Report each scenario mover's trigger time, walked distance, completion and time spent " +
            "waiting for the robot, in Play only",
            MainThreadRequired = true)]
        public static object Movers()
        {
            if (!Application.isPlaying)
            {
                throw new InvalidOperationException("Enter Play before reading scenario movers.");
            }

            var root = GameObject.Find(ObstacleRootName);
            var movers = root == null
                ? Array.Empty<object>()
                : root.GetComponentsInChildren<ScenarioMover>()
                    .Where(m => m.Motion != null)
                    .OrderBy(m => m.name, StringComparer.Ordinal)
                    .Select(m => (object)new
                    {
                        name = m.name,
                        triggerTime = m.Motion.TriggerTime,
                        walked = Math.Round(m.Motion.Walked, 3),
                        finished = m.Motion.Finished,
                        blockedSeconds = Math.Round(m.Motion.BlockedSeconds, 2),
                        x = Math.Round(m.Motion.X, 3),
                        y = Math.Round(m.Motion.Y, 3)
                    })
                    .ToArray();
            return new { movers, physicsTime = RosTimeUtility.PhysicsTimeSeconds };
        }

        [CliCommand("scenario_obstacle_clear",
            "Remove one named scenario obstacle or mover, or all of them when name is omitted, in Play only",
            MainThreadRequired = true)]
        public static object ClearObstacles(string name = "")
        {
            var root = GameObject.Find(ObstacleRootName);
            var removed = 0;
            if (root != null)
            {
                foreach (Transform child in root.transform)
                {
                    if (string.IsNullOrEmpty(name) || child.name == name)
                    {
                        child.gameObject.SetActive(false);
                        UnityEngine.Object.Destroy(child.gameObject);
                        removed++;
                    }
                }
            }

            Physics.SyncTransforms();
            return new { removed, physicsTime = RosTimeUtility.PhysicsTimeSeconds };
        }
    }
}
