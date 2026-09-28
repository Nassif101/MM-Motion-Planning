using System;
using System.Linq;
using MotionPlanningSim.Control;
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
    }
}
