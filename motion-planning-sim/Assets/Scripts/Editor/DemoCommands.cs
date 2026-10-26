using System;
using MotionPlanningSim.Environment;
using MotionPlanningSim.ROS;
using Unity.Pipeline.Commands;
using UnityEditor;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    /// <summary>
    /// Demo fixtures for tools/demo.py: draggable obstacle boxes for B5's free drive. They live under their own root,
    /// apart from the scenario runner's obstacles, and the saved scene is never modified.
    /// </summary>
    public static class DemoCommands
    {
        private const string RootName = "DemoDraggables";
        private const string BaseLinkPath = "MobileManipulator/base_footprint/base_link";

        [CliCommand("demo_draggable",
            "Create a box obstacle ahead of the robot (ahead m along its heading, bottom at z m, ROS size_x/size_y/" +
            "height m) that the demo's B5 avoids while it is dragged in the Scene view; selects it. Play only",
            MainThreadRequired = true)]
        public static object CreateDraggable(float ahead = 1.5f, float z = 1.0f, float size_x = 0.4f,
            float size_y = 0.4f, float height = 0.4f)
        {
            if (!Application.isPlaying)
            {
                throw new InvalidOperationException("Enter Play before creating draggable obstacles.");
            }

            if (!(size_x > 0 && size_y > 0 && height > 0 && size_x <= 2 && size_y <= 2 && height <= 2 &&
                  z >= 0 && z <= 2.5f && ahead >= 0 && ahead <= 10))
            {
                throw new ArgumentOutOfRangeException(nameof(size_x),
                    "Sizes in (0, 2] m, bottom z in [0, 2.5] m, ahead in [0, 10] m.");
            }

            var robot = GameObject.Find(BaseLinkPath) ??
                        throw new InvalidOperationException($"No {BaseLinkPath} in the scene.");
            var heading = Vector3.ProjectOnPlane(robot.transform.forward, Vector3.up).normalized;
            var root = GameObject.Find(RootName) ?? new GameObject(RootName);
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = $"draggable_{root.transform.childCount + 1}";
            box.transform.SetParent(root.transform, false);
            var position = robot.transform.position + heading * ahead;
            box.transform.position = new Vector3(position.x, z + height / 2f, position.z);
            // ROS x (forward) is Unity z and ROS y is -Unity x: axis-aligned in the map.
            box.transform.localScale = new Vector3(size_y, height, size_x);
            box.GetComponent<Renderer>().material.color = new Color(1.0f, 0.5f, 0.0f);
            var body = box.AddComponent<Rigidbody>();
            body.isKinematic = true;  // dragged by hand; touches the robot as a contact
            box.AddComponent<DraggableObstacle>();
            Physics.SyncTransforms();
            Selection.activeGameObject = box;
            SceneView.lastActiveSceneView?.FrameSelected();
            var ros = box.GetComponent<DraggableObstacle>().ToRos();
            return new
            {
                name = box.name, x = ros.Center.x, y = ros.Center.y, z = ros.Center.z, topic = DraggableObstaclePublisher.Topic,
            };
        }

        [CliCommand("demo_draggable_clear", "Remove every draggable obstacle box", MainThreadRequired = true)]
        public static object ClearDraggables()
        {
            var root = GameObject.Find(RootName);
            var removed = root != null ? root.transform.childCount : 0;
            if (root != null)
            {
                UnityEngine.Object.Destroy(root);
            }

            return new { removed };
        }
    }
}
