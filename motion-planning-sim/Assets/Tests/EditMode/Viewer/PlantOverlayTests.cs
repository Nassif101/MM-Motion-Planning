using System;
using System.Linq;
using MotionPlanningSim.Viewer;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>4B-1 Task 4: the plan overlays' geometry from a probe copy of the view robot.</summary>
    public sealed class PlantOverlayTests
    {
        static PlantRobotView NewRobot()
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(ViewerSceneContractTests.PrefabPath);
            return ((GameObject)PrefabUtility.InstantiatePrefab(prefab)).GetComponent<PlantRobotView>();
        }

        // The panel's corners (ROS map frame) are a 1.2 m square at the tool, and follow the planned base and joints
        [Test]
        public void PanelCornersFollowThePlannedJoints()
        {
            var robot = NewRobot();
            var panel = PlantPlanOverlay.FindPanel(robot);
            Assert.IsNotNull(panel, "tool0/PayloadPanel");
            foreach (var sample in new[]
                     {
                         new double[] { 0, 0, 0, 0, 0.968, -1.735, 0, 0.767, 0 },  // carry
                         new double[] { 3, -2, 1.2, 0.5, 0.4, -1.2, 0.3, 1.0, -0.4 },
                     })
            {
                var c = PlantPlanOverlay.PanelCorners(robot, panel, sample);
                Assert.AreEqual(4, c.Length);
                var edges = new[] { (c[0] - c[1]).Length(), (c[1] - c[2]).Length(), (c[2] - c[3]).Length(), (c[3] - c[0]).Length() };
                foreach (var e in edges) Assert.AreEqual(1.2, e, 0.002, "a 1.2 m square face");
                var centre = (c[0] + c[1] + c[2] + c[3]) / 4f;
                var tool = robot.PayloadAnchor.position;  // Unity world -> ROS map: (z, -x, y)
                var toolRos = new System.Numerics.Vector3(tool.z, -tool.x, tool.y);
                Assert.Less((centre - toolRos).Length(), 0.15f, "the panel sits at tool0");
                var baseRos = new System.Numerics.Vector3((float)sample[0], (float)sample[1], 0f);
                Assert.Less((new System.Numerics.Vector3(centre.X, centre.Y, 0) - baseRos).Length(), 2.0f, "near the planned base");
            }
        }

        [Test]
        public void GhostsAreCappedAtTwelve()
        {
            Assert.AreEqual(12, PlantPlanOverlay.ClampGhosts(40));
            Assert.AreEqual(1, PlantPlanOverlay.ClampGhosts(0));
            Assert.AreEqual(6, PlantPlanOverlay.ClampGhosts(6));
        }
    }
}
