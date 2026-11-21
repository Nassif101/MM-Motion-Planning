using System;
using System.Linq;
using MotionPlanningSim.Viewer;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
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

        // Task 4 review I1: paused, no new scan rebuilds the lidar squares; they must re-face a turning camera
        [Test]
        public void LidarSquaresReorientWhenTheCameraTurns()
        {
            Assert.IsFalse(PlantLidarOverlay.NeedsReorient(Vector3.forward, Vector3.forward));
            Assert.IsFalse(PlantLidarOverlay.NeedsReorient(Vector3.forward, Quaternion.Euler(0, 8, 0) * Vector3.forward));
            Assert.IsTrue(PlantLidarOverlay.NeedsReorient(Vector3.forward, Quaternion.Euler(0, 15, 0) * Vector3.forward));
            Assert.IsTrue(PlantLidarOverlay.NeedsReorient(Vector3.forward, Vector3.right));
        }

        static Frame Prediction(int stages, string status = "ok")
        {
            var xs = new float[(stages + 1) * 9];
            for (var k = 0; k <= stages; ++k) xs[9 * k] = (float)(0.05 * k);  // 0.5 m/s along x
            return new Frame
            {
                Type = FrameType.Prediction,
                Header = new JObject { ["t"] = 3.0, ["status"] = status, ["stage_dt"] = 0.1, ["stages"] = stages, ["plan_id"] = 0 },
                Blocks = new[] { xs },
            };
        }

        // 4B-2: B5's prediction ghosts at 0.5, 1, 1.5 and 2 s, only within the horizon received
        [Test]
        public void PredictionGhostsAtHalfSecondSteps()
        {
            CollectionAssert.AreEqual(new[] { 0.5, 1.0, 1.5, 2.0 }, PlantPredictionOverlay.GhostTimes);
            var ghosts = PlantPredictionOverlay.GhostStates(B5Prediction.FromFrame(Prediction(20)));
            CollectionAssert.AreEqual(new[] { 0.25, 0.5, 0.75, 1.0 }, ghosts.Select(g => Math.Round(g[0], 5)).ToArray());
            var shorter = PlantPredictionOverlay.GhostStates(B5Prediction.FromFrame(Prediction(10)));
            Assert.AreEqual(2, shorter.Count, "a 1 s horizon: the 0.5 and 1 s ghosts");
            Assert.AreEqual(0, PlantPredictionOverlay.GhostStates(null).Count);
            // held and executed differ in colour, and both differ from F2's plan blue
            var executed = PlantPredictionOverlay.Colour(held: false, stale: false);
            var held = PlantPredictionOverlay.Colour(held: true, stale: false);
            Assert.AreNotEqual(executed, held);
            Assert.AreNotEqual(new Color(0.35f, 0.75f, 1f), executed);
            Assert.AreNotEqual(held, PlantPredictionOverlay.Colour(held: true, stale: true), "stale is grey");
        }

        // 4B-2: F2's ghosts at evenly spaced plan times from the display samples; F2's samples without them
        [Test]
        public void F2GhostsUseDisplayTimes()
        {
            var display = new float[41 * 10];  // 4 s; the base stands for the first 2 s (an arm turn), then drives
            for (var k = 0; k <= 40; ++k)
            {
                display[10 * k] = (float)(0.1 * k);
                display[10 * k + 1] = (float)(k <= 20 ? 0.0 : 0.05 * (k - 20));
            }
            var samples = new float[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };  // F2's two base samples
            Frame plans(bool withDisplay) => new Frame
            {
                Type = FrameType.Plans,
                Header = new JObject { ["t"] = 1.0, ["plan_id"] = 3, ["status"] = "ok" },
                Blocks = withDisplay ? new[] { samples, display } : new[] { samples },
            };
            var timed = PlantPlanOverlay.GhostStates(MediumPlan.FromFrame(plans(true)), 4);
            CollectionAssert.AreEqual(new[] { 0.0, 0.0, 0.5, 1.0 }, timed.Select(g => Math.Round(g[0], 5)).ToArray());
            var spaced = PlantPlanOverlay.GhostStates(MediumPlan.FromFrame(plans(false)), 4);
            Assert.AreEqual(1, spaced.Count, "4B-1: at most one ghost per sample after the first");
            Assert.AreEqual(1.0, spaced[0][0], 1e-6);
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
