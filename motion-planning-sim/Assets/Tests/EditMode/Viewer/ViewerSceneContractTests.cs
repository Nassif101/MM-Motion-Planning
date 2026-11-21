using System;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using MotionPlanningSim.Viewer;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>The viewer scene's contract (design 6.1): rendering only, no ROS simulator, sensors or physics.</summary>
    public sealed class ViewerSceneContractTests
    {
        public const string ScenePath = "Assets/Scenes/PlantViewer.unity";
        public const string PrefabPath = "Assets/Robots/MobileManipulator/Prefabs/MobileManipulatorView.prefab";

        static readonly string[] JointNames =
        {
            "front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint",
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        };

        static string Forbidden(Component c)
        {
            if (c == null) return "missing script";
            if (c is ArticulationBody || c is Collider || c is Rigidbody || c is Joint) return c.GetType().Name;
            var type = c.GetType();
            var assembly = type.Assembly.GetName().Name;
            if (assembly.StartsWith("Unity.Robotics", StringComparison.Ordinal) ||
                assembly.StartsWith("UnitySensors", StringComparison.Ordinal)) return type.FullName;
            if (assembly == "MotionPlanningSim.Runtime" && type.Name != "ThirdPersonRobotCamera") return type.FullName;
            return null;
        }

        [Test]
        public void ViewerSceneHasNoForbiddenComponents()
        {
            var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            var bad = scene.GetRootGameObjects()
                .SelectMany(r => r.GetComponentsInChildren<Component>(true).Select(c => (r, c)))
                .Select(p => (path: p.c == null ? p.r.name : p.c.transform.name, why: Forbidden(p.c)))
                .Where(p => p.why != null).Take(10).ToList();
            Assert.IsEmpty(bad, string.Join("; ", bad.Select(b => $"{b.path}: {b.why}")));
            Assert.AreEqual(1, scene.GetRootGameObjects().SelectMany(r => r.GetComponentsInChildren<PlantRobotView>(true)).Count());
        }

        [Test]
        public void BootstrapIsWired()
        {
            var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            var bootstrap = scene.GetRootGameObjects().SelectMany(r => r.GetComponentsInChildren<PlantViewerBootstrap>(true)).Single();
            var missing = bootstrap.MissingReferences().ToList();
            Assert.IsEmpty(missing, string.Join(", ", missing));
            var prediction = scene.GetRootGameObjects().SelectMany(r => r.GetComponentsInChildren<PlantPredictionOverlay>(true)).Single();
            Assert.IsNotNull(prediction.Robot, "4B-2: the prediction overlay's robot");
            Assert.IsNotNull(prediction.LineMaterial, "4B-2: the prediction path's material");
            Assert.IsNotNull(prediction.GhostMaterial, "4B-2: the prediction ghosts' material");
            var camera = scene.GetRootGameObjects().SelectMany(r => r.GetComponentsInChildren<MotionPlanningSim.Visualization.ThirdPersonRobotCamera>(true)).Single();
            Assert.IsNotNull(camera.Target, "orbit target");
            Assert.AreEqual("PayloadPanel", camera.PayloadTarget.name, "the V2 payload view is kept");
            Assert.IsNotNull(camera.GetComponent<FreeCamera>(), "the free camera shares the scene camera");
        }

        [Test]
        public void ViewPrefabHasTenViewJoints()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
            Assert.IsNotNull(prefab, PrefabPath);
            var names = prefab.GetComponentsInChildren<ViewJoint>(true).Select(j => j.JointName).OrderBy(n => n).ToArray();
            CollectionAssert.AreEqual(JointNames.OrderBy(n => n).ToArray(), names);
            Assert.IsNotNull(prefab.GetComponent<PlantRobotView>().PayloadAnchor, "tool0");
            Assert.IsNotNull(prefab.transform.Find("base_footprint/base_link"), "the link hierarchy is kept");
            Assert.IsTrue(prefab.GetComponentsInChildren<MeshRenderer>(true).Length > 0, "the meshes are kept");
        }

        [Test]
        public void SceneRecordsSiteChecksum()
        {
            var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            var site = scene.GetRootGameObjects().SelectMany(r => r.GetComponentsInChildren<PlantViewerSite>(true)).Single();
            var xml = File.ReadAllText(Path.Combine(Application.dataPath, "..", "..",
                "ros2_ws/src/mobile_manipulator_plant/models/site_v2.xml"));
            var expected = Regex.Match(xml, "<text name=\"site_checksum\" data=\"([^\"]*)\"").Groups[1].Value;
            Assert.IsNotEmpty(expected);
            Assert.AreEqual(expected, site.Checksum);
            Assert.IsTrue(site.HasSiteGeom("site/Environment/NavigationObstacles/Building/BeamEast/proxy_0"));
            Assert.AreEqual(Regex.Matches(xml, "<geom name=\"site/").Count, site.SiteGeomCount);
        }
    }
}
