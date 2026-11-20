using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using MotionPlanningSim.Viewer;
using Unity.Pipeline.Commands;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    /// <summary>
    /// Builds the plant viewer's scene and robot (spec 4A design 6.1). The scene is a copy of ConstructionSiteV2
    /// with its ROS simulator, experiment objects, collision proxies and every collider removed; its robot becomes
    /// MobileManipulatorView, the same meshes and links with physics, URDF, ROS and sensor components removed and
    /// each moving joint recorded as a <see cref="ViewJoint"/>. Rerun after V2's art changes; ConstructionSiteV2 is
    /// never modified.
    /// </summary>
    public static class PlantViewerSetup
    {
        public const string SourceScenePath = "Assets/Scenes/ConstructionSiteV2.unity";
        public const string ScenePath = "Assets/Scenes/PlantViewer.unity";
        public const string PrefabPath = "Assets/Robots/MobileManipulator/Prefabs/MobileManipulatorView.prefab";

        private static readonly string[] RemovedRoots = { "SimulationROS", "Experiment" };
        private static readonly string[] RemovedLinkChildren = { "Collisions", "Plugins" };

        private static string Repository => Path.GetFullPath(Path.Combine(Application.dataPath, "..", ".."));

        [CliCommand("build_plant_viewer",
            "Build Assets/Scenes/PlantViewer.unity and the physics-free MobileManipulatorView prefab from ConstructionSiteV2",
            MainThreadRequired = true)]
        public static object Build()
        {
            if (Application.isPlaying) throw new InvalidOperationException("Exit Play before building the viewer scene.");
            SceneSwitch.Guard(SourceScenePath);
            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(ScenePath) != null) AssetDatabase.DeleteAsset(ScenePath);
            if (!AssetDatabase.CopyAsset(SourceScenePath, ScenePath)) throw new InvalidOperationException("cannot copy " + SourceScenePath);
            var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);

            GameObject robot = null;
            foreach (var root in scene.GetRootGameObjects())
            {
                if (RemovedRoots.Contains(root.name)) UnityEngine.Object.DestroyImmediate(root);
                else if (root.GetComponent<UrdfRobot>() != null) robot = root;
            }
            if (robot == null) throw new InvalidOperationException(SourceScenePath + " has no URDF robot at its root");

            var environment = scene.GetRootGameObjects().Single(r => r.name == "Environment");
            var proxies = environment.transform.Find("NavigationObstacles");
            if (proxies != null) UnityEngine.Object.DestroyImmediate(proxies.gameObject);  // collider boxes only
            foreach (var c in environment.GetComponentsInChildren<Collider>(true)) UnityEngine.Object.DestroyImmediate(c);
            foreach (var r in environment.GetComponentsInChildren<Rigidbody>(true)) UnityEngine.Object.DestroyImmediate(r);

            var joints = MakeViewRobot(robot);
            var site = SiteRecord();
            var siteObject = new GameObject("PlantViewerSite").AddComponent<PlantViewerSite>();
            siteObject.Configure(site.checksum, site.geoms);
            var scenario = new GameObject("ScenarioObjects").AddComponent<PlantEnvironmentView>();
            scenario.Site = siteObject;

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            AssetDatabase.SaveAssets();
            return new { scene = ScenePath, prefab = PrefabPath, joints, site_geoms = site.geoms.Count, site_checksum = site.checksum };
        }

        // Turns the scene's robot instance (with its scene-only children such as tool0/PayloadPanel) into the view
        // prefab: records each moving joint, then keeps only Transform, MeshFilter, MeshRenderer and ViewJoint.
        private static string[] MakeViewRobot(GameObject robot)
        {
            PrefabUtility.UnpackPrefabInstance(robot, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
            robot.name = "MobileManipulatorView";
            robot.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            foreach (var t in robot.GetComponentsInChildren<Transform>(true).Where(t => RemovedLinkChildren.Contains(t.name)).ToList())
                if (t != null) UnityEngine.Object.DestroyImmediate(t.gameObject);

            var recorded = new List<string>();
            foreach (var body in robot.GetComponentsInChildren<ArticulationBody>(true))
            {
                if (body.jointType != ArticulationJointType.RevoluteJoint) continue;
                var joint = body.GetComponent<UrdfJoint>();
                if (joint == null || string.IsNullOrEmpty(joint.jointName)) continue;
                body.gameObject.AddComponent<ViewJoint>().Configure(joint.jointName, body.anchorRotation * Vector3.right,
                    body.transform.localRotation);
                recorded.Add(joint.jointName);
            }

            // MonoBehaviours first (URDF joints require their ArticulationBody), then the rest.
            for (var pass = 0; pass < 2; ++pass)
                foreach (var c in robot.GetComponentsInChildren<Component>(true))
                {
                    if (c == null || c is Transform || c is MeshFilter || c is MeshRenderer || c is ViewJoint) continue;
                    if (pass == 0 && !(c is MonoBehaviour)) continue;
                    UnityEngine.Object.DestroyImmediate(c);
                }

            var tool = robot.GetComponentsInChildren<Transform>(true).Single(t => t.name == "tool0");
            var profile = File.ReadAllText(Path.Combine(Repository,
                "ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml"));
            robot.AddComponent<PlantRobotView>().Configure(ProfileValue(profile, "wheel_radius_m"),
                ProfileValue(profile, "effective_track_m"), tool);
            PrefabUtility.SaveAsPrefabAssetAndConnect(robot, PrefabPath, InteractionMode.AutomatedAction);
            return recorded.ToArray();
        }

        private static float ProfileValue(string yaml, string key)
        {
            var m = Regex.Match(yaml, @"^\s*" + Regex.Escape(key) + @":\s*([-+0-9.eE]+)", RegexOptions.Multiline);
            if (!m.Success) throw new InvalidOperationException("profile.yaml has no " + key);
            return float.Parse(m.Groups[1].Value, System.Globalization.CultureInfo.InvariantCulture);
        }

        // The site model's checksum and geom names, read as view_run.py reads them.
        private static (string checksum, List<string> geoms) SiteRecord()
        {
            var xml = File.ReadAllText(Path.Combine(Repository, "ros2_ws/src/mobile_manipulator_plant/models/site_v2.xml"));
            var checksum = Regex.Match(xml, "<text name=\"site_checksum\" data=\"([^\"]*)\"");
            if (!checksum.Success) throw new InvalidOperationException("site_v2.xml has no site_checksum");
            var geoms = Regex.Matches(xml, "<geom name=\"(site/[^\"]*)\"").Cast<Match>().Select(g => g.Groups[1].Value).ToList();
            return (checksum.Groups[1].Value, geoms);
        }
    }
}
