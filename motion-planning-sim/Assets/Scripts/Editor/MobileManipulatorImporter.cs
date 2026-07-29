using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MotionPlanningSim.Editor
{
    /// <summary>
    /// Imports the generated mobile-manipulator URDF and saves a validated prefab.
    /// The ROS package remains the source of truth; run the repository sync script first.
    /// </summary>
    internal static class MobileManipulatorImporter
    {
        private const string UrdfAssetPath =
            "Assets/Robots/MobileManipulator/urdf/mobile_manipulator.urdf";

        private const string MeshRoot =
            "Assets/Robots/MobileManipulator/urdf/meshes/visual";

        private const string PrefabFolder =
            "Assets/Robots/MobileManipulator/Prefabs";

        private const string PrefabPath =
            PrefabFolder + "/MobileManipulator.prefab";

        private const string RobotTag = "robot";

        [MenuItem("Tools/Motion Planning/Import Mobile Manipulator")]
        private static void Import()
        {
            EnsureRobotTag();
            EnsurePrefabFolder();
            AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);

            var urdfAsset = AssetDatabase.LoadAssetAtPath<UnityEngine.Object>(UrdfAssetPath);
            if (urdfAsset == null)
            {
                throw new FileNotFoundException(
                    "Sync the generated ROS description into Unity before importing.",
                    UrdfAssetPath);
            }

            var previousScene = SceneManager.GetActiveScene();
            var importScene = EditorSceneManager.NewScene(
                NewSceneSetup.EmptyScene,
                NewSceneMode.Additive);
            SceneManager.SetActiveScene(importScene);
            Selection.activeObject = null;

            GameObject robot = null;
            try
            {
                robot = RunUrdfImporter();
                RefreshGeneratedMeshAssets();
                RepairMeshReferences(robot);
                ConfigureRobotRoot(robot);
                ValidateRobot(robot, out var visualBounds);

                var prefab = PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
                if (prefab == null)
                {
                    throw new InvalidOperationException(
                        $"Failed to save mobile manipulator prefab at {PrefabPath}.");
                }

                AssetDatabase.SaveAssets();
                Selection.activeObject = prefab;
                Debug.Log(
                    $"Imported mobile manipulator prefab at {PrefabPath}; " +
                    $"visual bounds {visualBounds.size}.");
            }
            finally
            {
                if (robot != null)
                {
                    UnityEngine.Object.DestroyImmediate(robot);
                }

                SceneManager.SetActiveScene(previousScene);
                EditorSceneManager.CloseScene(importScene, true);
            }
        }

        private static GameObject RunUrdfImporter()
        {
            var absoluteUrdfPath = Path.Combine(
                Application.dataPath,
                "Robots/MobileManipulator/urdf/mobile_manipulator.urdf");
            var settings = ImportSettings.DefaultSettings();
            settings.chosenAxis = ImportSettings.axisType.yAxis;
            settings.convexMethod = ImportSettings.convexDecomposer.unity;

            // The installed importer has a Windows path-comparison bug that causes
            // unnecessary STL regeneration. The direct asset repair below makes the
            // result deterministic without modifying package-cache code.
            settings.OverwriteExistingPrefabs = false;

            var importer = UrdfRobotExtensions.Create(
                absoluteUrdfPath,
                settings,
                loadStatus: false,
                forceRuntimeMode: false);
            GameObject robot = null;
            try
            {
                while (importer.MoveNext())
                {
                    if (importer.Current != null)
                    {
                        robot = importer.Current;
                    }
                }
            }
            finally
            {
                importer.Dispose();
            }

            return robot ??
                   throw new InvalidOperationException(
                       "Unity URDF Importer returned no robot GameObject.");
        }

        private static void RefreshGeneratedMeshAssets()
        {
            AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);

            var meshAssetPaths = AssetDatabase.FindAssets("t:Mesh", new[] { MeshRoot })
                .Select(AssetDatabase.GUIDToAssetPath)
                .Where(path => path.EndsWith(".asset", StringComparison.OrdinalIgnoreCase))
                .OrderBy(path => path);
            foreach (var path in meshAssetPaths)
            {
                AssetDatabase.ImportAsset(
                    path,
                    ImportAssetOptions.ForceUpdate |
                    ImportAssetOptions.ForceSynchronousImport);
            }

            AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
        }

        private static void RepairMeshReferences(GameObject robot)
        {
            var repairedCount = 0;
            foreach (var filter in robot.GetComponentsInChildren<MeshFilter>(true))
            {
                if (!filter.gameObject.name.EndsWith("_0", StringComparison.Ordinal))
                    continue;

                var meshPath = $"{MeshRoot}/{filter.gameObject.name}.asset";
                var mesh = AssetDatabase.LoadAssetAtPath<UnityEngine.Mesh>(meshPath);
                if (mesh == null)
                {
                    throw new FileNotFoundException(
                        "Generated Unity mesh asset is missing.",
                        meshPath);
                }

                filter.sharedMesh = mesh;
                repairedCount++;
            }

            if (repairedCount != 12)
            {
                throw new InvalidOperationException(
                    $"Expected 12 visual mesh references, repaired {repairedCount}.");
            }
        }

        private static void ConfigureRobotRoot(GameObject robot)
        {
            robot.name = "MobileManipulator";
            robot.tag = RobotTag;
            robot.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

            var controller = robot.GetComponent<Controller>();
            if (controller != null)
            {
                UnityEngine.Object.DestroyImmediate(controller);
            }

            var fkRobot = robot.GetComponent<FKRobot>();
            if (fkRobot != null)
            {
                UnityEngine.Object.DestroyImmediate(fkRobot);
            }
        }

        private static void ValidateRobot(GameObject robot, out Bounds visualBounds)
        {
            var links = robot.GetComponentsInChildren<UrdfLink>(true);
            var joints = robot.GetComponentsInChildren<UrdfJoint>(true);
            var revolute = robot.GetComponentsInChildren<UrdfJointRevolute>(true);
            var continuous = robot.GetComponentsInChildren<UrdfJointContinuous>(true);
            var articulationBodies = robot.GetComponentsInChildren<ArticulationBody>(true);
            var renderers = robot.GetComponentsInChildren<MeshRenderer>(true);
            var colliders = robot.GetComponentsInChildren<Collider>(true);

            RequireCount("URDF links", links.Length, 17);
            RequireCount("URDF joints", joints.Length, 16);
            RequireCount("revolute joints", revolute.Length, 6);
            RequireCount("continuous joints", continuous.Length, 4);
            RequireCount("articulation bodies", articulationBodies.Length, 16);
            RequireCount("visual renderers", renderers.Length, 12);
            RequireCount("colliders", colliders.Length, 12);

            foreach (var renderer in renderers)
            {
                var filter = renderer.GetComponent<MeshFilter>();
                if (filter == null || filter.sharedMesh == null)
                {
                    throw new InvalidOperationException(
                        $"Visual {renderer.name} has no persistent mesh reference.");
                }
            }

            visualBounds = renderers[0].bounds;
            foreach (var renderer in renderers.Skip(1))
            {
                visualBounds.Encapsulate(renderer.bounds);
            }

            if (visualBounds.size.x < 0.70f ||
                visualBounds.size.y < 1.45f ||
                visualBounds.size.z < 0.80f)
            {
                throw new InvalidOperationException(
                    $"Imported visual bounds are unexpectedly small: {visualBounds.size}.");
            }
        }

        private static void RequireCount(string label, int actual, int expected)
        {
            if (actual != expected)
            {
                throw new InvalidOperationException(
                    $"Expected {expected} {label}, found {actual}.");
            }
        }

        private static void EnsurePrefabFolder()
        {
            const string robotAssetRoot = "Assets/Robots/MobileManipulator";
            if (!AssetDatabase.IsValidFolder(PrefabFolder))
            {
                AssetDatabase.CreateFolder(robotAssetRoot, "Prefabs");
            }
        }

        private static void EnsureRobotTag()
        {
            var tagManager = new SerializedObject(
                AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]);
            var tags = tagManager.FindProperty("tags");
            for (var index = 0; index < tags.arraySize; index++)
            {
                if (tags.GetArrayElementAtIndex(index).stringValue == RobotTag)
                    return;
            }

            tags.InsertArrayElementAtIndex(tags.arraySize);
            tags.GetArrayElementAtIndex(tags.arraySize - 1).stringValue = RobotTag;
            tagManager.ApplyModifiedProperties();
        }
    }
}
