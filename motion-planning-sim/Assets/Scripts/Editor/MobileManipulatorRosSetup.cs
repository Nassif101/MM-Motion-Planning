using System;
using System.Collections.Generic;
using System.Linq;
using MotionPlanningSim.Control;
using MotionPlanningSim.Environment;
using MotionPlanningSim.ROS;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnitySensors.ROS.Publisher.Sensor;
using UnitySensors.Sensor.LiDAR;
using UnitySensors.Sensor.TF;

namespace MotionPlanningSim.Editor
{
    internal static class MobileManipulatorRosSetup
    {
        private const string BasePrefabPath =
            "Assets/Robots/MobileManipulator/Prefabs/MobileManipulator.prefab";

        private const string SensorizedPrefabPath =
            "Assets/Robots/MobileManipulator/Prefabs/MobileManipulatorSensors.prefab";

        private const string LidarPrefabPath =
            "Assets/Samples/UnitySensorsROS/3.0.0/Sample/Prefabs/" +
            "LiDAR/Livox/Mid-360_ros.prefab";

        private const string LidarFrameName = "livox_frame";
        private const string LidarTopicName = "/livox/lidar";
        private const float LidarMeasurementOffsetMetres = 0.047f;
        private const float LidarFrequencyHz = 10.0f;
        private const int LidarPointsPerScan = 20000;
        private const float LidarMinimumRangeMetres = 0.1f;
        private const float LidarMaximumRangeMetres = 70.0f;

        private const string RosConnectionPrefabPath =
            "Assets/Resources/ROSConnectionPrefab.prefab";

        private const string PhysicsMaterialFolder =
            "Assets/PhysicsMaterials";

        private const string WheelPhysicsMaterialPath =
            PhysicsMaterialFolder + "/MobileManipulatorWheel.physicMaterial";

        private const string FloorPhysicsMaterialPath =
            PhysicsMaterialFolder + "/CompactedSiteFloor.physicMaterial";

        private static readonly string[] JointNames =
        {
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        [MenuItem("Tools/Motion Planning/Configure Mobile Manipulator ROS")]
        private static void Configure()
        {
            var wheelMaterial = EnsurePhysicsMaterial(
                WheelPhysicsMaterialPath,
                "MobileManipulatorWheel",
                MobileManipulatorPhysicalContract.WheelStaticFriction,
                MobileManipulatorPhysicalContract.WheelDynamicFriction);
            var floorMaterial = EnsurePhysicsMaterial(
                FloorPhysicsMaterialPath,
                "CompactedSiteFloor",
                MobileManipulatorPhysicalContract.FloorStaticFriction,
                MobileManipulatorPhysicalContract.FloorDynamicFriction);

            BuildSensorizedPrefab(wheelMaterial);
            ConfigureActiveScene(wheelMaterial, floorMaterial);
            AssetDatabase.SaveAssets();
            Debug.Log(
                "Configured mobile manipulator base control, wheel/floor contact, " +
                "ROS state, clock, base TF, and lidar.");
        }

        private static void BuildSensorizedPrefab(PhysicsMaterial wheelMaterial)
        {
            var basePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(BasePrefabPath);
            var lidarPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(LidarPrefabPath);
            if (basePrefab == null || lidarPrefab == null)
            {
                throw new InvalidOperationException(
                    "The imported robot and UnitySensorsROS Mid-360 prefab are required.");
            }

            var previousScene = SceneManager.GetActiveScene();
            var temporaryScene = EditorSceneManager.NewScene(
                NewSceneSetup.EmptyScene,
                NewSceneMode.Additive);
            SceneManager.SetActiveScene(temporaryScene);

            GameObject robot = null;
            try
            {
                robot = (GameObject)PrefabUtility.InstantiatePrefab(
                    basePrefab,
                    temporaryScene);
                ConfigureRobot(robot, lidarPrefab, wheelMaterial);

                var saved = PrefabUtility.SaveAsPrefabAsset(
                    robot,
                    SensorizedPrefabPath);
                if (saved == null)
                {
                    throw new InvalidOperationException(
                        $"Failed to save {SensorizedPrefabPath}.");
                }
            }
            finally
            {
                if (robot != null)
                {
                    UnityEngine.Object.DestroyImmediate(robot);
                }

                SceneManager.SetActiveScene(previousScene);
                EditorSceneManager.CloseScene(temporaryScene, true);
            }
        }

        private static void ConfigureActiveScene(
            PhysicsMaterial wheelMaterial,
            PhysicsMaterial floorMaterial)
        {
            var scene = SceneManager.GetActiveScene();
            var robot = scene.GetRootGameObjects()
                .SingleOrDefault(root => root.name == "MobileManipulator");
            if (robot == null)
            {
                throw new InvalidOperationException(
                    "The active scene must contain one MobileManipulator root.");
            }

            var lidarPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(LidarPrefabPath);
            ConfigureRobot(robot, lidarPrefab, wheelMaterial);
            ConfigureFloorContact(scene, floorMaterial);
            EnsureSceneRosBootstrap(scene);
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
        }

        private static void ConfigureRobot(
            GameObject robot,
            GameObject lidarPrefab,
            PhysicsMaterial wheelMaterial)
        {
            var lidarFrame = FindUniqueLink(robot, LidarFrameName);
            var topMount = FindUniqueLink(robot, "top_sensor_mount_link");
            var baseLink = FindUniqueLink(robot, "base_link");

            var lidarRoot = FindOrCreateLidar(robot, lidarPrefab, lidarFrame);
            if (lidarRoot.transform.parent == topMount)
            {
                lidarRoot.transform.SetParent(lidarFrame, true);
            }
            else if (lidarRoot.transform.parent != lidarFrame)
            {
                throw new InvalidOperationException(
                    "The Livox Mid-360 must be attached to the top mount or livox_frame.");
            }

            RemoveTfLinks(lidarRoot);
            ConfigureLidarContract(lidarRoot, lidarFrame);

            var jointsByName = robot.GetComponentsInChildren<UrdfJoint>(true)
                .Where(joint => !string.IsNullOrWhiteSpace(joint.jointName))
                .ToDictionary(joint => joint.jointName, StringComparer.Ordinal);
            var articulations = new ArticulationBody[JointNames.Length];
            for (var index = 0; index < JointNames.Length; index++)
            {
                if (!jointsByName.TryGetValue(JointNames[index], out var urdfJoint))
                {
                    throw new InvalidOperationException(
                        $"Missing URDF joint {JointNames[index]}.");
                }

                articulations[index] = urdfJoint.GetComponent<ArticulationBody>();
            }

            GetOrAdd<MobileManipulatorJointStatePublisher>(robot)
                .Configure(articulations, (string[])JointNames.Clone());
            GetOrAdd<GroundTruthBaseTfPublisher>(robot)
                .Configure(baseLink);
            GetOrAdd<SkidSteerBaseController>(robot).Configure(
                baseLink.GetComponent<ArticulationBody>(),
                articulations[0],
                articulations[2],
                articulations[1],
                articulations[3]);
            ConfigureWheelContactMaterial(articulations, wheelMaterial);
        }

        private static PhysicsMaterial EnsurePhysicsMaterial(
            string assetPath,
            string materialName,
            float staticFriction,
            float dynamicFriction)
        {
            if (!AssetDatabase.IsValidFolder(PhysicsMaterialFolder))
            {
                AssetDatabase.CreateFolder("Assets", "PhysicsMaterials");
            }

            var material = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(assetPath);
            if (material == null)
            {
                material = new PhysicsMaterial(materialName);
                AssetDatabase.CreateAsset(material, assetPath);
            }

            material.name = materialName;
            material.staticFriction = staticFriction;
            material.dynamicFriction = dynamicFriction;
            material.bounciness = 0.0f;
            material.frictionCombine = PhysicsMaterialCombine.Minimum;
            material.bounceCombine = PhysicsMaterialCombine.Minimum;
            EditorUtility.SetDirty(material);
            return material;
        }

        private static void ConfigureWheelContactMaterial(
            IEnumerable<ArticulationBody> articulations,
            PhysicsMaterial wheelMaterial)
        {
            foreach (var wheel in articulations.Take(4))
            {
                var colliders = wheel.GetComponentsInChildren<Collider>(true);
                if (colliders.Length != 1)
                {
                    throw new InvalidOperationException(
                        $"Wheel {wheel.name} must contain exactly one collider.");
                }

                colliders[0].sharedMaterial = wheelMaterial;
            }
        }

        private static void ConfigureFloorContact(
            Scene scene,
            PhysicsMaterial floorMaterial)
        {
            var matches = scene.GetRootGameObjects()
                .SelectMany(root => root.GetComponentsInChildren<Transform>(true))
                .Where(candidate => candidate.name == "CompactedSiteGround")
                .ToArray();
            if (matches.Length != 1)
            {
                throw new InvalidOperationException(
                    $"Expected one CompactedSiteGround, found {matches.Length}.");
            }

            var collider = matches[0].GetComponent<BoxCollider>();
            if (collider == null || collider.isTrigger)
            {
                throw new InvalidOperationException(
                    "CompactedSiteGround requires a non-trigger BoxCollider.");
            }

            collider.sharedMaterial = floorMaterial;
        }

        private static GameObject FindOrCreateLidar(
            GameObject robot,
            GameObject lidarPrefab,
            Transform lidarFrame)
        {
            var sensors = robot.GetComponentsInChildren<RaycastLiDARSensor>(true);
            if (sensors.Length > 1)
            {
                throw new InvalidOperationException(
                    "Expected at most one RaycastLiDARSensor on the robot.");
            }

            if (sensors.Length == 1)
            {
                Transform lidarRoot = null;
                var candidate = sensors[0].transform;
                while (candidate != null && candidate != robot.transform)
                {
                    if (candidate.name == lidarPrefab.name ||
                        candidate.name == "VLP-16_ros")
                    {
                        lidarRoot = candidate;
                        break;
                    }

                    candidate = candidate.parent;
                }

                if (lidarRoot != null &&
                    lidarRoot.name == lidarPrefab.name &&
                    lidarRoot.parent == lidarFrame)
                {
                    return lidarRoot.gameObject;
                }

                UnityEngine.Object.DestroyImmediate(
                    lidarRoot != null ? lidarRoot.gameObject : sensors[0].gameObject,
                    true);
            }

            var created = (GameObject)PrefabUtility.InstantiatePrefab(
                lidarPrefab,
                lidarFrame);
            created.transform.localPosition = new Vector3(
                0.0f,
                -LidarMeasurementOffsetMetres,
                0.0f);
            created.transform.localRotation = Quaternion.identity;
            created.transform.localScale = Vector3.one;
            return created;
        }

        private static void RemoveTfLinks(GameObject lidarRoot)
        {
            foreach (var tfLink in lidarRoot.GetComponentsInChildren<TFLink>(true))
            {
                UnityEngine.Object.DestroyImmediate(tfLink, true);
            }
        }

        private static void ConfigureLidarContract(
            GameObject lidarRoot,
            Transform lidarFrame)
        {
            var sensor = lidarRoot.GetComponentInChildren<RaycastLiDARSensor>(true);
            var publisher =
                lidarRoot.GetComponentInChildren<LiDARPointCloud2MsgPublisher>(true);
            if (sensor == null || publisher == null)
            {
                throw new InvalidOperationException(
                    "The Livox Mid-360 sensor and point-cloud publisher are required.");
            }

            sensor.transform.SetPositionAndRotation(
                lidarFrame.position,
                lidarFrame.rotation);

            var serializedSensor = new SerializedObject(sensor);
            serializedSensor.FindProperty("_frequency").floatValue =
                LidarFrequencyHz;
            serializedSensor.FindProperty("_pointsNumPerScan").intValue =
                LidarPointsPerScan;
            serializedSensor.FindProperty("_minRange").floatValue =
                LidarMinimumRangeMetres;
            serializedSensor.FindProperty("_maxRange").floatValue =
                LidarMaximumRangeMetres;
            serializedSensor.ApplyModifiedPropertiesWithoutUndo();

            var serializedPublisher = new SerializedObject(publisher);
            serializedPublisher.FindProperty("_frequency").floatValue =
                LidarFrequencyHz;
            serializedPublisher.FindProperty("_topicName").stringValue =
                LidarTopicName;
            var frameId = serializedPublisher.FindProperty(
                "_serializer._header._frame_id");
            if (frameId == null)
            {
                throw new InvalidOperationException(
                    "Could not configure the Livox point-cloud frame ID.");
            }

            frameId.stringValue = LidarFrameName;
            serializedPublisher.ApplyModifiedPropertiesWithoutUndo();

            if (Vector3.Distance(sensor.transform.position, lidarFrame.position) >
                    1e-5f ||
                Quaternion.Angle(sensor.transform.rotation, lidarFrame.rotation) >
                    1e-3f)
            {
                throw new InvalidOperationException(
                    "The lidar measurement origin does not match livox_frame.");
            }
        }

        private static void EnsureSceneRosBootstrap(Scene scene)
        {
            var simulationRos = scene.GetRootGameObjects()
                .SingleOrDefault(root => root.name == "SimulationROS");
            if (simulationRos == null)
            {
                simulationRos = new GameObject("SimulationROS");
                SceneManager.MoveGameObjectToScene(simulationRos, scene);
            }

            GetOrAdd<SimulationClockPublisher>(simulationRos)
                .Configure(50.0f, 0.5f);

            var connections = scene.GetRootGameObjects()
                .SelectMany(root => root.GetComponentsInChildren<ROSConnection>(true))
                .ToArray();
            if (connections.Length == 0)
            {
                var connectionPrefab =
                    AssetDatabase.LoadAssetAtPath<GameObject>(RosConnectionPrefabPath);
                if (connectionPrefab == null)
                {
                    throw new InvalidOperationException(
                        "The project ROSConnection prefab is missing.");
                }

                var connection = (GameObject)PrefabUtility.InstantiatePrefab(
                    connectionPrefab,
                    simulationRos.transform);
                connection.name = "ROSConnection";
            }
            else if (connections.Length > 1)
            {
                throw new InvalidOperationException(
                    "The scene must have exactly one ROSConnection.");
            }
        }

        private static Transform FindUniqueLink(GameObject root, string objectName)
        {
            var matches = root.GetComponentsInChildren<UrdfLink>(true)
                .Where(link => link.name == objectName)
                .Select(link => link.transform)
                .ToArray();
            if (matches.Length != 1)
            {
                throw new InvalidOperationException(
                    $"Expected one {objectName}, found {matches.Length}.");
            }

            return matches[0];
        }

        private static T GetOrAdd<T>(GameObject target) where T : Component
        {
            return target.GetComponent<T>() ?? target.AddComponent<T>();
        }
    }
}
