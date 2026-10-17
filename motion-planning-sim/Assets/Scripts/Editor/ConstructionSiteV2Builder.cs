using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MotionPlanningSim.Environment;
using MotionPlanningSim.ROS;
using Unity.Pipeline.Commands;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MotionPlanningSim.Editor
{
    // Builds ConstructionSiteV2 from the site kit and SiteV2Layout. Only the generated roots are
    // rebuilt (Environment/Ground, Structural, NavigationObstacles, Disturbances, Experiment,
    // lighting and cameras); Environment/HandDressing, the robot and the ROS bootstrap are kept.
    public static class ConstructionSiteV2Builder
    {
        private const string SourceScenePath = "Assets/Scenes/ConstructionSiteV1.unity";
        private const string EnvironmentName = "Environment";
        private const string ObstaclesName = "NavigationObstacles";
        private const string HandDressingName = "HandDressing";
        private const string GroundObjectName = "CompactedSiteGround"; // RobotContactMonitor ignores it by name
        private const float GroundThickness = 0.2f;
        private const float GroundDecalTop = -AsphaltTop;
        private const float AsphaltTop = SiteV2Layout.AsphaltThickness;
        private const float SurroundSize = 200.0f;

        private static readonly string[] GeneratedRoots =
        {
            "Experiment",
            "ConstructionSiteGlobalVolume",
            "ConstructionSiteExposureVolume",
            "ConstructionSiteSun",
            "ConstructionSiteFillLights",
            "ConstructionSiteOverviewCamera",
            "RobotThirdPersonCamera",
        };

        // Edit-time pose of the main camera (the robot camera takes over in Play): an overview
        // from beyond the south-west corner, as in V1.
        private static readonly Vector3 OverviewRosPosition = new Vector3(-42.0f, -34.0f, 34.0f);
        private static readonly Vector3 OverviewRosTarget = new Vector3(0.0f, 0.0f, 0.0f);

        private static SiteDefinition Site => SiteRegistry.Get(SiteV2Layout.SiteName);

        [CliCommand("build_construction_site_v2",
            "Build ConstructionSiteV2 from the site kit and layout, save it, and run the V2 validator",
            MainThreadRequired = true)]
        public static object Build()
        {
            if (Application.isPlaying)
                throw new InvalidOperationException("Exit Play before building the site.");

            var bootstrapped = EnsureScene();
            SceneSwitch.Guard(Site.ScenePath);  // reopening the target itself is the builder's own rebuild
            var scene = EditorSceneManager.OpenScene(Site.ScenePath, OpenSceneMode.Single);
            foreach (var root in scene.GetRootGameObjects())
            {
                if (GeneratedRoots.Contains(root.name))
                    UnityEngine.Object.DestroyImmediate(root);
            }

            var robot = RequiredRoot(scene, "MobileManipulator");
            RequiredRoot(scene, "SimulationROS");

            var environment = scene.GetRootGameObjects().FirstOrDefault(r => r.name == EnvironmentName)
                              ?? new GameObject(EnvironmentName);
            environment.transform.SetSiblingIndex(0);
            // Everything under Environment is generated except the hand-tuned dressing.
            foreach (var child in environment.transform.Cast<Transform>().ToArray())
            {
                if (child.name != HandDressingName)
                    UnityEngine.Object.DestroyImmediate(child.gameObject);
            }

            var ground = ConstructionSiteTools.CreateRoot("Ground", environment.transform).transform;
            var structural = ConstructionSiteTools.CreateRoot("Structural", environment.transform).transform;
            var obstacles = ConstructionSiteTools.CreateRoot(ObstaclesName, environment.transform).transform;
            var disturbances = ConstructionSiteTools.CreateRoot(Nav2MapExporter.DisturbancesRootName, environment.transform);
            disturbances.SetActive(false);
            var handDressing = environment.transform.Find(HandDressingName)
                               ?? ConstructionSiteTools.CreateRoot(HandDressingName, environment.transform).transform;
            handDressing.SetAsLastSibling();

            BuildGround(ground);
            foreach (var zone in SiteV2Layout.Zones())
                BuildZone(zone, ground, structural, obstacles);
            BuildScanDressing(structural);
            BuildDisturbances(disturbances.transform);

            var experiment = ConstructionSiteTools.CreateRoot("Experiment");
            experiment.transform.SetSiblingIndex(1);
            BuildSpawnMarker(experiment.transform);
            PlaceRobotAtSpawn(robot);

            ConstructionSiteTools.CreateLightingAndCamera();
            var camera = scene.GetRootGameObjects().Single(r => r.name == "RobotThirdPersonCamera").transform;
            camera.position = RosToUnity(OverviewRosPosition);
            camera.LookAt(RosToUnity(OverviewRosTarget));

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            AssetDatabase.SaveAssets();

            var boxes = CollectBoxes(scene);
            var problems = SiteValidation.Problems(boxes, SiteV2Layout.Features(SiteKit.LoadSpec()), Site.Grid);
            return new
            {
                scene = Site.ScenePath,
                bootstrappedFromV1 = bootstrapped,
                colliderBoxes = boxes.Count,
                problems = problems.ToArray()
            };
        }

        [CliCommand("validate_construction_site_v2",
            "Validate ConstructionSiteV2: grid, intersections, support, doors, clear areas, overhead heights",
            MainThreadRequired = true)]
        public static object Validate()
        {
            var scene = SceneManager.GetActiveScene();
            if (scene.path != Site.ScenePath)
            {
                if (Application.isPlaying)
                    throw new InvalidOperationException($"Open {Site.ScenePath} (Edit mode) first.");
                scene = SceneSwitch.Open(Site.ScenePath);
            }
            var boxes = CollectBoxes(scene);
            var problems = SiteValidation.Problems(boxes, SiteV2Layout.Features(SiteKit.LoadSpec()), Site.Grid);
            return new { scene = scene.path, colliderBoxes = boxes.Count, problems = problems.ToArray() };
        }

        // The V2 scene starts as a copy of V1 so it carries the same robot (prefab instance with
        // its scene overrides), ROS bootstrap and connection objects; the generated V1 content is
        // replaced on the first build.
        private static bool EnsureScene()
        {
            if (File.Exists(Site.ScenePath))
                return false;
            if (!AssetDatabase.CopyAsset(SourceScenePath, Site.ScenePath))
                throw new InvalidOperationException($"Could not copy {SourceScenePath} to {Site.ScenePath}.");
            return true;
        }

        private static GameObject RequiredRoot(Scene scene, string name)
        {
            return scene.GetRootGameObjects().SingleOrDefault(r => r.name == name)
                   ?? throw new InvalidOperationException(
                       $"{scene.path} has no {name} root; delete the scene to bootstrap it again from {SourceScenePath}.");
        }

        private static void BuildGround(Transform ground)
        {
            // One flat floor collider for the whole site (grid), top at z 0, floor friction.
            var grid = Site.Grid;
            var sizeX = grid.MaxRosX - grid.MinRosX;
            var sizeY = grid.MaxRosY - grid.MinRosY;
            var centre = new Vector2((grid.MinRosX + grid.MaxRosX) * 0.5f, (grid.MinRosY + grid.MaxRosY) * 0.5f);
            var floor = new GameObject(GroundObjectName);
            floor.transform.SetParent(ground, false);
            floor.transform.position = RosMapPose.ToUnityPosition(centre.x, centre.y, 0.0f);
            var collider = floor.AddComponent<BoxCollider>();
            collider.center = new Vector3(0.0f, -GroundThickness * 0.5f, 0.0f);
            collider.size = new Vector3(sizeY, GroundThickness, sizeX);
            collider.sharedMaterial = SiteKit.GetPhysicsMaterial("floor_default");
            GameObjectUtility.SetStaticEditorFlags(floor, StaticEditorFlags.BatchingStatic);

            // Its visual sits just below the flush decals (asphalt, slab) so they never z-fight. The
            // visuals leave a hole over the Tier 1 trench, whose pit the Terrain zone draws.
            var trench = Rect.MinMaxRect(SiteV2Layout.TrenchMinX, SiteV2Layout.TrenchMinY, SiteV2Layout.TrenchMaxX,
                SiteV2Layout.TrenchMaxY);
            PlaceSurfaceWithHole("Surface", floor.transform, Rect.MinMaxRect(grid.MinRosX, grid.MinRosY, grid.MaxRosX,
                grid.MaxRosY), trench, GroundDecalTop);
            // Ground beyond the site, for views over the fence (visual only). Its corners sit on
            // the site's 2 m texture grid too, so its parts tile without seams.
            var half = SurroundSize * 0.5f;
            var west = grid.MinRosX - 2.0f * Mathf.Round((half + grid.MinRosX - centre.x) * 0.5f);
            var north = grid.MaxRosY + 2.0f * Mathf.Round((half - grid.MaxRosY + centre.y) * 0.5f);
            PlaceSurfaceWithHole("Surroundings", ground, Rect.MinMaxRect(west, north - SurroundSize,
                west + SurroundSize, north), trench, GroundDecalTop - 0.007f);
        }

        // A compacted-ground slab GroundThickness deep with its top at topZ, covering outer except
        // the hole: four boxes around it (south, north, west, east), all visual only.
        private static void PlaceSurfaceWithHole(string name, Transform parent, Rect outer, Rect hole, float topZ)
        {
            var parts = new[]
            {
                ("South", Rect.MinMaxRect(outer.xMin, outer.yMin, outer.xMax, hole.yMin)),
                ("North", Rect.MinMaxRect(outer.xMin, hole.yMax, outer.xMax, outer.yMax)),
                ("West", Rect.MinMaxRect(outer.xMin, hole.yMin, hole.xMin, hole.yMax)),
                ("East", Rect.MinMaxRect(hole.xMax, hole.yMin, outer.xMax, hole.yMax)),
            };
            foreach (var (side, r) in parts)
            {
                SiteKit.PlaceBox($"{name}{side}", "compacted_ground", parent, null,
                    new RosBox(new Vector3(r.center.x, r.center.y, topZ - GroundThickness * 0.5f),
                        new Vector3(r.width, r.height, GroundThickness)));
            }
        }

        private static void BuildZone(ZoneLayout zone, Transform ground, Transform structural, Transform obstacles)
        {
            Transform decals = null;
            var visuals = ConstructionSiteTools.CreateRoot(zone.Zone, structural).transform;
            var proxies = ConstructionSiteTools.CreateRoot(zone.Zone, obstacles).transform;
            foreach (var piece in zone.Pieces)
            {
                SiteKit.Place(piece.PieceId, piece.Name, visuals, proxies, piece.X, piece.Y, piece.Z, piece.Yaw,
                    piece.LengthX);
            }
            foreach (var box in zone.Boxes)
            {
                if (box.Collider)
                {
                    SiteKit.PlaceBox(box.Name, box.Material, visuals, proxies, box.Box);
                    continue;
                }
                decals = decals != null ? decals : ConstructionSiteTools.CreateRoot(zone.Zone, ground).transform;
                SiteKit.PlaceBox(box.Name, box.Material, decals, null, box.Box);
            }
            if (proxies.childCount == 0)
                UnityEngine.Object.DestroyImmediate(proxies.gameObject);
            if (visuals.childCount == 0)
                UnityEngine.Object.DestroyImmediate(visuals.gameObject);
        }

        // The Tier 2 disturbances: one root per group under Environment/Disturbances, each with a
        // Visuals and a Colliders child (the root itself is inactive in the saved scene). Nothing
        // here goes under NavigationObstacles, so the map and box exporters never see it.
        private static void BuildDisturbances(Transform disturbances)
        {
            foreach (var group in SiteV2Layout.Disturbances())
            {
                var root = ConstructionSiteTools.CreateRoot(group.Zone, disturbances).transform;
                var visuals = ConstructionSiteTools.CreateRoot("Visuals", root).transform;
                var colliders = ConstructionSiteTools.CreateRoot("Colliders", root).transform;
                foreach (var piece in group.Pieces)
                {
                    SiteKit.Place(piece.PieceId, piece.Name, visuals, colliders, piece.X, piece.Y, piece.Z, piece.Yaw,
                        piece.LengthX);
                }
                foreach (var box in group.Boxes)
                {
                    SiteKit.PlaceBox(box.Name, box.Material, visuals, colliders, box.Box,
                        string.IsNullOrEmpty(box.PhysicsMaterial) ? null : SiteKit.GetPhysicsMaterial(box.PhysicsMaterial));
                }
            }
        }

        // Optional Megascans dressing: each scan is placed only when it exists locally (the scans
        // are git-ignored); missing ones are skipped without a warning.
        private static void BuildScanDressing(Transform structural)
        {
            Transform root = null;
            foreach (var item in SiteV2Layout.Dressing())
            {
                if (AssetDatabase.LoadAssetAtPath<GameObject>(item.AssetPath) == null)
                    continue;
                root = root != null ? root : ConstructionSiteTools.CreateRoot("ScanDressing", structural).transform;
                ConstructionSiteTools.PlaceImportedAsset(item.AssetPath, item.Name,
                    RosMapPose.ToUnityPosition(item.X, item.Y, 0.0f),
                    RosMapPose.ToUnityRotation(item.YawDegrees * Mathf.Deg2Rad), root);
            }
        }

        // A painted bay outline at the default spawn (visual only).
        private static void BuildSpawnMarker(Transform experiment)
        {
            var marker = new GameObject("DefaultSpawn");
            marker.transform.SetParent(experiment, false);
            marker.transform.SetPositionAndRotation(
                RosMapPose.ToUnityPosition(SiteV2Layout.SpawnX, SiteV2Layout.SpawnY, 0.0f),
                RosMapPose.ToUnityRotation(SiteV2Layout.SpawnYaw));
            const float length = 1.6f, width = 1.2f, line = 0.06f, top = SiteV2Layout.MarkingTop;
            var cos = Math.Cos(SiteV2Layout.SpawnYaw);
            var sin = Math.Sin(SiteV2Layout.SpawnYaw);
            void Line(string name, float lx, float ly, float sx, float sy)
            {
                var x = SiteV2Layout.SpawnX + cos * lx - sin * ly;
                var y = SiteV2Layout.SpawnY + sin * lx + cos * ly;
                SiteKit.PlaceBox(name, "walkway_marking", marker.transform, null,
                    new RosBox(new Vector3((float)x, (float)y, (top - AsphaltTop) * 0.5f),
                        new Vector3(sx, sy, top + AsphaltTop), (float)SiteV2Layout.SpawnYaw));
            }
            Line("Front", length * 0.5f - line * 0.5f, 0.0f, line, width);
            Line("Back", -length * 0.5f + line * 0.5f, 0.0f, line, width);
            Line("Left", 0.0f, width * 0.5f - line * 0.5f, length - 2.0f * line, line);
            Line("Right", 0.0f, -width * 0.5f + line * 0.5f, length - 2.0f * line, line);
        }

        // Puts base_link at the default spawn pose; the runner teleports it later (scenario_place).
        private static void PlaceRobotAtSpawn(GameObject robot)
        {
            var baseLink = ConstructionSiteTools.FindChildRecursive(robot.transform, "base_link")
                           ?? throw new InvalidOperationException("MobileManipulator has no base_link.");
            var root = robot.transform;
            var localPosition = root.InverseTransformPoint(baseLink.position);
            var localRotation = Quaternion.Inverse(root.rotation) * baseLink.rotation;
            var rotation = RosMapPose.ToUnityRotation(SiteV2Layout.SpawnYaw) * Quaternion.Inverse(localRotation);
            var target = RosMapPose.ToUnityPosition(SiteV2Layout.SpawnX, SiteV2Layout.SpawnY, 0.0f);
            var offset = rotation * localPosition;
            root.SetPositionAndRotation(new Vector3(target.x - offset.x, root.position.y, target.z - offset.z), rotation);
            // The robot is a prefab instance: record the pose as an override so the save keeps it.
            if (PrefabUtility.IsPartOfPrefabInstance(robot))
                PrefabUtility.RecordPrefabInstancePropertyModifications(root);
        }

        // Collider boxes under Environment/NavigationObstacles in the ROS map frame, named by their
        // path below it (Tier 2 disturbances are excluded, as in the exports).
        internal static List<NamedBox> CollectBoxes(Scene scene)
        {
            var obstacles = Nav2MapExporter.NavigationObstaclesRoot(scene, Site);
            return Nav2MapExporter.NavigationColliders(obstacles)
                .Select(collider => new NamedBox { Name = RelativePath(obstacles, collider.transform), Box = ToRos(collider) })
                .ToList();
        }

        private static RosBox ToRos(Collider collider)
        {
            if (collider is BoxCollider box)
            {
                var t = box.transform;
                var scale = t.lossyScale;
                var size = Vector3.Scale(box.size, new Vector3(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z)));
                if (PlanningBoxGeometry.TryUnityOrientedBoxToRos(t.TransformPoint(box.center), size, t.rotation, out var oriented))
                    return oriented;
            }
            return PlanningBoxGeometry.UnityBoundsToRos(collider.bounds);
        }

        private static string RelativePath(Transform root, Transform transform)
        {
            var parts = new List<string>();
            for (var current = transform; current != null && current != root; current = current.parent)
                parts.Add(current.name);
            parts.Reverse();
            return string.Join("/", parts);
        }

        private static Vector3 RosToUnity(Vector3 ros) => RosMapPose.ToUnityPosition(ros.x, ros.y, ros.z);
    }
}
