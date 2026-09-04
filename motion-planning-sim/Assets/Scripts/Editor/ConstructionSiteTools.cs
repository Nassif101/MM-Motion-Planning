using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MotionPlanningSim.Environment;
using MotionPlanningSim.Visualization;
using Unity.Pipeline.Commands;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

namespace MotionPlanningSim.Editor
{
    // Reproducible editor tooling; scene geometry remains authored and validated in Unity.
    public static class ConstructionSiteTools
    {
        private const string ScenePath = "Assets/Scenes/ConstructionSiteV1.unity";
        private const string MaterialRoot = "Assets/Materials/ConstructionSite";
        private const float PanelWidth = MobileManipulatorPhysicalContract.PayloadWidthMetres;
        private const float PanelHeight = MobileManipulatorPhysicalContract.PayloadHeightMetres;
        private const float PanelThickness = MobileManipulatorPhysicalContract.PayloadThicknessMetres;
        private const float InitialFootprintDepth = 1.2f;
        private const float NominalSideMargin = 0.3f;
        private const float NominalLaneWidth = 2.4f;
        private const float PrimaryTurnPocket = 2.9f;

        private static readonly string[] ModelRoots =
        {
            "Assets/Models/roadside_construction_mid",
            "Assets/Models/unfinished_building_mid"
        };

        [Serializable]
        private sealed class AssetAudit
        {
            public string generatedUtc;
            public List<AssetRecord> assets = new List<AssetRecord>();
        }

        [Serializable]
        private sealed class AssetRecord
        {
            public string path;
            public string name;
            public Vector3 sizeMetres;
            public Vector3 centre;
            public int meshCount;
            public int vertexCount;
            public int materialCount;
            public string[] materialNames;
        }

        [MenuItem("Tools/Motion Planning/Audit Construction Assets")]
        [CliCommand(
            "audit_construction_assets",
            "Audit imported construction FBX bounds and mesh/material counts",
            MainThreadRequired = true)]
        public static string AuditConstructionAssets()
        {
            var audit = new AssetAudit { generatedUtc = DateTime.UtcNow.ToString("O") };
            var paths = AssetDatabase.FindAssets("t:GameObject", ModelRoots)
                .Select(AssetDatabase.GUIDToAssetPath)
                .Where(path => path.EndsWith(".fbx", StringComparison.OrdinalIgnoreCase))
                .Distinct()
                .OrderBy(path => path, StringComparer.Ordinal)
                .ToArray();

            var previewScene = EditorSceneManager.NewPreviewScene();
            try
            {
                foreach (var path in paths)
                {
                    var asset = AssetDatabase.LoadAssetAtPath<GameObject>(path);
                    if (asset == null)
                        continue;

                    var instance = PrefabUtility.InstantiatePrefab(asset, previewScene) as GameObject;
                    if (instance == null)
                        continue;

                    try
                    {
                        var renderers = instance.GetComponentsInChildren<Renderer>(true);
                        var meshes = instance.GetComponentsInChildren<MeshFilter>(true)
                            .Select(filter => filter.sharedMesh)
                            .Where(mesh => mesh != null)
                            .Concat(instance.GetComponentsInChildren<SkinnedMeshRenderer>(true)
                                .Select(renderer => renderer.sharedMesh)
                                .Where(mesh => mesh != null))
                            .Distinct()
                            .ToArray();

                        var bounds = CalculateBounds(renderers, instance.transform.position);
                        var materials = renderers
                            .SelectMany(renderer => renderer.sharedMaterials)
                            .Where(material => material != null)
                            .Distinct()
                            .ToArray();

                        audit.assets.Add(new AssetRecord
                        {
                            path = path,
                            name = asset.name,
                            sizeMetres = bounds.size,
                            centre = bounds.center,
                            meshCount = meshes.Length,
                            vertexCount = meshes.Sum(mesh => mesh.vertexCount),
                            materialCount = materials.Length,
                            materialNames = materials.Select(material => material.name).ToArray()
                        });
                    }
                    finally
                    {
                        UnityEngine.Object.DestroyImmediate(instance);
                    }
                }
            }
            finally
            {
                EditorSceneManager.ClosePreviewScene(previewScene);
            }

            var outputDirectory = Path.Combine(Directory.GetCurrentDirectory(), "Temp");
            Directory.CreateDirectory(outputDirectory);
            var outputPath = Path.Combine(outputDirectory, "construction-asset-audit.json");
            File.WriteAllText(outputPath, JsonUtility.ToJson(audit, true));
            return $"Audited {audit.assets.Count} construction FBXs: {outputPath}";
        }

        [MenuItem("Tools/Motion Planning/Build Construction Site V1")]
        [CliCommand(
            "build_construction_site",
            "Build the repeatable ConstructionSiteV1 environment and initial panel payload",
            MainThreadRequired = true)]
        public static string BuildConstructionSite()
        {
            var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            RemoveGeneratedAndLegacyRoots(scene);

            var materials = CreateMaterials();
            var environment = CreateRoot("Environment");
            var ground = CreateRoot("Ground", environment.transform);
            var structural = CreateRoot("Structural", environment.transform);
            var navigation = CreateRoot("NavigationObstacles", environment.transform);
            var dressing = CreateRoot("VisualSetDressing", environment.transform);
            var experiment = CreateRoot("Experiment");

            CreateBox("CompactedSiteGround", Vector3.zero + Vector3.down * 0.1f,
                new Vector3(40.0f, 0.2f, 40.0f), materials.Ground, ground.transform);
            CreatePerimeter(navigation.transform, materials.DarkConcrete);
            CreatePrimaryTransportRoute(navigation.transform, materials.Fence, materials.Concrete);
            CreateManipulationChallenge(navigation.transform, materials.Fence, materials.Warning);
            CreateUnfinishedStructure(structural.transform, navigation.transform, materials, dressing.transform);
            CreateMaterialStorage(navigation.transform, materials, dressing.transform);
            CreateImportedDressing(dressing.transform);
            CreateExperimentMarkers(experiment.transform, materials);
            CreateOrReplacePanelPayload(materials.Panel);
            CreateLightingAndCamera();
            EnsureBuildSettings();

            var validation = ValidateConstructionSite(scene);
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();
            var mapValidation = Nav2MapExporter.ExportConstructionSiteMap();
            return $"Built {ScenePath}. {validation} {mapValidation}";
        }

        [MenuItem("Tools/Motion Planning/Validate Construction Site V1")]
        [CliCommand(
            "validate_construction_site",
            "Validate construction-site roots, payload, and designed clearances",
            MainThreadRequired = true)]
        public static string ValidateConstructionSiteCommand()
        {
            var scene = SceneManager.GetActiveScene().path == ScenePath
                ? SceneManager.GetActiveScene()
                : EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            return ValidateConstructionSite(scene);
        }

        private static void RemoveGeneratedAndLegacyRoots(Scene scene)
        {
            var replaceableNames = new HashSet<string>(StringComparer.Ordinal)
            {
                "Walls",
                "Plane",
                "Environment",
                "Experiment",
                "ConstructionSiteSun",
                "ConstructionSiteGlobalVolume",
                "ConstructionSiteExposureVolume",
                "ConstructionSiteFillLights",
                "ConstructionSiteOverviewCamera",
                "RobotThirdPersonCamera"
            };

            foreach (var root in scene.GetRootGameObjects())
            {
                if (replaceableNames.Contains(root.name))
                    UnityEngine.Object.DestroyImmediate(root);
            }
        }

        private sealed class SiteMaterials
        {
            public Material Ground;
            public Material Concrete;
            public Material DarkConcrete;
            public Material Fence;
            public Material Warning;
            public Material Timber;
            public Material Panel;
            public Material Start;
            public Material Delivery;
            public Material Route;
        }

        private static SiteMaterials CreateMaterials()
        {
            EnsureAssetFolder(MaterialRoot);
            return new SiteMaterials
            {
                Ground = CreateMaterial("CompactedGround", new Color(0.34f, 0.27f, 0.18f), 0.12f),
                Concrete = CreateMaterial("WeatheredConcrete", new Color(0.42f, 0.43f, 0.40f), 0.18f),
                DarkConcrete = CreateMaterial("DarkConcrete", new Color(0.20f, 0.21f, 0.20f), 0.14f),
                Fence = CreateMaterial("ConstructionFence", new Color(0.54f, 0.30f, 0.08f), 0.34f),
                Warning = CreateMaterial("SafetyOrange", new Color(1.0f, 0.23f, 0.015f), 0.28f),
                Timber = CreateMaterial("SiteTimber", new Color(0.33f, 0.17f, 0.07f), 0.20f),
                Panel = CreateMaterial("PayloadPanel", new Color(0.10f, 0.52f, 0.72f), 0.42f),
                Start = CreateMaterial("StartZone", new Color(0.08f, 0.42f, 0.18f), 0.25f),
                Delivery = CreateMaterial("DeliveryZone", new Color(0.80f, 0.55f, 0.04f), 0.25f),
                Route = CreateMaterial("ReferenceRoute", new Color(0.08f, 0.55f, 0.75f), 0.20f)
            };
        }

        private static void EnsureAssetFolder(string folderPath)
        {
            var pieces = folderPath.Split('/');
            var current = pieces[0];
            for (var index = 1; index < pieces.Length; index++)
            {
                var next = current + "/" + pieces[index];
                if (!AssetDatabase.IsValidFolder(next))
                    AssetDatabase.CreateFolder(current, pieces[index]);
                current = next;
            }
        }

        private static Material CreateMaterial(string name, Color colour, float smoothness)
        {
            var path = $"{MaterialRoot}/{name}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material == null)
            {
                var shader = Shader.Find("HDRP/Lit") ?? Shader.Find("Standard");
                material = new Material(shader) { name = name };
                AssetDatabase.CreateAsset(material, path);
            }

            material.color = colour;
            if (material.HasProperty("_BaseColor"))
                material.SetColor("_BaseColor", colour);
            if (material.HasProperty("_Smoothness"))
                material.SetFloat("_Smoothness", smoothness);
            EditorUtility.SetDirty(material);
            return material;
        }

        private static Material CreateScannedMaterial(string modelPath)
        {
            var folder = Path.GetDirectoryName(modelPath)?.Replace('\\', '/');
            var assetId = Path.GetFileNameWithoutExtension(modelPath);
            var materialPath = $"{MaterialRoot}/Scan_{assetId}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(materialPath);
            if (material == null)
            {
                material = new Material(Shader.Find("HDRP/Lit")) { name = $"Scan_{assetId}" };
                AssetDatabase.CreateAsset(material, materialPath);
            }

            var baseColour = FindScanTexture(folder, "basecolor");
            var normal = FindScanTexture(folder, "normal");
            if (baseColour != null && material.HasProperty("_BaseColorMap"))
                material.SetTexture("_BaseColorMap", baseColour);
            if (material.HasProperty("_BaseColor"))
                material.SetColor("_BaseColor", Color.white);
            if (normal != null && material.HasProperty("_NormalMap"))
            {
                ConfigureAsNormalMap(AssetDatabase.GetAssetPath(normal));
                normal = AssetDatabase.LoadAssetAtPath<Texture2D>(AssetDatabase.GetAssetPath(normal));
                material.SetTexture("_NormalMap", normal);
                if (material.HasProperty("_NormalScale"))
                    material.SetFloat("_NormalScale", 1.0f);
            }
            if (material.HasProperty("_Smoothness"))
                material.SetFloat("_Smoothness", 0.22f);

            HDMaterial.ValidateMaterial(material);
            EditorUtility.SetDirty(material);
            return material;
        }

        private static Texture2D FindScanTexture(string folder, string role)
        {
            if (string.IsNullOrEmpty(folder))
                return null;
            var path = AssetDatabase.FindAssets("t:Texture2D", new[] { folder })
                .Select(AssetDatabase.GUIDToAssetPath)
                .FirstOrDefault(candidate =>
                    candidate.IndexOf($"_{role}", StringComparison.OrdinalIgnoreCase) >= 0 &&
                    candidate.IndexOf("billboard", StringComparison.OrdinalIgnoreCase) < 0);
            return string.IsNullOrEmpty(path) ? null : AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        private static void ConfigureAsNormalMap(string texturePath)
        {
            if (!(AssetImporter.GetAtPath(texturePath) is TextureImporter importer))
                return;
            if (importer.textureType == TextureImporterType.NormalMap && !importer.sRGBTexture)
                return;
            importer.textureType = TextureImporterType.NormalMap;
            importer.sRGBTexture = false;
            importer.SaveAndReimport();
        }

        private static GameObject CreateRoot(string name, Transform parent = null)
        {
            var root = new GameObject(name);
            if (parent != null)
                root.transform.SetParent(parent, false);
            return root;
        }

        private static GameObject CreateBox(
            string name,
            Vector3 position,
            Vector3 size,
            Material material,
            Transform parent,
            Vector3? eulerAngles = null)
        {
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name;
            box.transform.SetParent(parent, false);
            box.transform.position = position;
            box.transform.localScale = size;
            if (eulerAngles.HasValue)
                box.transform.eulerAngles = eulerAngles.Value;
            box.GetComponent<Renderer>().sharedMaterial = material;
            GameObjectUtility.SetStaticEditorFlags(
                box,
                StaticEditorFlags.BatchingStatic |
                StaticEditorFlags.OccluderStatic |
                StaticEditorFlags.OccludeeStatic |
                StaticEditorFlags.ReflectionProbeStatic);
            return box;
        }

        private static GameObject CreateColliderProxy(string name, Vector3 position, Vector3 size, Transform parent)
        {
            var proxy = new GameObject(name, typeof(BoxCollider));
            proxy.transform.SetParent(parent, false);
            proxy.transform.position = position;
            proxy.GetComponent<BoxCollider>().size = size;
            GameObjectUtility.SetStaticEditorFlags(proxy, StaticEditorFlags.BatchingStatic);
            return proxy;
        }

        private static void CreatePerimeter(Transform parent, Material material)
        {
            var perimeter = CreateRoot("Perimeter", parent).transform;
            CreateBox("NorthBoundary", new Vector3(0, 1.5f, 19.8f), new Vector3(40, 3, 0.4f), material, perimeter);
            CreateBox("SouthBoundary", new Vector3(0, 1.5f, -19.8f), new Vector3(40, 3, 0.4f), material, perimeter);
            CreateBox("EastBoundary", new Vector3(19.8f, 1.5f, 0), new Vector3(0.4f, 3, 40), material, perimeter);
            CreateBox("WestBoundary", new Vector3(-19.8f, 1.5f, 0), new Vector3(0.4f, 3, 40), material, perimeter);
        }

        private static void CreatePrimaryTransportRoute(Transform parent, Material fence, Material concrete)
        {
            var route = CreateRoot("PrimaryTransportRoute", parent).transform;

            // East north-south lane: 2.4 m between the inner fence faces.
            CreateBox("EastLaneFenceWest", new Vector3(9.1f, 1.1f, 11.1f), new Vector3(0.4f, 2.2f, 11.8f), fence, route);
            CreateBox("EastLaneFenceEast", new Vector3(11.9f, 1.1f, 11.1f), new Vector3(0.4f, 2.2f, 11.8f), fence, route);

            // East-west lane and its two deliberately generous turning pockets.
            CreateBox("CrossLaneFenceNorth", new Vector3(1.9f, 1.1f, 4.9f), new Vector3(13.2f, 2.2f, 0.4f), fence, route);
            CreateBox("CrossLaneFenceSouth", new Vector3(2.0f, 1.1f, 2.1f), new Vector3(20.4f, 2.2f, 0.4f), fence, route);

            // West north-south lane toward the delivery zone.
            CreateBox("WestLaneFenceWest", new Vector3(-7.9f, 1.1f, -4.8f), new Vector3(0.4f, 2.2f, 12.6f), fence, route);
            CreateBox("WestLaneFenceEast", new Vector3(-5.1f, 1.1f, -4.8f), new Vector3(0.4f, 2.2f, 12.6f), fence, route);

            // Staggered low blocks make the lower lane visually construction-like while retaining 1.8 m clearance.
            CreateBox("ChicaneBlockWest", new Vector3(-7.4f, 0.45f, -2.5f), new Vector3(0.6f, 0.9f, 1.8f), concrete, route);
            CreateBox("ChicaneBlockEast", new Vector3(-5.6f, 0.45f, -7.0f), new Vector3(0.6f, 0.9f, 1.8f), concrete, route);
        }

        private static void CreateManipulationChallenge(Transform parent, Material fence, Material warning)
        {
            var challenge = CreateRoot("ManipulationChallenge", parent).transform;

            var controlledGate = CreateRoot("ControlledGate_1p35m", challenge).transform;
            CreateBox("GatePostLeft", new Vector3(0, 1.2f, -6.525f), new Vector3(1.6f, 2.4f, 0.25f), fence, controlledGate);
            CreateBox("GatePostRight", new Vector3(0, 1.2f, -8.125f), new Vector3(1.6f, 2.4f, 0.25f), fence, controlledGate);

            var manipulationGate = CreateRoot("ManipulationRequiredGate_1p05m", challenge).transform;
            CreateBox("GatePostLeft", new Vector3(7.0f, 1.2f, -7.225f), new Vector3(0.4f, 2.4f, 0.4f), warning, manipulationGate);
            CreateBox("GatePostRight", new Vector3(8.45f, 1.2f, -7.225f), new Vector3(0.4f, 2.4f, 0.4f), warning, manipulationGate);
            CreateBox("ApproachWallLeft", new Vector3(5.85f, 1.2f, -6.8f), new Vector3(0.3f, 2.4f, 2.4f), warning, manipulationGate, new Vector3(0, 65, 0));
            CreateBox("ApproachWallRight", new Vector3(9.6f, 1.2f, -7.65f), new Vector3(0.3f, 2.4f, 2.4f), warning, manipulationGate, new Vector3(0, 65, 0));
        }

        private static void CreateUnfinishedStructure(
            Transform structuralParent,
            Transform navigationParent,
            SiteMaterials materials,
            Transform dressingParent)
        {
            var structure = CreateRoot("UnfinishedBuilding", structuralParent).transform;
            CreateBox("Foundation", new Vector3(-14.0f, 0.15f, 8.0f), new Vector3(8.0f, 0.3f, 8.0f), materials.Concrete, structure);

            var collision = CreateRoot("BuildingCollision", navigationParent).transform;
            var columnLocations = new[]
            {
                new Vector3(-17, 1.5f, 5), new Vector3(-11, 1.5f, 5),
                new Vector3(-17, 1.5f, 11), new Vector3(-11, 1.5f, 11)
            };
            for (var index = 0; index < columnLocations.Length; index++)
                CreateColliderProxy($"ColumnProxy_{index + 1:00}", columnLocations[index], new Vector3(0.55f, 3.0f, 0.55f), collision);

            CreateBox("NorthBeam", new Vector3(-14, 3.0f, 11), new Vector3(6.6f, 0.45f, 0.55f), materials.Concrete, structure);
            CreateBox("SouthBeam", new Vector3(-14, 3.0f, 5), new Vector3(6.6f, 0.45f, 0.55f), materials.Concrete, structure);
            CreateBox("WestBeam", new Vector3(-17, 3.0f, 8), new Vector3(0.55f, 0.45f, 6.6f), materials.Concrete, structure);

            var columnAsset = "Assets/Models/unfinished_building_mid/ujjleixga/ujjleixga.fbx";
            foreach (var position in columnLocations)
                PlaceImportedAsset(columnAsset, "ScannedConcreteColumn", new Vector3(position.x, 0, position.z), Quaternion.identity, dressingParent);
        }

        private static void CreateMaterialStorage(Transform navigationParent, SiteMaterials materials, Transform dressingParent)
        {
            var storage = CreateRoot("MaterialStorage", navigationParent).transform;
            CreateBox("TimberStackProxy", new Vector3(14.8f, 0.65f, -12.0f), new Vector3(3.5f, 1.3f, 2.2f), materials.Timber, storage);
            CreateBox("ConcreteStackProxy", new Vector3(2.5f, 0.75f, -14.0f), new Vector3(3.0f, 1.5f, 2.4f), materials.Concrete, storage);
            PlaceImportedAsset(
                "Assets/Models/roadside_construction_mid/ujriaadga/ujriaadga.fbx",
                "ScannedMaterialAssembly",
                new Vector3(14.8f, 0, -12.0f),
                Quaternion.Euler(0, 25, 0),
                dressingParent);
        }

        private static void CreateImportedDressing(Transform parent)
        {
            var assets = new[]
            {
                ("Assets/Models/unfinished_building_mid/riuvL/riuvL.fbx", "ConcreteRubble", new Vector3(-15.0f, 0, -10.0f), 15.0f),
                ("Assets/Models/unfinished_building_mid/rjgmQ/rjgmQ.fbx", "BrickRubble", new Vector3(-12.0f, 0, -14.0f), -20.0f),
                ("Assets/Models/roadside_construction_mid/tlwtahufa/tlwtahufa.fbx", "ConcreteBarrier", new Vector3(15.0f, 0, 7.0f), 90.0f),
                ("Assets/Models/roadside_construction_mid/ubitfhtfa/ubitfhtfa.fbx", "LongConcreteBarrier", new Vector3(3.0f, 0, 17.0f), 0.0f),
                ("Assets/Models/roadside_construction_mid/vhvpabi/vhvpabi.fbx", "TrafficCone", new Vector3(8.2f, 0, 6.0f), 0.0f),
                ("Assets/Models/roadside_construction_mid/vhvpabi/vhvpabi.fbx", "TrafficCone", new Vector3(12.8f, 0, 6.0f), 20.0f),
                ("Assets/Models/roadside_construction_mid/vhvpabi/vhvpabi.fbx", "TrafficCone", new Vector3(-9.0f, 0, 0.5f), -15.0f),
                ("Assets/Models/roadside_construction_mid/tlhjacuva/tlhjacuva.fbx", "SmallDebris", new Vector3(5.0f, 0, -12.0f), 35.0f)
            };

            var repeatedNames = new Dictionary<string, int>(StringComparer.Ordinal);
            foreach (var asset in assets)
            {
                repeatedNames.TryGetValue(asset.Item2, out var count);
                repeatedNames[asset.Item2] = count + 1;
                PlaceImportedAsset(
                    asset.Item1,
                    $"{asset.Item2}_{count + 1:00}",
                    asset.Item3,
                    Quaternion.Euler(0, asset.Item4, 0),
                    parent);
            }
        }

        private static GameObject PlaceImportedAsset(
            string assetPath,
            string instanceName,
            Vector3 groundPosition,
            Quaternion rotation,
            Transform parent)
        {
            var asset = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
            if (asset == null)
            {
                Debug.LogWarning($"Construction dressing asset is unavailable: {assetPath}");
                return null;
            }

            var instance = PrefabUtility.InstantiatePrefab(asset, parent) as GameObject;
            if (instance == null)
                return null;

            instance.name = instanceName;
            instance.transform.position = groundPosition;
            instance.transform.rotation = rotation;
            var renderers = instance.GetComponentsInChildren<Renderer>(true);
            var scannedMaterial = CreateScannedMaterial(assetPath);
            foreach (var renderer in renderers)
                renderer.sharedMaterial = scannedMaterial;
            if (renderers.Length > 0)
            {
                var bounds = CalculateBounds(renderers, groundPosition);
                instance.transform.position += Vector3.up * (groundPosition.y - bounds.min.y);
            }

            foreach (var childTransform in instance.GetComponentsInChildren<Transform>(true))
            {
                GameObjectUtility.SetStaticEditorFlags(
                    childTransform.gameObject,
                    StaticEditorFlags.BatchingStatic |
                    StaticEditorFlags.OccludeeStatic |
                    StaticEditorFlags.ReflectionProbeStatic);
            }
            return instance;
        }

        private static void CreateExperimentMarkers(Transform parent, SiteMaterials materials)
        {
            CreateZone("StartZone", new Vector3(10.5f, 0.015f, 15.0f), 3.0f, materials.Start, parent);
            CreateZone("DeliveryZone", new Vector3(-6.5f, 0.015f, -14.5f), 4.0f, materials.Delivery, parent);
            var route = CreateRoot("ReferenceRoute", parent).transform;
            var waypoints = new[]
            {
                new Vector3(10.5f, 0.08f, 15.0f),
                new Vector3(10.5f, 0.08f, 6.7f),
                new Vector3(10.5f, 0.08f, 3.5f),
                new Vector3(-3.0f, 0.08f, 3.5f),
                new Vector3(-6.5f, 0.08f, 3.5f),
                new Vector3(-6.5f, 0.08f, -9.5f),
                new Vector3(-6.5f, 0.08f, -14.5f)
            };
            for (var index = 0; index < waypoints.Length; index++)
            {
                var marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                marker.name = $"Waypoint_{index + 1:00}";
                marker.transform.SetParent(route, false);
                marker.transform.position = waypoints[index];
                marker.transform.localScale = Vector3.one * 0.16f;
                marker.GetComponent<Renderer>().sharedMaterial = materials.Route;
                UnityEngine.Object.DestroyImmediate(marker.GetComponent<Collider>());
            }

            var footprint = CreateBox(
                "InitialPayloadFootprint_1p2m",
                new Vector3(10.5f, 0.025f, 15.0f),
                new Vector3(PanelWidth, 0.05f, InitialFootprintDepth),
                materials.Route,
                parent);
            UnityEngine.Object.DestroyImmediate(footprint.GetComponent<Collider>());
        }

        private static void CreateZone(string name, Vector3 position, float diameter, Material material, Transform parent)
        {
            var zone = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            zone.name = name;
            zone.transform.SetParent(parent, false);
            zone.transform.position = position;
            zone.transform.localScale = new Vector3(diameter, 0.01f, diameter);
            zone.GetComponent<Renderer>().sharedMaterial = material;
            UnityEngine.Object.DestroyImmediate(zone.GetComponent<Collider>());
        }

        private static void CreateOrReplacePanelPayload(Material panelMaterial)
        {
            var robot = SceneManager.GetActiveScene().GetRootGameObjects()
                .FirstOrDefault(root => root.name == "MobileManipulator");
            if (robot == null)
                throw new InvalidOperationException("ConstructionSiteV1 requires the MobileManipulator prefab instance.");

            var tool = FindChildRecursive(robot.transform, "tool0");
            if (tool == null)
                throw new InvalidOperationException("MobileManipulator is missing the required tool0 frame.");

            var existing = tool.Find("PayloadPanel");
            if (existing != null)
                UnityEngine.Object.DestroyImmediate(existing.gameObject);

            var panel = GameObject.CreatePrimitive(PrimitiveType.Cube);
            panel.name = "PayloadPanel";
            panel.transform.SetParent(tool, false);
            panel.transform.localPosition =
                MobileManipulatorPhysicalContract.PayloadCentreOfMassAtTool;
            panel.transform.localRotation = Quaternion.identity;
            // Broad face is tool-local XZ; its normal is the tool0 local Y axis.
            panel.transform.localScale = MobileManipulatorPhysicalContract.PayloadSizeUnity;
            panel.GetComponent<Renderer>().sharedMaterial = panelMaterial;

            ConfigureReferencePayloadInertia(tool);
        }

        private static void ConfigureReferencePayloadInertia(Transform tool)
        {
            var toolBody = tool.GetComponent<ArticulationBody>();
            if (toolBody == null)
                throw new InvalidOperationException(
                    "tool0 requires an ArticulationBody to represent the rigidly attached payload.");

            toolBody.mass = MobileManipulatorPhysicalContract.PayloadMassKg;
            toolBody.centerOfMass =
                MobileManipulatorPhysicalContract.PayloadCentreOfMassAtTool;
            toolBody.inertiaTensor =
                MobileManipulatorPhysicalContract.ReferencePayloadInertia;
            toolBody.inertiaTensorRotation = Quaternion.identity;
        }

        private static Transform FindChildRecursive(Transform root, string name)
        {
            if (root.name == name)
                return root;
            for (var index = 0; index < root.childCount; index++)
            {
                var match = FindChildRecursive(root.GetChild(index), name);
                if (match != null)
                    return match;
            }
            return null;
        }

        private static void CreateLightingAndCamera()
        {
            var volumeObject = new GameObject("ConstructionSiteGlobalVolume");
            var volume = volumeObject.AddComponent<Volume>();
            volume.isGlobal = true;
            volume.priority = 10.0f;
            volume.sharedProfile = AssetDatabase.LoadAssetAtPath<VolumeProfile>(
                "Assets/Settings/SkyandFogSettingsProfile.asset");

            var exposureObject = new GameObject("ConstructionSiteExposureVolume");
            var exposureVolume = exposureObject.AddComponent<Volume>();
            exposureVolume.isGlobal = true;
            exposureVolume.priority = 20.0f;
            exposureVolume.sharedProfile = CreateExposureProfile();

            var sunObject = new GameObject("ConstructionSiteSun");
            var sun = sunObject.AddComponent<Light>();
            sun.type = LightType.Directional;
            sun.color = new Color(1.0f, 0.91f, 0.78f);
            sun.intensity = 95000.0f;
            sun.shadows = LightShadows.Soft;
            sunObject.transform.rotation = Quaternion.Euler(42.0f, -32.0f, 0);
            RenderSettings.sun = sun;

            var fillLights = new GameObject("ConstructionSiteFillLights");
            CreateFillLight("NorthWestFill", new Vector3(-12, 12, 10), 5000.0f, fillLights.transform);
            CreateFillLight("SouthEastFill", new Vector3(12, 10, -10), 4000.0f, fillLights.transform);

            var robot = SceneManager.GetActiveScene().GetRootGameObjects()
                .FirstOrDefault(root => root.name == "MobileManipulator");
            var baseLink = robot == null
                ? null
                : FindChildRecursive(robot.transform, "base_link");
            if (baseLink == null)
                throw new InvalidOperationException(
                    "ConstructionSiteV1 requires MobileManipulator/base_link for the follow camera.");
            var tool = FindChildRecursive(robot.transform, "tool0");
            var payload = tool == null ? null : tool.Find("PayloadPanel");
            if (payload == null)
                throw new InvalidOperationException(
                    "ConstructionSiteV1 requires tool0/PayloadPanel for the payload camera view.");

            var cameraObject = new GameObject("RobotThirdPersonCamera");
            cameraObject.tag = "MainCamera";
            var camera = cameraObject.AddComponent<Camera>();
            camera.nearClipPlane = 0.1f;
            camera.farClipPlane = 150.0f;
            camera.fieldOfView = 52.0f;
            var hdCamera = cameraObject.AddComponent<HDAdditionalCameraData>();
            hdCamera.antialiasing = HDAdditionalCameraData.AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            cameraObject.AddComponent<ThirdPersonRobotCamera>().Configure(baseLink, payload);
        }

        private static VolumeProfile CreateExposureProfile()
        {
            var profilePath = $"{MaterialRoot}/ConstructionSiteExposure.asset";
            var profile = AssetDatabase.LoadAssetAtPath<VolumeProfile>(profilePath);
            if (profile == null)
            {
                profile = ScriptableObject.CreateInstance<VolumeProfile>();
                profile.name = "ConstructionSiteExposure";
                AssetDatabase.CreateAsset(profile, profilePath);
            }

            if (!profile.TryGet<Exposure>(out var exposure))
                exposure = profile.Add<Exposure>(true);
            exposure.active = true;
            exposure.mode.Override(ExposureMode.Fixed);
            exposure.fixedExposure.Override(12.5f);
            EditorUtility.SetDirty(profile);
            return profile;
        }

        private static void CreateFillLight(string name, Vector3 position, float intensity, Transform parent)
        {
            var lightObject = new GameObject(name);
            lightObject.transform.SetParent(parent, false);
            lightObject.transform.position = position;
            var light = lightObject.AddComponent<Light>();
            light.type = LightType.Point;
            light.color = new Color(1.0f, 0.78f, 0.58f);
            light.range = 30.0f;
            light.intensity = intensity;
            light.shadows = LightShadows.None;
        }

        private static void EnsureBuildSettings()
        {
            var scenes = EditorBuildSettings.scenes.ToList();
            var constructionIndex = scenes.FindIndex(scene => scene.path == ScenePath);
            if (constructionIndex >= 0)
                scenes.RemoveAt(constructionIndex);
            scenes.Insert(0, new EditorBuildSettingsScene(ScenePath, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }

        private static string ValidateConstructionSite(Scene scene)
        {
            if (!scene.IsValid() || scene.path != ScenePath)
                throw new InvalidOperationException($"Expected active scene {ScenePath}.");

            var requiredRoots = new[] { "SimulationROS", "MobileManipulator", "Environment", "Experiment" };
            var rootNames = new HashSet<string>(scene.GetRootGameObjects().Select(root => root.name));
            var missingRoots = requiredRoots.Where(name => !rootNames.Contains(name)).ToArray();
            if (missingRoots.Length > 0)
                throw new InvalidOperationException("Missing required scene roots: " + string.Join(", ", missingRoots));

            var robot = scene.GetRootGameObjects().First(root => root.name == "MobileManipulator");
            var tool = FindChildRecursive(robot.transform, "tool0");
            var panel = tool == null ? null : tool.Find("PayloadPanel");
            if (panel == null)
                throw new InvalidOperationException("PayloadPanel must be explicitly parented to tool0.");
            if ((panel.localScale - new Vector3(PanelWidth, PanelThickness, PanelHeight)).sqrMagnitude > 1e-6f)
                throw new InvalidOperationException("PayloadPanel dimensions do not match the initial 1.2 x 1.2 m contract.");
            if (Quaternion.Angle(panel.localRotation, Quaternion.identity) > 0.01f)
                throw new InvalidOperationException("PayloadPanel must remain in tool-local XZ with its normal along tool0 local Y.");

            var toolBody = tool.GetComponent<ArticulationBody>();
            if (toolBody == null ||
                Mathf.Abs(toolBody.mass - MobileManipulatorPhysicalContract.PayloadMassKg) > 1e-5f ||
                (toolBody.centerOfMass - MobileManipulatorPhysicalContract.PayloadCentreOfMassAtTool).sqrMagnitude > 1e-10f ||
                (toolBody.inertiaTensor - MobileManipulatorPhysicalContract.ReferencePayloadInertia).sqrMagnitude > 1e-10f ||
                Quaternion.Angle(toolBody.inertiaTensorRotation, Quaternion.identity) > 0.001f)
            {
                throw new InvalidOperationException(
                    "tool0 payload mass, centre of mass, or inertia does not match the reference-panel contract.");
            }

            var requiredLane = PayloadClearance.RequiredStraightPassage(PanelWidth, NominalSideMargin);
            var requiredPocket = PayloadClearance.RequiredTurningPocket(PanelWidth, InitialFootprintDepth, NominalSideMargin);
            if (NominalLaneWidth < requiredLane)
                throw new InvalidOperationException("Nominal lane is narrower than the payload clearance contract.");
            if (PrimaryTurnPocket < requiredPocket)
                throw new InvalidOperationException("Primary turning pocket is smaller than the payload swept envelope.");

            return $"Validated 1.2 x 1.2 m payload; nominal lane {NominalLaneWidth:F2} m " +
                   $"(required {requiredLane:F2} m); turn pocket {PrimaryTurnPocket:F2} m " +
                   $"(required {requiredPocket:F2} m).";
        }

        private static Bounds CalculateBounds(IReadOnlyList<Renderer> renderers, Vector3 fallbackCentre)
        {
            if (renderers.Count == 0)
                return new Bounds(fallbackCentre, Vector3.zero);

            var bounds = renderers[0].bounds;
            for (var index = 1; index < renderers.Count; index++)
                bounds.Encapsulate(renderers[index].bounds);
            return bounds;
        }
    }
}
