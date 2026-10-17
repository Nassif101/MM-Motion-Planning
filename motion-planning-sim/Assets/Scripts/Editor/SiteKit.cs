using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using MotionPlanningSim.Environment;
using MotionPlanningSim.ROS;
using Unity.Pipeline.Commands;
using UnityEditor;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    // Places site kit pieces: a visual without colliders and one BoxCollider per spec proxy.
    // Pure geometry (spec parsing, proxy placement, the bounds comparison) lives in the Runtime
    // assembly (KitSpec, KitGeometry); this class only instantiates.
    public static class SiteKit
    {
        public const string KitRoot = "Assets/SiteKit";
        private const string MaterialFolder = KitRoot + "/Materials";
        private const string TextureFolder = KitRoot + "/Textures";
        private const string PhysicsMaterialFolder = KitRoot + "/PhysicsMaterials";
        private const string GeneratedMeshFolder = KitRoot + "/Meshes/Generated";
        private const float BoundsTolerance = 0.02f;

        // Material name, texture folder (null for a plain colour), tiling per metre of UV,
        // colour tint, smoothness, metallic.
        private readonly struct MaterialDef
        {
            public MaterialDef(string name, bool textured, float tiling, Color tint, float smoothness, float metallic,
                bool colourMap = true)
            {
                ColourMap = colourMap;
                Name = name;
                Textured = textured;
                Tiling = tiling;
                Tint = tint;
                Smoothness = smoothness;
                Metallic = metallic;
            }

            public readonly string Name;
            public readonly bool Textured;
            // False keeps only the normal map (surface detail) under a plain tint.
            public readonly bool ColourMap;
            public readonly float Tiling;
            public readonly Color Tint;
            public readonly float Smoothness;
            public readonly float Metallic;
        }

        // The ambientCG sets are about 2 m (ground-like) or 1 m (small-scale) per tile; the
        // Blender UVs are a 1 m cube projection and the primitives get metre UVs.
        private static readonly MaterialDef[] MaterialDefs =
        {
            // Tints pull the warm ambientCG sets toward the grey of a working site under the
            // warm V1 sun (asphalt and ground read orange otherwise).
            new MaterialDef("asphalt", true, 0.5f, new Color(0.9f, 0.95f, 1.0f), 0.15f, 0.0f),
            new MaterialDef("compacted_ground", true, 0.5f, new Color(0.72f, 0.8f, 0.95f), 0.10f, 0.0f),
            new MaterialDef("gravel", true, 0.5f, new Color(0.62f, 0.65f, 0.72f), 0.15f, 0.0f), // Gravel023 is near white in sun
            new MaterialDef("concrete", true, 0.5f, Color.white, 0.20f, 0.0f),
            // Bricks019 is a cream running bond of 2:1 units: a 1.8 m tile makes them 440 x 215 mm
            // concrete blocks, and the tint neutralises the cream to block grey.
            new MaterialDef("blockwork", true, 1.0f / 1.8f, new Color(0.8f, 0.85f, 1.0f), 0.20f, 0.0f),
            new MaterialDef("timber", true, 1.0f, Color.white, 0.25f, 0.0f),
            new MaterialDef("galvanized_steel", true, 1.0f, Color.white, 0.55f, 0.90f),
            new MaterialDef("painted_steel", true, 1.0f, Color.white, 0.45f, 0.10f),
            new MaterialDef("plastic", true, 1.0f, Color.white, 0.50f, 0.0f),
            // PaintedMetal007 is heavily rusted; hoarding and cabins use its relief under a plain
            // contractor green instead.
            new MaterialDef("cabin_paint", true, 1.0f, new Color(0.11f, 0.27f, 0.19f), 0.40f, 0.10f, colourMap: false),
            new MaterialDef("lane_marking", false, 1.0f, new Color(0.85f, 0.85f, 0.82f), 0.30f, 0.0f),
            new MaterialDef("walkway_marking", false, 1.0f, new Color(0.90f, 0.72f, 0.08f), 0.30f, 0.0f),
        };

        // Name, static friction, dynamic friction; friction combine minimum as the V1 floor.
        private static readonly (string name, float staticFriction, float dynamicFriction)[] PhysicsDefs =
        {
            ("floor_default", 0.05f, 0.02f),
            ("gravel_patch", 0.10f, 0.05f),
            ("mud_patch", 0.03f, 0.01f),
        };

        private static KitSpec cachedSpec;
        private static DateTime cachedSpecTime;
        private static readonly Dictionary<string, Material> MaterialCache = new Dictionary<string, Material>();

        public static string SpecPath => Path.GetFullPath(Path.Combine(
            Directory.GetCurrentDirectory(), "..", "tools", "site_kit", "kit_v2.json"));

        public static KitSpec LoadSpec()
        {
            var path = SpecPath;
            var time = File.GetLastWriteTimeUtc(path);
            if (cachedSpec == null || time != cachedSpecTime)
            {
                cachedSpec = KitSpec.Parse(File.ReadAllText(path));
                cachedSpecTime = time;
            }
            return cachedSpec;
        }

        [CliCommand("site_kit_assets", "Create or refresh the site kit materials and physics materials",
            MainThreadRequired = true)]
        public static object BuildAssets()
        {
            EnsureFolder(KitRoot, "Materials");
            EnsureFolder(KitRoot, "PhysicsMaterials");
            foreach (var def in MaterialDefs)
                EnsureMaterial(def);
            foreach (var def in PhysicsDefs)
                EnsurePhysicsMaterial(def.name, def.staticFriction, def.dynamicFriction);
            AssetDatabase.SaveAssets();
            MaterialCache.Clear();
            return new { materials = MaterialDefs.Length, physicsMaterials = PhysicsDefs.Length };
        }

        // Places one kit piece. The returned visual root sits under visualParent (no colliders,
        // static, shared instanced material); its proxies sit under proxyParent/<name>. Poses are
        // ROS map poses. lengthX stretches a parametric_x piece to that length.
        public static GameObject Place(string pieceId, string name, Transform visualParent, Transform proxyParent,
            double rosX, double rosY, double rosZ, double yaw, float lengthX = 0.0f)
        {
            var piece = LoadSpec().Get(pieceId);
            var scaleX = lengthX > 0.0f ? lengthX / piece.size[0] : 1.0f;
            if (lengthX > 0.0f && !piece.parametric_x)
                throw new ArgumentException($"{pieceId} is not parametric in x.", nameof(lengthX));

            var position = RosMapPose.ToUnityPosition(rosX, rosY, (float)rosZ);
            var rotation = RosMapPose.ToUnityRotation(yaw);

            var visual = new GameObject(name);
            GameObject proxyRoot = null;
            try
            {
                visual.transform.SetParent(visualParent, false);
                visual.transform.SetPositionAndRotation(position, rotation);
                BuildVisual(piece, visual.transform, scaleX);
                SetStatic(visual);

                proxyRoot = new GameObject(name);
                proxyRoot.transform.SetParent(proxyParent, false);
                proxyRoot.transform.SetPositionAndRotation(position, rotation);
                BuildProxies(piece, proxyRoot.transform, rosX, rosY, rosZ, yaw, lengthX);
                return visual;
            }
            catch
            {
                UnityEngine.Object.DestroyImmediate(visual);
                if (proxyRoot != null)
                    UnityEngine.Object.DestroyImmediate(proxyRoot);
                throw;
            }
        }

        // Places a primitive box with a kit material: the visual under visualParent and, when
        // proxyParent is given, one BoxCollider under proxyParent/<name> (named proxy_0, as kit
        // proxies are). Without proxyParent it is a visual-only decal. The box is a ROS map box.
        public static GameObject PlaceBox(string name, string material, Transform visualParent, Transform proxyParent,
            RosBox box, PhysicsMaterial physicsMaterial = null)
        {
            var bottom = box.Center.z - box.Size.z * 0.5f;
            var position = RosMapPose.ToUnityPosition(box.Center.x, box.Center.y, bottom);
            var rotation = RosMapPose.ToUnityRotation(box.Yaw);

            var visual = new GameObject(name);
            visual.transform.SetParent(visualParent, false);
            visual.transform.SetPositionAndRotation(position, rotation);
            var mesh = new GameObject("mesh");
            mesh.transform.SetParent(visual.transform, false);
            mesh.AddComponent<MeshFilter>().sharedMesh = BoxMesh(box.Size.x, box.Size.y, box.Size.z);
            mesh.AddComponent<MeshRenderer>().sharedMaterial = GetMaterial(material);
            SetStatic(visual);

            if (proxyParent != null)
            {
                var proxyRoot = new GameObject(name);
                proxyRoot.transform.SetParent(proxyParent, false);
                proxyRoot.transform.SetPositionAndRotation(position, rotation);
                var child = new GameObject("proxy_0");
                child.transform.SetParent(proxyRoot.transform, false);
                child.transform.SetPositionAndRotation(
                    RosMapPose.ToUnityPosition(box.Center.x, box.Center.y, box.Center.z), rotation);
                var collider = child.AddComponent<BoxCollider>();
                collider.size = new Vector3(box.Size.y, box.Size.z, box.Size.x);
                if (physicsMaterial != null)
                    collider.sharedMaterial = physicsMaterial;
            }
            return visual;
        }

        private static void BuildVisual(KitPiece piece, Transform visual, float scaleX)
        {
            var meshAsset = string.IsNullOrEmpty(piece.mesh)
                ? null
                : AssetDatabase.LoadAssetAtPath<GameObject>($"{KitRoot}/{piece.mesh}");
            if (meshAsset == null)
            {
                // Primitive pieces, and placeholders for an FBX that is not there yet.
                if (!string.IsNullOrEmpty(piece.mesh))
                    Debug.LogWarning($"Site kit: {KitRoot}/{piece.mesh} is missing; {piece.id} is placed as a box placeholder.");
                var size = piece.Size;
                var box = new GameObject("mesh");
                box.transform.SetParent(visual, false);
                box.AddComponent<MeshFilter>().sharedMesh = BoxMesh(size.x * scaleX, size.y, size.z);
                box.AddComponent<MeshRenderer>().sharedMaterial = GetMaterial(piece.material);
                return;
            }

            // The FBX root carries its own axis-conversion rotation, so the model sits under a
            // frame that orients the imported Unity axes (x length, y up, z width) to the piece frame.
            var frame = new GameObject("mesh");
            frame.transform.SetParent(visual, false);
            var instance = (GameObject)PrefabUtility.InstantiatePrefab(meshAsset);
            instance.name = "model";
            instance.transform.SetParent(frame.transform, false);
            foreach (var collider in instance.GetComponentsInChildren<Collider>(true))
                UnityEngine.Object.DestroyImmediate(collider);

            var mismatch = KitGeometry.BoundsMismatch(piece,
                KitGeometry.ImportedRosSize(LocalBoundsSize(frame.transform)), BoundsTolerance);
            if (mismatch != null)
                throw new InvalidOperationException(mismatch);

            AssignMaterials(piece, instance);
            // Imported mesh: piece x along Unity x. The piece frame has x along Unity z.
            frame.transform.localRotation = Quaternion.Euler(0.0f, -90.0f, 0.0f);
            frame.transform.localScale = new Vector3(scaleX, 1.0f, 1.0f);
        }

        private static Vector3 LocalBoundsSize(Transform root)
        {
            var bounds = new Bounds();
            var any = false;
            foreach (var filter in root.GetComponentsInChildren<MeshFilter>(true))
            {
                if (filter.sharedMesh == null)
                    continue;
                var toRoot = root.worldToLocalMatrix * filter.transform.localToWorldMatrix;
                var b = filter.sharedMesh.bounds;
                for (var i = 0; i < 8; i++)
                {
                    var corner = b.center + Vector3.Scale(b.extents,
                        new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                    var p = toRoot.MultiplyPoint3x4(corner);
                    if (!any)
                    {
                        bounds = new Bounds(p, Vector3.zero);
                        any = true;
                    }
                    else
                    {
                        bounds.Encapsulate(p);
                    }
                }
            }
            if (!any)
                throw new InvalidOperationException("Imported kit mesh has no geometry.");
            return bounds.size;
        }

        private static void AssignMaterials(KitPiece piece, GameObject instance)
        {
            foreach (var renderer in instance.GetComponentsInChildren<Renderer>(true))
            {
                var slots = renderer.sharedMaterials;
                for (var i = 0; i < slots.Length; i++)
                {
                    var slotName = slots[i] != null ? slots[i].name : piece.material;
                    slots[i] = GetMaterial(slotName);
                }
                renderer.sharedMaterials = slots;
            }
        }

        private static void BuildProxies(KitPiece piece, Transform proxyRoot, double rosX, double rosY, double rosZ,
            double yaw, float lengthX)
        {
            var physicsMaterial = string.IsNullOrEmpty(piece.physics_material)
                ? null
                : AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(
                    $"{PhysicsMaterialFolder}/{piece.physics_material}.physicMaterial");
            if (!string.IsNullOrEmpty(piece.physics_material) && physicsMaterial == null)
                throw new InvalidOperationException($"{piece.id}: physics material '{piece.physics_material}' is missing; run site_kit_assets.");

            var boxes = KitGeometry.ProxyBoxes(piece, rosX, rosY, rosZ, yaw, lengthX);
            for (var i = 0; i < boxes.Length; i++)
            {
                var box = boxes[i];
                var child = new GameObject($"proxy_{i}");
                child.transform.SetParent(proxyRoot, false);
                child.transform.SetPositionAndRotation(
                    RosMapPose.ToUnityPosition(box.Center.x, box.Center.y, box.Center.z),
                    RosMapPose.ToUnityRotation(box.Yaw));
                var collider = child.AddComponent<BoxCollider>();
                // Unity x = ROS -y (width), y = up, z = ROS x (length).
                collider.size = new Vector3(box.Size.y, box.Size.z, box.Size.x);
                if (physicsMaterial != null)
                    collider.sharedMaterial = physicsMaterial;
            }
        }

        private static void SetStatic(GameObject root)
        {
            const StaticEditorFlags flags = StaticEditorFlags.BatchingStatic | StaticEditorFlags.ContributeGI |
                                            StaticEditorFlags.OccluderStatic | StaticEditorFlags.OccludeeStatic |
                                            StaticEditorFlags.ReflectionProbeStatic;
            foreach (var t in root.GetComponentsInChildren<Transform>(true))
                GameObjectUtility.SetStaticEditorFlags(t.gameObject, flags);
        }

        // A shared box mesh asset per size (Meshes/Generated), so placed boxes do not embed their
        // own mesh in the scene and a rebuild references the same assets.
        internal static Mesh BoxMesh(float length, float width, float height)
        {
            var name = string.Format(CultureInfo.InvariantCulture, "box_{0:0.####}x{1:0.####}x{2:0.####}",
                length, width, height);
            var path = $"{GeneratedMeshFolder}/{name}.asset";
            var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(path);
            if (mesh != null)
                return mesh;
            if (!AssetDatabase.IsValidFolder(GeneratedMeshFolder))
                AssetDatabase.CreateFolder(KitRoot + "/Meshes", "Generated");
            mesh = CreateBoxMesh(length, width, height);
            mesh.name = name;
            AssetDatabase.CreateAsset(mesh, path);
            return mesh;
        }

        // A box mesh in Unity axes (x width, y up, z length), origin at the bottom centre, with
        // UVs in metres so a tiled material keeps a constant scale on every face.
        private static Mesh CreateBoxMesh(float length, float width, float height)
        {
            var hx = width * 0.5f;
            var hz = length * 0.5f;
            var vertices = new List<Vector3>();
            var uvs = new List<Vector2>();
            var normals = new List<Vector3>();
            var triangles = new List<int>();

            void Face(Vector3 origin, Vector3 u, Vector3 v, Vector3 normal)
            {
                var start = vertices.Count;
                vertices.Add(origin);
                vertices.Add(origin + u);
                vertices.Add(origin + u + v);
                vertices.Add(origin + v);
                uvs.Add(Vector2.zero);
                uvs.Add(new Vector2(u.magnitude, 0.0f));
                uvs.Add(new Vector2(u.magnitude, v.magnitude));
                uvs.Add(new Vector2(0.0f, v.magnitude));
                for (var k = 0; k < 4; k++)
                    normals.Add(normal);
                // Wind so the face points along its normal: Unity treats clockwise as the front,
                // so a front triangle (a, b, c) has Cross(b - a, c - a) along the normal.
                var alongNormal = Vector3.Dot(Vector3.Cross(u, v), normal) > 0.0f;
                int[] order = alongNormal ? new[] { 0, 1, 2, 0, 2, 3 } : new[] { 0, 2, 1, 0, 3, 2 };
                foreach (var o in order)
                    triangles.Add(start + o);
            }

            Face(new Vector3(-hx, 0, -hz), new Vector3(width, 0, 0), new Vector3(0, 0, length), Vector3.down);
            Face(new Vector3(-hx, height, -hz), new Vector3(width, 0, 0), new Vector3(0, 0, length), Vector3.up);
            Face(new Vector3(-hx, 0, hz), new Vector3(width, 0, 0), new Vector3(0, height, 0), Vector3.forward);
            Face(new Vector3(-hx, 0, -hz), new Vector3(width, 0, 0), new Vector3(0, height, 0), Vector3.back);
            Face(new Vector3(-hx, 0, -hz), new Vector3(0, 0, length), new Vector3(0, height, 0), Vector3.left);
            Face(new Vector3(hx, 0, -hz), new Vector3(0, 0, length), new Vector3(0, height, 0), Vector3.right);

            var mesh = new Mesh();
            mesh.SetVertices(vertices);
            mesh.SetUVs(0, uvs);
            mesh.SetNormals(normals);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateTangents();
            mesh.RecalculateBounds();
            return mesh;
        }

        internal static PhysicsMaterial GetPhysicsMaterial(string name)
        {
            return AssetDatabase.LoadAssetAtPath<PhysicsMaterial>($"{PhysicsMaterialFolder}/{name}.physicMaterial")
                   ?? throw new InvalidOperationException($"Physics material '{name}' is missing; run site_kit_assets.");
        }

        internal static Material GetMaterial(string name)
        {
            if (MaterialCache.TryGetValue(name, out var cached) && cached != null)
                return cached;
            var material = AssetDatabase.LoadAssetAtPath<Material>($"{MaterialFolder}/{name}.mat");
            if (material == null)
                throw new InvalidOperationException($"Kit material '{name}' is missing; run site_kit_assets.");
            MaterialCache[name] = material;
            return material;
        }

        private static void EnsureMaterial(MaterialDef def)
        {
            var path = $"{MaterialFolder}/{def.Name}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material == null)
            {
                material = new Material(Shader.Find("HDRP/Lit")) { name = def.Name };
                AssetDatabase.CreateAsset(material, path);
            }

            material.SetColor("_BaseColor", def.Tint);
            material.SetFloat("_Smoothness", def.Smoothness);
            material.SetFloat("_Metallic", def.Metallic);
            if (def.Textured)
            {
                var colour = ConfigureTexture($"{TextureFolder}/{def.Name}/color.jpg", false);
                var normal = ConfigureTexture($"{TextureFolder}/{def.Name}/normal.jpg", true);
                material.SetTexture("_BaseColorMap", def.ColourMap ? colour : null);
                material.SetTexture("_NormalMap", normal);
                material.SetFloat("_NormalScale", 1.0f);
                var scale = new Vector2(def.Tiling, def.Tiling);
                material.SetTextureScale("_BaseColorMap", scale);
                material.SetTextureScale("_NormalMap", scale);
            }
            material.enableInstancing = true;
            UnityEngine.Rendering.HighDefinition.HDMaterial.ValidateMaterial(material);
            EditorUtility.SetDirty(material);
        }

        private static Texture2D ConfigureTexture(string path, bool normalMap)
        {
            var importer = (TextureImporter)AssetImporter.GetAtPath(path);
            if (importer == null)
                throw new FileNotFoundException($"Kit texture missing: {path}");
            var type = normalMap ? TextureImporterType.NormalMap : TextureImporterType.Default;
            if (importer.textureType != type || importer.maxTextureSize != 1024)
            {
                importer.textureType = type;
                importer.maxTextureSize = 1024;
                importer.SaveAndReimport();
            }
            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        private static void EnsurePhysicsMaterial(string name, float staticFriction, float dynamicFriction)
        {
            var path = $"{PhysicsMaterialFolder}/{name}.physicMaterial";
            var material = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(path);
            if (material == null)
            {
                material = new PhysicsMaterial(name);
                AssetDatabase.CreateAsset(material, path);
            }
            material.staticFriction = staticFriction;
            material.dynamicFriction = dynamicFriction;
            material.bounciness = 0.0f;
            material.frictionCombine = PhysicsMaterialCombine.Minimum;
            material.bounceCombine = PhysicsMaterialCombine.Minimum;
            EditorUtility.SetDirty(material);
        }

        private static void EnsureFolder(string parent, string child)
        {
            if (!AssetDatabase.IsValidFolder($"{parent}/{child}"))
                AssetDatabase.CreateFolder(parent, child);
        }
    }
}
