using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using MotionPlanningSim.Environment;
using Unity.Pipeline.Commands;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MotionPlanningSim.Editor
{
    public static class Nav2MapExporter
    {
        private const string ScenePath = "Assets/Scenes/ConstructionSiteV1.unity";
        private const string EnvironmentRootName = "Environment";
        private const string ObstaclesRootName = "NavigationObstacles";
        private const string MapBaseName = "construction_site";
        private const string ExporterVersion = "1";
        private const float MinimumUnityHeight = 0.02f;
        private const float MaximumUnityHeight = 3.2f;

        private static readonly Nav2GridSpec Grid = new Nav2GridSpec(
            -20.0f,
            -20.0f,
            20.0f,
            20.0f,
            0.05f);

        [Serializable]
        private sealed class MapMetadata
        {
            public string exporterVersion;
            public string sourceScene;
            public string sourceSceneGuid;
            public string mapFrame;
            public string coordinateMapping;
            public float resolutionMetres;
            public int widthCells;
            public int heightCells;
            public float originRosX;
            public float originRosY;
            public float minimumUnityHeight;
            public float maximumUnityHeight;
            public int sourceColliderCount;
            public int occupiedCellCount;
            public string pgmSha256;
        }

        private sealed class MapBuild
        {
            public Collider[] SourceColliders;
            public bool[] OccupiedCells;
            public byte[] PgmBytes;
            public string Yaml;
            public int OccupiedCellCount;
        }

        [MenuItem("Tools/Motion Planning/Export Construction Site Nav2 Map")]
        [CliCommand(
            "export_nav2_map",
            "Export ConstructionSiteV1 navigation colliders as a standard Nav2 PGM/YAML map",
            MainThreadRequired = true)]
        public static string ExportConstructionSiteMap()
        {
            var scene = OpenConstructionScene();
            var build = BuildMap(scene);
            ValidateKnownCells(build.OccupiedCells);
            var outputDirectory = GetMapOutputDirectory();
            Directory.CreateDirectory(outputDirectory);

            var pgmPath = Path.Combine(outputDirectory, MapBaseName + ".pgm");
            var yamlPath = Path.Combine(outputDirectory, MapBaseName + ".yaml");
            var metadataPath = Path.Combine(outputDirectory, MapBaseName + ".metadata.json");

            File.WriteAllBytes(pgmPath, build.PgmBytes);
            File.WriteAllText(yamlPath, build.Yaml, new UTF8Encoding(false));

            File.WriteAllText(
                metadataPath,
                BuildMetadata(build),
                new UTF8Encoding(false));

            return FormatSummary(build, outputDirectory);
        }

        [MenuItem("Tools/Motion Planning/Validate Construction Site Nav2 Map")]
        [CliCommand(
            "validate_nav2_map",
            "Validate the committed Nav2 map against current ConstructionSiteV1 colliders",
            MainThreadRequired = true)]
        public static string ValidateConstructionSiteMap()
        {
            var scene = OpenConstructionScene();
            var build = BuildMap(scene);
            var outputDirectory = GetMapOutputDirectory();
            var pgmPath = Path.Combine(outputDirectory, MapBaseName + ".pgm");
            var yamlPath = Path.Combine(outputDirectory, MapBaseName + ".yaml");
            var metadataPath = Path.Combine(outputDirectory, MapBaseName + ".metadata.json");

            if (!File.Exists(pgmPath) || !File.Exists(yamlPath) || !File.Exists(metadataPath))
                throw new InvalidOperationException("Nav2 map artifacts are missing; run export_nav2_map.");
            if (!File.ReadAllBytes(pgmPath).SequenceEqual(build.PgmBytes))
                throw new InvalidOperationException("Nav2 PGM is stale relative to scene navigation colliders.");
            if (!string.Equals(File.ReadAllText(yamlPath), build.Yaml, StringComparison.Ordinal))
                throw new InvalidOperationException("Nav2 map YAML is stale or has an invalid contract.");
            if (!string.Equals(
                    File.ReadAllText(metadataPath),
                    BuildMetadata(build),
                    StringComparison.Ordinal))
            {
                throw new InvalidOperationException("Nav2 map metadata is stale or has an invalid contract.");
            }

            ValidateKnownCells(build.OccupiedCells);
            return FormatSummary(build, outputDirectory);
        }

        private static Scene OpenConstructionScene()
        {
            var active = SceneManager.GetActiveScene();
            return active.IsValid() && active.path == ScenePath
                ? active
                : EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        }

        private static MapBuild BuildMap(Scene scene)
        {
            if (!scene.IsValid() || scene.path != ScenePath)
                throw new InvalidOperationException($"Expected source scene {ScenePath}.");

            var environment = scene.GetRootGameObjects()
                .SingleOrDefault(root => root.name == EnvironmentRootName);
            var obstacles = environment == null
                ? null
                : environment.transform.Find(ObstaclesRootName);
            if (obstacles == null)
            {
                throw new InvalidOperationException(
                    $"Required collider source {EnvironmentRootName}/{ObstaclesRootName} is missing.");
            }

            Physics.SyncTransforms();
            var colliders = obstacles.GetComponentsInChildren<Collider>(true)
                .Where(collider => collider.enabled &&
                                   collider.gameObject.activeInHierarchy &&
                                   !collider.isTrigger)
                .OrderBy(collider => GetHierarchyPath(collider.transform), StringComparer.Ordinal)
                .ToArray();
            if (colliders.Length == 0)
                throw new InvalidOperationException("NavigationObstacles contains no enabled colliders.");

            var occupied = new bool[checked(Grid.Width * Grid.Height)];
            foreach (var collider in colliders)
            {
                if (!Nav2MapGeometry.TryGetOverlappingCells(
                        collider.bounds,
                        Grid,
                        MinimumUnityHeight,
                        MaximumUnityHeight,
                        out var minimumX,
                        out var minimumY,
                        out var maximumX,
                        out var maximumY))
                {
                    continue;
                }

                // Collider bounds conservatively cover rotated geometry and never under-report obstacles.
                for (var cellY = minimumY; cellY <= maximumY; cellY++)
                {
                    for (var cellX = minimumX; cellX <= maximumX; cellX++)
                        occupied[Nav2MapGeometry.CellIndex(Grid, cellX, cellY)] = true;
                }
            }

            var imagePixels = Nav2MapEncoding.ToPgmImageRows(occupied, Grid.Width, Grid.Height);
            var header = Encoding.ASCII.GetBytes(
                $"P5\n# Generated from {ScenePath} NavigationObstacles\n{Grid.Width} {Grid.Height}\n255\n");
            var pgm = new byte[header.Length + imagePixels.Length];
            Buffer.BlockCopy(header, 0, pgm, 0, header.Length);
            Buffer.BlockCopy(imagePixels, 0, pgm, header.Length, imagePixels.Length);

            return new MapBuild
            {
                SourceColliders = colliders,
                OccupiedCells = occupied,
                PgmBytes = pgm,
                Yaml = BuildYaml(),
                OccupiedCellCount = occupied.Count(value => value)
            };
        }

        private static string BuildYaml()
        {
            return string.Format(
                CultureInfo.InvariantCulture,
                "image: {0}.pgm\n" +
                "mode: trinary\n" +
                "resolution: {1:F3}\n" +
                "origin: [{2:F3}, {3:F3}, 0.000]\n" +
                "negate: 0\n" +
                "occupied_thresh: 0.65\n" +
                "free_thresh: 0.25\n",
                MapBaseName,
                Grid.Resolution,
                Grid.MinRosX,
                Grid.MinRosY);
        }

        private static string BuildMetadata(MapBuild build)
        {
            var metadata = new MapMetadata
            {
                exporterVersion = ExporterVersion,
                sourceScene = ScenePath,
                sourceSceneGuid = AssetDatabase.AssetPathToGUID(ScenePath),
                mapFrame = "map",
                coordinateMapping = "ros_x=unity_z; ros_y=-unity_x; ros_z=unity_y",
                resolutionMetres = Grid.Resolution,
                widthCells = Grid.Width,
                heightCells = Grid.Height,
                originRosX = Grid.MinRosX,
                originRosY = Grid.MinRosY,
                minimumUnityHeight = MinimumUnityHeight,
                maximumUnityHeight = MaximumUnityHeight,
                sourceColliderCount = build.SourceColliders.Length,
                occupiedCellCount = build.OccupiedCellCount,
                pgmSha256 = Sha256(build.PgmBytes)
            };
            return JsonUtility.ToJson(metadata, true) + "\n";
        }

        private static void ValidateKnownCells(IReadOnlyList<bool> occupied)
        {
            if (!IsOccupied(occupied, Nav2MapGeometry.UnityWorldToRosMap(new Vector3(9.1f, 1, 11.1f))))
                throw new InvalidOperationException("Known east-lane fence is absent from the exported map.");
            if (IsOccupied(occupied, Nav2MapGeometry.UnityWorldToRosMap(new Vector3(10.5f, 0, 15.0f))))
                throw new InvalidOperationException("Start zone is unexpectedly occupied in the exported map.");
        }

        private static bool IsOccupied(IReadOnlyList<bool> occupied, Vector2 rosPosition)
        {
            var cellX = Mathf.FloorToInt((rosPosition.x - Grid.MinRosX) / Grid.Resolution);
            var cellY = Mathf.FloorToInt((rosPosition.y - Grid.MinRosY) / Grid.Resolution);
            if (cellX < 0 || cellX >= Grid.Width || cellY < 0 || cellY >= Grid.Height)
                throw new ArgumentOutOfRangeException(nameof(rosPosition), "Validation point is outside the map.");
            return occupied[Nav2MapGeometry.CellIndex(Grid, cellX, cellY)];
        }

        private static string GetMapOutputDirectory()
        {
            return Path.GetFullPath(Path.Combine(
                Application.dataPath,
                "..",
                "..",
                "ros2_ws",
                "src",
                "mobile_manipulator_navigation",
                "maps"));
        }

        private static string GetHierarchyPath(Transform transform)
        {
            var pieces = new Stack<string>();
            for (var current = transform; current != null; current = current.parent)
                pieces.Push(current.name);
            return string.Join("/", pieces);
        }

        private static string Sha256(byte[] bytes)
        {
            using var algorithm = SHA256.Create();
            return BitConverter.ToString(algorithm.ComputeHash(bytes)).Replace("-", "").ToLowerInvariant();
        }

        private static string FormatSummary(MapBuild build, string outputDirectory)
        {
            return $"Validated Nav2 map {Grid.Width}x{Grid.Height} @ {Grid.Resolution:F2} m/cell " +
                   $"from {build.SourceColliders.Length} colliders; " +
                   $"{build.OccupiedCellCount} occupied cells; output {outputDirectory}.";
        }
    }
}
