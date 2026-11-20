using System;
using System.IO;
using System.Linq;
using Unity.Pipeline.Commands;
using UnityEditor;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    /// <summary>
    /// What the plant viewer's player needs beside its scene (spec 4A design 6.5): the routing map, copied from the
    /// navigation package into StreamingAssets (git-ignored; the build copies it again).
    /// </summary>
    public static class PlantViewerBuild
    {
        public const string MapFolder = "Assets/StreamingAssets/PlantViewer";
        public static readonly string[] MapFiles =
            { "construction_site_v2.yaml", "construction_site_v2.pgm", "construction_site_v2.metadata.json" };

        private static string Repository => Path.GetFullPath(Path.Combine(Application.dataPath, "..", ".."));

        [CliCommand("copy_plant_viewer_map",
            "Copy construction_site_v2.{yaml,pgm,metadata.json} into Assets/StreamingAssets/PlantViewer",
            MainThreadRequired = true)]
        public static object CopyMap()
        {
            var source = Path.Combine(Repository, "ros2_ws", "src", "mobile_manipulator_navigation", "maps");
            Directory.CreateDirectory(MapFolder);
            foreach (var name in MapFiles)
            {
                var from = Path.Combine(source, name);
                if (!File.Exists(from)) throw new InvalidOperationException("The routing map file is missing: " + from);
                File.Copy(from, Path.Combine(MapFolder, name), overwrite: true);
            }
            AssetDatabase.Refresh();
            return new { folder = MapFolder, files = MapFiles.ToArray() };
        }
    }
}
