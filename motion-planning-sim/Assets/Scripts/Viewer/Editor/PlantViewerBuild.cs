using System;
using System.IO;
using System.Linq;
using Unity.Pipeline.Commands;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    /// <summary>
    /// The plant viewer's macOS player (spec 4A design 8): the viewer scene only, with the routing map copied from
    /// the navigation package into StreamingAssets (git-ignored) before every build. The output folder
    /// <c>motion-planning-sim/Builds/</c> is git-ignored.
    /// </summary>
    public static class PlantViewerBuild
    {
        public const string Output = "Builds/PlantViewer-macOS/PlantViewer.app";

        /// <summary>
        /// Builds the player: <c>unity command build_plant_viewer_player</c> in the open Editor, or
        /// <c>-executeMethod MotionPlanningSim.Editor.PlantViewerBuild.Build</c> with it closed. Apple Silicon only.
        /// </summary>
        [CliCommand("build_plant_viewer_player", "Build the macOS plant viewer player (PlantViewer.unity only) into Builds/",
            MainThreadRequired = true)]
        public static object Build()
        {
            if (Application.isPlaying) throw new InvalidOperationException("Exit Play before building the player.");
            CopyMap();
            EditorUserBuildSettings.SetPlatformSettings("Standalone", "OSXUniversal", "Architecture", "ARM64");
            var options = new BuildPlayerOptions
            {
                scenes = new[] { PlantViewerSetup.ScenePath },
                locationPathName = Output,
                target = BuildTarget.StandaloneOSX,
                options = BuildOptions.None,
            };
            var report = BuildPipeline.BuildPlayer(options);
            var summary = report.summary;
            if (summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException($"The player build {summary.result}: {summary.totalErrors} errors (see the Editor log).");
            return new
            {
                output = Output, scenes = options.scenes, result = summary.result.ToString(),
                size_mb = summary.totalSize / 1e6, seconds = summary.totalTime.TotalSeconds, warnings = summary.totalWarnings,
            };
        }

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
