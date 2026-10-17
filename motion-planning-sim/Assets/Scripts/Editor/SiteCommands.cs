using System;
using System.Linq;
using Unity.Pipeline.Commands;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MotionPlanningSim.Editor
{
    /// <summary>Lets the site tooling ask which scene is open and switch scenes in Edit mode.</summary>
    public static class SiteCommands
    {
        [CliCommand("site_active", "Report the active scene path and whether the Editor is in Play mode",
            MainThreadRequired = true)]
        public static object Active()
        {
            return new { scene = SceneManager.GetActiveScene().path, playing = Application.isPlaying };
        }

        [CliCommand("site_open", "Open a scene in Edit mode; no-op when it is already the active scene",
            MainThreadRequired = true)]
        public static object Open(string scene)
        {
            if (Application.isPlaying)
                throw new InvalidOperationException("Stop Play before opening a scene.");
            var active = SceneManager.GetActiveScene();
            if (active.IsValid() && active.path == scene)
                return new { scene, changed = false };
            EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
            return new { scene, changed = true };
        }

        [CliCommand("site_disturbances",
            "Switch the active site's Tier 2 disturbances (Environment/Disturbances) on or off; Play only",
            MainThreadRequired = true)]
        public static object Disturbances(bool enabled)
        {
            if (!Application.isPlaying)
                throw new InvalidOperationException("Enter Play first: the disturbances are switched at run time.");
            var environment = SceneManager.GetActiveScene().GetRootGameObjects()
                .FirstOrDefault(r => r.name == "Environment");
            var root = environment != null ? environment.transform.Find(Nav2MapExporter.DisturbancesRootName) : null;
            if (root == null)
                throw new InvalidOperationException("The active scene has no Environment/Disturbances.");
            root.gameObject.SetActive(enabled);
            return new { enabled, count = root.childCount };
        }
    }
}
