using System;
using System.IO;
using MotionPlanningSim.Editor;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class PlantViewerSetupTests
    {
        static string Absolute(string assetPath) => Path.Combine(Application.dataPath, "..", assetPath);

        [TearDown]
        public void DropUnsavedScene() => EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        // Task 4 review I1: an open V2 with unsaved edits passed the old guard (its path equals the source), and the
        // build then opened the viewer scene in Single mode, discarding those edits
        [Test]
        public void BuildRefusesWhileTheSourceSceneHasUnsavedChanges()
        {
            var v2 = EditorSceneManager.OpenScene(PlantViewerSetup.SourceScenePath, OpenSceneMode.Single);
            new GameObject("unsaved edit");
            EditorSceneManager.MarkSceneDirty(v2);
            var viewer = Absolute(PlantViewerSetup.ScenePath);
            var before = File.GetLastWriteTimeUtc(viewer);
            var e = Assert.Throws<InvalidOperationException>(() => PlantViewerSetup.Build());
            StringAssert.Contains("unsaved", e.Message);
            Assert.IsTrue(v2.isDirty, "the edits are still open");
            Assert.AreEqual(before, File.GetLastWriteTimeUtc(viewer), "the viewer scene was not rebuilt");
        }
    }
}
