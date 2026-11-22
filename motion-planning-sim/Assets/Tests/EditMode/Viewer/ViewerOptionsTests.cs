using System;
using MotionPlanningSim.Viewer.Core;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>Task 7: the player's command line (connection, journal, quality, frame times, scripted demo).</summary>
    public sealed class ViewerOptionsTests
    {
        [Test]
        public void DefaultsWithoutArguments()
        {
            var o = ViewerOptions.Parse(new[] { "PlantViewer" });
            Assert.AreEqual(("localhost", 7700), (o.Host, o.Port));
            Assert.IsNull(o.JournalDir);
            Assert.AreEqual("Balanced", o.Quality, "the level that met the 30 FPS target on the Mac (Task 7)");
            Assert.IsNull(o.FrameTimes);
            Assert.IsNull(o.Scenario);
            Assert.IsNull(o.Goal);
            Assert.AreEqual(1, o.Seed);
            Assert.AreEqual(1.0, o.Speed);
            Assert.IsFalse(o.Minimap);
            Assert.AreEqual(0, o.Fps, "0: no cap (vsync)");
        }

        [Test]
        public void ParsesEveryOption()
        {
            var o = ViewerOptions.Parse(new[]
            {
                "PlantViewer", "-host", "10.0.0.2", "-port", "7701", "-journal-dir", "/tmp/j", "-quality", "Balanced",
                "-frame-times", "/tmp/f.csv", "-scenario", "v2_open_yard_nav", "-seed", "3", "-goal", "18,0,0.5",
                "-speed", "2", "-minimap", "-fps", "30",
            });
            Assert.AreEqual(("10.0.0.2", 7701, "/tmp/j", "Balanced", "/tmp/f.csv"),
                (o.Host, o.Port, o.JournalDir, o.Quality, o.FrameTimes));
            Assert.AreEqual(("v2_open_yard_nav", 3), (o.Scenario, o.Seed));
            Assert.AreEqual((18.0, 0.0, 0.5), o.Goal);
            Assert.AreEqual(2.0, o.Speed);
            Assert.IsTrue(o.Minimap);
            Assert.AreEqual(30, o.Fps);
        }

        [Test]
        public void Fps30MeansTheVsyncLock()
        {
            Assert.IsTrue(ViewerOptions.Parse(new[] { "x", "-fps", "30" }).VsyncLock30);
            Assert.IsFalse(ViewerOptions.Parse(new[] { "x", "-fps", "45" }).VsyncLock30);
            Assert.IsFalse(ViewerOptions.Parse(new[] { "x" }).VsyncLock30);
        }

        [Test]
        public void PresetOptionParses()
        {
            Assert.AreEqual(Preset.Off, ViewerOptions.Parse(new[] { "x" }).StartPreset);
            Assert.AreEqual(Preset.Demo, ViewerOptions.Parse(new[] { "x", "-preset", "demo" }).StartPreset);
            Assert.AreEqual(Preset.Diagnose, ViewerOptions.Parse(new[] { "x", "-preset", "Diagnose" }).StartPreset);
            var e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-preset", "cinema" }));
            StringAssert.Contains("-preset", e.Message);
        }

        // 4B-1 Task 6: the pacing measurement runs each overlay alone (-overlays 4: only the ghosts)
        [Test]
        public void OverlaysOptionListsTheOverlaysOn()
        {
            Assert.IsNull(ViewerOptions.Parse(new[] { "x" }).Overlays);
            CollectionAssert.AreEqual(new[] { Overlay.Ghosts }, ViewerOptions.Parse(new[] { "x", "-overlays", "4" }).Overlays);
            CollectionAssert.AreEqual(new[] { Overlay.Route, Overlay.Lidar }, ViewerOptions.Parse(new[] { "x", "-overlays", "1,6" }).Overlays);
            CollectionAssert.IsEmpty(ViewerOptions.Parse(new[] { "x", "-overlays", "none" }).Overlays);
            var e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-overlays", "8" }));
            StringAssert.Contains("-overlays", e.Message);
        }

        // 4B-2: overlay 7 is B5's prediction
        [Test]
        public void OverlaysOptionAcceptsSeven()
        {
            CollectionAssert.AreEqual(new[] { Overlay.Prediction }, ViewerOptions.Parse(new[] { "x", "-overlays", "7" }).Overlays);
            var e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-overlays", "0" }));
            StringAssert.Contains("1-7", e.Message);
        }

        [Test]
        public void ABadValueNamesItsOption()
        {
            var e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-goal", "18,0" }));
            StringAssert.Contains("-goal", e.Message);
            e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-speed", "fast" }));
            StringAssert.Contains("-speed", e.Message);
            e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-fps", "0" }));
            StringAssert.Contains("-fps", e.Message);
            e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-goal", "1,2,3" }));
            StringAssert.Contains("-scenario", e.Message, "a scripted goal needs a scenario to reset to");
        }
    
        // Spec 4C: directing a run log
        [Test]
        public void ReplayOptionParses()
        {
            Assert.IsNull(ViewerOptions.Parse(new[] { "x" }).Replay);
            Assert.AreEqual("/runs/demo.mmlog", ViewerOptions.Parse(new[] { "x", "-replay", "/runs/demo.mmlog" }).Replay);
        }

    }
}
