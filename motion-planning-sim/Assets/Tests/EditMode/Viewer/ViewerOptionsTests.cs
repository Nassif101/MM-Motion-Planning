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
            Assert.IsNull(o.Quality);
            Assert.IsNull(o.FrameTimes);
            Assert.IsNull(o.Scenario);
            Assert.IsNull(o.Goal);
            Assert.AreEqual(1, o.Seed);
            Assert.AreEqual(1.0, o.Speed);
            Assert.IsFalse(o.Minimap);
        }

        [Test]
        public void ParsesEveryOption()
        {
            var o = ViewerOptions.Parse(new[]
            {
                "PlantViewer", "-host", "10.0.0.2", "-port", "7701", "-journal-dir", "/tmp/j", "-quality", "Balanced",
                "-frame-times", "/tmp/f.csv", "-scenario", "v2_open_yard_nav", "-seed", "3", "-goal", "18,0,0.5",
                "-speed", "2", "-minimap",
            });
            Assert.AreEqual(("10.0.0.2", 7701, "/tmp/j", "Balanced", "/tmp/f.csv"),
                (o.Host, o.Port, o.JournalDir, o.Quality, o.FrameTimes));
            Assert.AreEqual(("v2_open_yard_nav", 3), (o.Scenario, o.Seed));
            Assert.AreEqual((18.0, 0.0, 0.5), o.Goal);
            Assert.AreEqual(2.0, o.Speed);
            Assert.IsTrue(o.Minimap);
        }

        [Test]
        public void ABadValueNamesItsOption()
        {
            var e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-goal", "18,0" }));
            StringAssert.Contains("-goal", e.Message);
            e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-speed", "fast" }));
            StringAssert.Contains("-speed", e.Message);
            e = Assert.Throws<ArgumentException>(() => ViewerOptions.Parse(new[] { "x", "-goal", "1,2,3" }));
            StringAssert.Contains("-scenario", e.Message, "a scripted goal needs a scenario to reset to");
        }
    }
}
