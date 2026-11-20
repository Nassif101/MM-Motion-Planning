using System.Linq;
using MotionPlanningSim.Viewer;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>4B-1 Task 5 review I1, I2, I3, I5, I6.</summary>
    public sealed class Task5ReviewTests
    {
        // I1: the State's contacts field is a count (session.cpp), not a list
        [Test]
        public void TelemetryShowsTheContactCount()
        {
            var lines = TelemetryText.B5(JObject.Parse("{\"status\":\"ok\",\"contacts\":2,\"dwell\":0,\"fallback\":\"\",\"gate\":[0.01,0.0,0]}"));
            var all = string.Join("\n", lines);
            StringAssert.Contains("contacts 2", all);
            StringAssert.Contains("status ok", all);  // strings unquoted
            StringAssert.DoesNotContain("\"", all);
            StringAssert.Contains("contacts —", string.Join("\n", TelemetryText.B5(null)), "not received");
        }

        // I2: the open panels never overlap (a click on telemetry passed through to the minimap and set a goal)
        [Test]
        public void PanelsDoNotOverlap()
        {
            foreach (var (w, h) in new[] { (1920, 1080), (1280, 720), (2560, 1440), (1064, 963), (900, 700) })
            {
                var minimap = PanelLayout.Minimap(w, h, 1000, 800);
                var console = PanelLayout.Console(w, h, minimap.xMin);
                var telemetry = PanelLayout.Telemetry(w, h, console.yMin, minimap.xMin);
                var legend = PanelLayout.Legend(w, h, 12, console.yMin, minimap.xMin);
                var rects = new[] { minimap, console, telemetry, legend };
                for (var i = 0; i < rects.Length; ++i)
                    for (var j = i + 1; j < rects.Length; ++j)
                        Assert.IsFalse(rects[i].Overlaps(rects[j]), $"{w}x{h}: panels {i} and {j} overlap");
            }
        }

        // I3: the console lays out only the rows in view
        [Test]
        public void ConsoleDrawsOnlyVisibleRows()
        {
            var (first, count) = ConsoleLog.VisibleRange(2000, 1600f, 16f, 300f);
            Assert.AreEqual(100, first);
            Assert.LessOrEqual(count, 21);
            Assert.AreEqual((0, 0), ConsoleLog.VisibleRange(0, 0f, 16f, 300f));
            var (f2, c2) = ConsoleLog.VisibleRange(10, 9999f, 16f, 300f);  // scrolled past the end
            Assert.AreEqual(10, f2 + c2);
        }

        // I5: a journal can start with the whole picture
        [Test]
        public void SnapshotNamesEveryToggleWithoutModifying()
        {
            var o = new OverlaySettings();
            o.Apply(Preset.Demo);
            var snap = o.Snapshot();
            Assert.AreEqual("Demo", (string)snap["preset"]);
            Assert.AreEqual(false, (bool)snap["modified"]);
            Assert.AreEqual(6, ((JObject)snap["overlays"]).Count);
            Assert.AreEqual(5, ((JObject)snap["panels"]).Count);
            Assert.IsFalse(o.Modified);
        }

        // I6: status-line warnings reach the console once each, when they appear
        [Test]
        public void NewWarningsAreReportedOnce()
        {
            CollectionAssert.AreEqual(new[] { "journal off: disk full" }, StatusMessage.Added(new string[0], new[] { "journal off: disk full" }).ToArray());
            CollectionAssert.IsEmpty(StatusMessage.Added(new[] { "journal off: disk full" }, new[] { "journal off: disk full" }).ToArray());
            CollectionAssert.AreEqual(new[] { "site differs" }, StatusMessage.Added(new[] { "journal off: disk full" }, new[] { "journal off: disk full", "site differs" }).ToArray());
        }
    }
}
