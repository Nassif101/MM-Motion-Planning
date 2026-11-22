using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>
    /// Spec 4C Task 3: the journal's identity and orbit lines, a take per directing pass, the journal track (the directed
    /// view at a simulation time) and the render plan (pair check, frame times).
    /// </summary>
    public sealed class RenderTests
    {
        static JObject Hello() => new JObject { ["schema"] = 3, ["scenario"] = "v2_door_105_nav", ["seed"] = 1, ["mode"] = "realistic", ["epoch"] = 1 };

        static JObject Line(string text) => JObject.Parse(text);

        [Test]
        public void HeaderSurvivesTheLineBudget()
        {
            var w = new StringWriter();
            var j = new ViewJournal(w, () => 0.0, maxLinesPerSecond: 2, maxBytes: 10);
            j.WriteHeader(Hello());
            Assert.AreEqual(1, j.Lines);
            Assert.AreEqual("session", (string)Line(w.ToString().Split('\n')[0])["kind"]);
        }

        [Test]
        public void ReplayHeaderNamesTheLog()
        {
            var w = new StringWriter();
            new ViewJournal(w, () => 0.0).WriteHeader(Hello(), new JObject { ["log"] = "demo.mmlog", ["log_sha256"] = "ab" });
            var header = Line(w.ToString().Split('\n')[0]);
            Assert.AreEqual("demo.mmlog", (string)header["log"]);
            Assert.AreEqual("ab", (string)header["log_sha256"]);
            Assert.AreEqual(JournalIdentity.HelloSha256(Hello()), (string)header["hello_sha256"]);
        }

        [Test]
        public void LiveHeaderHasTheHelloHash()
        {
            var w = new StringWriter();
            new ViewJournal(w, () => 0.0).WriteHeader(Hello());
            var header = Line(w.ToString().Split('\n')[0]);
            Assert.AreEqual(64, ((string)header["hello_sha256"]).Length);
            Assert.IsNull(header["log_sha256"]);
            Assert.AreNotEqual(JournalIdentity.HelloSha256(Hello()),
                JournalIdentity.HelloSha256(new JObject { ["schema"] = 3, ["scenario"] = "v2_door_105_nav", ["seed"] = 2 }));
        }

        [Test]
        public void RestartStartsANewTake()
        {
            var files = new List<StringWriter>();
            var r = new JournalRecorder(n => { var w = new StringWriter(); files.Add(w); return w; }, () => 0.0);
            r.OnSession(Hello(), replay: false, new JObject { ["log"] = "demo.mmlog", ["log_sha256"] = "ab" });
            r.Write(1.0, "camera", new JObject { ["view"] = "Free" });
            r.NewTake();
            r.Write(0.5, "camera", new JObject { ["view"] = "Orbit" });
            Assert.AreEqual(2, files.Count);
            var second = files[1].ToString().Split(new[] { '\n' }, StringSplitOptions.RemoveEmptyEntries).Select(Line).ToList();
            Assert.AreEqual("ab", (string)second[0]["log_sha256"], "the same identity");
            Assert.AreEqual(2, second.Count);
            Assert.AreEqual(0.5, (double)second[1]["t"], 1e-12);
        }

        static JournalTrack Track(params string[] lines) =>
            JournalTrack.Parse(new[] { "{\"kind\":\"session\",\"wall\":0,\"scenario\":\"v2_door_105_nav\",\"hello_sha256\":\"h\"}" }.Concat(lines));

        [Test]
        public void OrbitLinesAreWrittenAndInterpolated()
        {
            var t = Track(
                "{\"t\":1.0,\"wall\":10.0,\"kind\":\"orbit\",\"yaw\":0,\"pitch\":20,\"distance\":4}",
                "{\"t\":1.1,\"wall\":10.1,\"kind\":\"orbit\",\"yaw\":90,\"pitch\":30,\"distance\":6}",
                "{\"t\":3.0,\"wall\":15.0,\"kind\":\"orbit\",\"yaw\":100,\"pitch\":30,\"distance\":6}");
            Assert.IsNull(t.ViewAt(0.5).Orbit, "nothing before the first line");
            var mid = t.ViewAt(1.05).Orbit.Value;
            Assert.AreEqual(45.0, mid.yaw, 1e-9);
            Assert.AreEqual(25.0, mid.pitch, 1e-9);
            Assert.AreEqual(5.0, mid.distance, 1e-9);
            // lines 5 s of wall time apart: the camera was still, then moved; held, not drifted
            Assert.AreEqual(90.0, t.ViewAt(2.0).Orbit.Value.yaw, 1e-9);
            Assert.AreEqual(100.0, t.ViewAt(3.5).Orbit.Value.yaw, 1e-9);
        }

        // Task 3 review I1: a slow viewer journals a move at long wall intervals; only a line marked still ends a move
        [Test]
        public void StillMarkerHoldsAndSlowMovesInterpolate()
        {
            var t = Track(
                "{\"t\":1.0,\"wall\":10.0,\"kind\":\"orbit\",\"yaw\":0,\"pitch\":20,\"distance\":4}",
                "{\"t\":1.5,\"wall\":10.5,\"kind\":\"orbit\",\"yaw\":50,\"pitch\":20,\"distance\":4,\"still\":true}",
                "{\"t\":4.0,\"wall\":13.0,\"kind\":\"orbit\",\"yaw\":60,\"pitch\":20,\"distance\":4}");
            Assert.AreEqual(25.0, t.ViewAt(1.25).Orbit.Value.yaw, 1e-9, "0.5 s of wall time between moving lines: interpolated");
            Assert.AreEqual(50.0, t.ViewAt(3.0).Orbit.Value.yaw, 1e-9, "held after the still line");
        }

        [Test]
        public void FreeCameraPoseIsInterpolated()
        {
            var t = Track(
                "{\"t\":1.0,\"wall\":10.0,\"kind\":\"free_camera\",\"position\":[0,1,0],\"rotation\":[0,0,0,1]}",
                "{\"t\":1.1,\"wall\":10.1,\"kind\":\"free_camera\",\"position\":[2,1,0],\"rotation\":[0,0,0,1]}");
            var v = t.ViewAt(1.05);
            Assert.AreEqual(1.0, v.FreePosition[0], 1e-6);
            Assert.AreEqual(1.0, v.FreeRotation[3], 1e-6);
        }

        [Test]
        public void ToggleStateIsTheLatestAtOrBeforeT()
        {
            var t = Track(
                "{\"t\":0.5,\"wall\":1,\"kind\":\"overlay_state\",\"preset\":\"Demo\",\"overlays\":{\"Route\":true,\"Lidar\":false},\"panels\":{\"Legend\":false,\"Minimap\":false}}",
                "{\"t\":1.0,\"wall\":2,\"kind\":\"camera\",\"view\":\"Chase\"}",
                "{\"t\":2.0,\"wall\":3,\"kind\":\"overlay\",\"toggle\":\"Lidar\",\"on\":true,\"preset\":\"Demo\",\"modified\":true}",
                "{\"t\":2.0,\"wall\":4,\"kind\":\"panel\",\"toggle\":\"Legend\",\"on\":true,\"preset\":\"Demo\",\"modified\":true}",
                "{\"t\":3.0,\"wall\":5,\"kind\":\"preset\",\"preset\":\"Diagnose\",\"overlays\":{\"Route\":true,\"Lidar\":true},\"panels\":{\"Legend\":true,\"Minimap\":true}}");
            var early = t.ViewAt(1.5);
            Assert.AreEqual("Chase", early.Camera);
            Assert.IsFalse(early.Toggles["Lidar"]);
            Assert.AreEqual("Demo", early.Preset);
            var later = t.ViewAt(2.0);
            Assert.IsTrue(later.Toggles["Lidar"]);
            Assert.IsTrue(later.Toggles["Legend"]);
            Assert.IsTrue(t.ViewAt(3.0).Toggles["Minimap"]);
            Assert.AreEqual("Diagnose", t.ViewAt(9.0).Preset);
        }

        [Test]
        public void SkippedKindsAreCounted()
        {
            var t = Track(
                "{\"t\":1,\"wall\":1,\"kind\":\"action\",\"action\":\"RunPause\"}",
                "{\"t\":1,\"wall\":1,\"kind\":\"minimap_cursor\",\"x\":1,\"y\":2}",
                "{\"t\":1,\"wall\":1,\"kind\":\"minimap_cursor\",\"x\":1,\"y\":3}",
                "{\"t\":2,\"wall\":2,\"kind\":\"goal_submitted\",\"target\":[1,2,0]}");
            Assert.AreEqual(1, t.SkippedKinds["action"]);
            Assert.AreEqual(2, t.SkippedKinds["minimap_cursor"]);
            Assert.AreEqual(1, t.SkippedKinds["goal_submitted"]);
        }

        static RunLog FixtureLog() => RunLogReader.Read(ViewerFixtures.Read("runlog_v3.mmlog"), "runlog_v3.mmlog");

        [Test]
        public void MismatchedJournalIsRefused()
        {
            var log = FixtureLog();
            var good = JournalTrack.Parse(new[] { $"{{\"kind\":\"session\",\"wall\":0,\"log\":\"x\",\"log_sha256\":\"{log.Sha256}\"}}" });
            Assert.IsTrue(RenderPlan.Check(log, good).ok);
            var other = JournalTrack.Parse(new[] { "{\"kind\":\"session\",\"wall\":0,\"log\":\"x\",\"log_sha256\":\"00\"}" });
            var refused = RenderPlan.Check(log, other);
            Assert.IsFalse(refused.ok);
            StringAssert.Contains("log", refused.refusal);
            var hello = (JObject)log.Frames[0].Header;
            var live = JournalTrack.Parse(new[] { $"{{\"kind\":\"session\",\"wall\":0,\"hello_sha256\":\"{JournalIdentity.HelloSha256(hello)}\"}}" });
            var accepted = RenderPlan.Check(log, live);
            Assert.IsTrue(accepted.ok);
            Assert.IsNotEmpty(accepted.warning);
            var none = JournalTrack.Parse(new[] { "{\"kind\":\"session\",\"wall\":0,\"scenario\":\"v2_door_105_nav\"}" });
            Assert.IsFalse(RenderPlan.Check(log, none).ok, "a journal without a hash");
        }

        [Test]
        public void FrameTimesFollowSpeed()
        {
            var (count, timeOf) = RenderPlan.Frames(startT: 0.0, endT: 10.0, from: 2.0, to: 4.0, speed: 0.5);
            Assert.AreEqual(3.0, timeOf(120), 1e-12);
            Assert.AreEqual(241, count, "2 s of simulation at 0.5x is 4 s of video, 240 intervals");
            Assert.AreEqual(4.0, timeOf(count - 1), 1e-12);
        }

        [Test]
        public void RenderRangeIsClampedToTheLog()
        {
            var (count, timeOf) = RenderPlan.Frames(startT: 0.02, endT: 0.40, from: -5.0, to: 99.0, speed: 1.0);
            Assert.AreEqual(0.02, timeOf(0), 1e-12);
            Assert.LessOrEqual(timeOf(count - 1), 0.40 + 1e-12);
            Assert.AreEqual(23, count, "0.38 s at 60 frames per second: 22 intervals");
            var (none, _) = RenderPlan.Frames(startT: 0.02, endT: 0.40, from: 1.0, to: 2.0, speed: 1.0);
            Assert.AreEqual(0, none, "a range after the log");
        }
    }
}
