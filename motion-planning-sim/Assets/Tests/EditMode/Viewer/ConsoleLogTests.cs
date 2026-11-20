using System.Collections.Generic;
using System.IO;
using System.Linq;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>4B-1 Task 3: the diagnostic console's store (design 7.3).</summary>
    public sealed class ConsoleLogTests
    {
        [Test]
        public void ConsoleCollapsesRepeats()
        {
            var log = new ConsoleLog();
            for (var k = 0; k < 3; ++k) log.Add(Severity.Error, "plant", 1.0 + k, 10.0 + k, "contact arm|wall");
            Assert.AreEqual(1, log.Entries.Count);
            Assert.AreEqual(3, log.Entries[0].Count);
            Assert.AreEqual(3.0, log.Entries[0].SimT);  // the latest occurrence's times
            log.Add(Severity.Error, "plant", 4.0, 13.0, "contact panel|door");
            Assert.AreEqual(2, log.Entries.Count);
        }

        [Test]
        public void ConsoleIsBoundedAndCountsDrops()
        {
            var log = new ConsoleLog(cap: 2000, perFrame: 10000);
            for (var k = 0; k < 2500; ++k) log.Add(Severity.Info, "goal", k, k, "m" + k);
            Assert.AreEqual(2000, log.Entries.Count);
            Assert.AreEqual(500, log.Dropped);
            Assert.AreEqual("m500", log.Entries[0].Text);
        }

        [Test]
        public void ConsoleKeepsItsFrameBudget()
        {
            var log = new ConsoleLog(perFrame: 200);
            log.BeginFrame();
            for (var k = 0; k < 200; ++k) Assert.IsTrue(log.Add(Severity.Info, "viewer", null, k, "m" + k));
            Assert.IsFalse(log.Add(Severity.Info, "viewer", null, 200, "m200"));
            Assert.AreEqual(1, log.Dropped);
            log.BeginFrame();
            Assert.IsTrue(log.Add(Severity.Info, "viewer", null, 201, "m201"));
        }

        [Test]
        public void ConsoleFiltersBySeverityAndSubsystemAndSearch()
        {
            var log = new ConsoleLog();
            log.Add(Severity.Info, "goal", 1, 1, "goal_accepted 4");
            log.Add(Severity.Warning, "medium", 2, 2, "fallback no_plan");
            log.Add(Severity.Error, "plant", 3, 3, "contact panel|door");
            Assert.AreEqual(2, log.Filter(Severity.Warning, null, null).Count());
            Assert.AreEqual(1, log.Filter(Severity.Info, new HashSet<string> { "goal" }, null).Count());
            Assert.AreEqual("contact panel|door", log.Filter(Severity.Info, null, "PANEL").Single().Text);
        }

        [Test]
        public void ConsoleExportsVisibleEntries()
        {
            var log = new ConsoleLog();
            log.Add(Severity.Warning, "medium", 2.5, 100.0, "fallback no_plan");
            log.Add(Severity.Warning, "medium", 2.6, 100.1, "fallback no_plan");
            var w = new StringWriter();
            ConsoleLog.Export(w, log.Filter(Severity.Info, null, null));
            StringAssert.Contains("warning", w.ToString());
            StringAssert.Contains("medium", w.ToString());
            StringAssert.Contains("fallback no_plan", w.ToString());
            StringAssert.Contains("x2", w.ToString());
        }

        // Task 3 review I1: a refused submission sends only its goal_ack, so the ack carries the warning
        [Test]
        public void RefusedGoalAcksAreWarnings()
        {
            foreach (var action in new[] { "rejected", "backpressure", "stale", "duplicate" })
                Assert.AreEqual((Severity.Warning, "goal"), Pick(ConsoleLog.FromEvent(JObject.Parse(
                    $"{{\"event\":\"goal_ack\",\"command\":\"submit_goal\",\"action\":\"{action}\",\"reason\":\"no_goal\"}}"))), action);
            foreach (var action in new[] { "accepted", "applied", "scheduled" })
                Assert.AreEqual((Severity.Info, "goal"), Pick(ConsoleLog.FromEvent(JObject.Parse(
                    $"{{\"event\":\"goal_ack\",\"command\":\"submit_goal\",\"action\":\"{action}\"}}"))), action);
        }

        static (Severity, string) Pick((Severity severity, string subsystem, string text) e) => (e.severity, e.subsystem);

        // Task 3 review I2: a grazing contact repeats with a different depth each time; it must still collapse
        [Test]
        public void RepeatedContactsWithDifferentDepthsCollapse()
        {
            var log = new ConsoleLog();
            foreach (var depth in new[] { 0.001, 0.0023, 0.0007 })
            {
                var (sev, sub, text) = ConsoleLog.FromEvent(JObject.Parse(
                    $"{{\"event\":\"contact\",\"t\":1,\"robot_geom\":\"panel\",\"other_geom\":\"site/door_frame\",\"depth\":{depth}}}"));
                log.Add(sev, sub, 1.0, 1.0, text);
            }
            Assert.AreEqual(1, log.Entries.Count);
            Assert.AreEqual(3, log.Entries[0].Count);
            StringAssert.Contains("panel", log.Entries[0].Text);
            StringAssert.Contains("site/door_frame", log.Entries[0].Text);
        }

        [Test]
        public void EventsMapToSeverityAndSubsystem()
        {
            (Severity, string) Map(string json)
            {
                var (sev, sub, _) = ConsoleLog.FromEvent(JObject.Parse(json));
                return (sev, sub);
            }
            Assert.AreEqual((Severity.Info, "goal"), Map("{\"event\":\"goal_accepted\",\"t\":1}"));
            Assert.AreEqual((Severity.Warning, "goal"), Map("{\"event\":\"goal_rejected\",\"t\":1,\"reason\":\"occupied\"}"));
            Assert.AreEqual((Severity.Warning, "goal"), Map("{\"event\":\"failed\",\"t\":1}"));
            Assert.AreEqual((Severity.Warning, "goal"), Map("{\"event\":\"timeout\",\"t\":1}"));
            Assert.AreEqual((Severity.Info, "route"), Map("{\"event\":\"route_accepted\",\"t\":1}"));
            Assert.AreEqual((Severity.Warning, "medium"), Map("{\"event\":\"fallback\",\"t\":1,\"reason\":\"no_plan\"}"));
            Assert.AreEqual((Severity.Error, "plant"), Map("{\"event\":\"contact\",\"t\":1}"));
            Assert.AreEqual((Severity.Warning, "stream"), Map("{\"event\":\"behind\",\"t\":1}"));
            Assert.AreEqual((Severity.Warning, "stream"), Map("{\"event\":\"dropped\",\"t\":1}"));
            Assert.AreEqual((Severity.Error, "plant"), Map("{\"event\":\"error\",\"reason\":\"x\"}"));
            Assert.AreEqual((Severity.Warning, "plant"), Map("{\"event\":\"run_stopped\",\"reason\":\"error\"}"));
            Assert.AreEqual((Severity.Info, "plant"), Map("{\"event\":\"run_stopped\",\"reason\":\"goals_done\"}"));
            Assert.AreEqual((Severity.Info, "plant"), Map("{\"event\":\"scenarios\"}"));
            StringAssert.Contains("occupied", ConsoleLog.FromEvent(JObject.Parse("{\"event\":\"goal_rejected\",\"reason\":\"occupied\"}")).text);
        }
    }
}
