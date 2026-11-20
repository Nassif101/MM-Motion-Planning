using System.IO;
using System.Linq;
using System.Text;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class ViewJournalTests
    {
        static JObject[] Lines(StringWriter w) =>
            w.ToString().Split('\n').Where(l => l.Length > 0).Select(JObject.Parse).ToArray();

        [Test]
        public void JournalHeaderCarriesSessionIdentity()
        {
            var hello = JObject.Parse(
                "{\"schema\":3,\"scenario\":\"v2_open_yard_nav\",\"seed\":2,\"mode\":\"realistic\"," +
                "\"site_checksum\":\"abc\",\"routing\":{\"map_yaml_sha256\":\"m\"},\"provenance\":{\"git_commit\":\"c\"}," +
                "\"epoch\":5,\"boxes\":[]}");
            var w = new StringWriter();
            using (var journal = new ViewJournal(w, () => 12.5))
            {
                journal.WriteHeader(hello);
                Assert.IsTrue(journal.Write(3.0, "camera", new JObject { ["view"] = "free" }));
            }
            var lines = Lines(w);
            Assert.AreEqual(2, lines.Length);
            Assert.AreEqual("session", (string)lines[0]["kind"]);
            foreach (var key in new[] { "scenario", "seed", "mode", "site_checksum", "routing", "provenance", "epoch" })
                Assert.IsTrue(JToken.DeepEquals(hello[key], lines[0][key]), key);
            Assert.IsNull(lines[0]["boxes"]);
            Assert.AreEqual(3.0, (double)lines[1]["t"]);
            Assert.AreEqual(12.5, (double)lines[1]["wall"]);
            Assert.AreEqual("camera", (string)lines[1]["kind"]);
            Assert.AreEqual("free", (string)lines[1]["view"]);
        }

        [Test]
        public void JournalThinsThinnableLinesFirst()
        {
            var w = new StringWriter();
            var journal = new ViewJournal(w, () => 7.5);  // one wall second
            for (var i = 0; i < 150; ++i) journal.Write(i * 0.01, "cursor", null, thinnable: true);
            var normal = Enumerable.Range(0, 20).Count(i => journal.Write(2.0, "toggle"));
            Assert.AreEqual(20, normal);
            Assert.LessOrEqual(journal.Lines, 100);
            Assert.GreaterOrEqual(journal.Thinned, 70);
            Assert.AreEqual(20, Lines(w).Count(l => (string)l["kind"] == "toggle"));
        }

        [Test]
        public void JournalStopsAtByteCap()
        {
            var w = new StringWriter();
            var wall = 0.0;
            var journal = new ViewJournal(w, () => wall, maxBytes: 1000);
            var accepted = 0;
            for (var i = 0; i < 200; ++i, wall += 1.0)
                if (journal.Write(i, "toggle", new JObject { ["panel"] = "minimap" })) ++accepted;
            Assert.Greater(accepted, 0);
            Assert.Less(accepted, 200);
            Assert.LessOrEqual(Encoding.UTF8.GetByteCount(w.ToString()), 1000);
            Assert.IsTrue(journal.Full);
            Assert.IsFalse(journal.Write(999, "toggle"));  // shorter, but the journal stays closed
        }

        [Test]
        public void ThinnableLinesStopBeforeTheByteCap()
        {
            var w = new StringWriter();
            var wall = 0.0;
            var journal = new ViewJournal(w, () => wall, maxBytes: 1000);
            while (journal.Write(1.0, "cursor", new JObject { ["x"] = 0.5 }, thinnable: true)) wall += 1.0;
            Assert.LessOrEqual(Encoding.UTF8.GetByteCount(w.ToString()), 900);
            Assert.IsFalse(journal.Full);
            Assert.IsTrue(journal.Write(2.0, "toggle"));  // actions still fit in the last tenth
        }
    }
}
