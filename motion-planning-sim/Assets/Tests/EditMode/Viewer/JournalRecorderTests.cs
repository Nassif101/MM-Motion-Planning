using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>Task 5 review I1 and I3: the per-session journal files, and recording failures kept from the caller.</summary>
    public sealed class JournalRecorderTests
    {
        sealed class FailingWriter : TextWriter
        {
            public override Encoding Encoding => Encoding.UTF8;
            public override void Write(char value) => throw new IOException("disk full");
            public override void Write(string value) => throw new IOException("disk full");
        }

        static JObject Hello(string scenario = "v2_open_yard_nav") =>
            new JObject { ["schema"] = 3, ["scenario"] = scenario, ["seed"] = 1 };

        static double Wall() => 1.0;

        [Test]
        public void AWriteFailureStopsRecordingWithoutThrowing()
        {
            var opened = 0;
            var r = new JournalRecorder(n => { ++opened; return new FailingWriter(); }, Wall);
            Assert.DoesNotThrow(() => r.OnSession(Hello(), replay: false));  // the header write fails
            Assert.That(r.Error, Does.Contain("disk full"));
            Assert.DoesNotThrow(() => r.Write(1.0, "action"));
            Assert.DoesNotThrow(() => r.OnSession(Hello("v2_store_room"), replay: false));
            Assert.AreEqual(1, opened, "recording stays off after a failure");
            Assert.DoesNotThrow(() => r.Dispose());
        }

        [Test]
        public void AFailedOpenStopsRecordingWithoutThrowing()
        {
            var r = new JournalRecorder(n => throw new UnauthorizedAccessException("denied"), Wall);
            Assert.DoesNotThrow(() => r.OnSession(Hello(), replay: false));
            Assert.That(r.Error, Does.Contain("denied"));
            Assert.DoesNotThrow(() => r.Write(1.0, "action"));
            Assert.AreEqual(0, r.Files);
        }

        [Test]
        public void OnlyAReplayOfTheJournaledSessionContinuesItsFile()
        {
            var files = new List<StringWriter>();
            var r = new JournalRecorder(n => { var w = new StringWriter(); files.Add(w); return w; }, Wall);
            r.OnSession(Hello(), replay: true);  // the first session after a connect
            r.Write(12.0, "action");
            r.OnSession(Hello(), replay: true);  // a reconnect's replay of the same session
            Assert.AreEqual(1, r.Files);
            r.OnSession(Hello(), replay: false);  // a reset to the same scenario and seed: a new take
            Assert.AreEqual(2, r.Files);
            Assert.That(files[1].ToString(), Does.StartWith("{").And.Contain("v2_open_yard_nav"), "a new header");
            r.OnSession(Hello("v2_store_room"), replay: true);  // a different session replayed: a new file too
            Assert.AreEqual(3, r.Files);
            Assert.IsNull(r.Error);
        }
    }
}
