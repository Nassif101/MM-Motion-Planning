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
    /// Spec 4C Task 2: run logs read for replay (the shared fixture tools/plant/fixtures/runlog_v3.mmlog), the replay
    /// clock and feed, and overlays held until the displayed time.
    /// </summary>
    public sealed class ReplayTests
    {
        static byte[] Fixture() => ViewerFixtures.Read("runlog_v3.mmlog");

        [Test]
        public void RunLogReadsTheFixtureAcrossChunks()
        {
            var log = RunLogReader.Read(Fixture(), "runlog_v3.mmlog");
            Assert.AreEqual(25, log.Frames.Count);
            Assert.AreEqual(FrameType.Hello, log.Frames[0].Type);
            Assert.AreEqual(20, log.Frames.Count(f => f.Type == FrameType.State));
            Assert.AreEqual(2, log.Frames.Count(f => f.Type == FrameType.Prediction));
            Assert.AreEqual(0.02, log.StartT, 1e-9);
            Assert.AreEqual(0.40, log.EndT, 1e-9);
            Assert.IsFalse(log.EndsEarly);
            Assert.AreEqual(64, log.Sha256.Length);
            Assert.AreEqual(log.Sha256, RunLogReader.Read(Fixture(), "copy").Sha256);
        }

        [Test]
        public void TruncatedLogPlaysToTheLastWholeFrame()
        {
            var bytes = Fixture();
            var cut = bytes.Take(bytes.Length - 10).ToArray();  // inside the second chunk
            var log = RunLogReader.Read(cut, "cut.mmlog");
            Assert.IsTrue(log.EndsEarly);
            Assert.AreEqual(7, log.Frames.Count, "the first chunk's frames");
            Assert.AreEqual(FrameType.Hello, log.Frames[0].Type);
        }

        [Test]
        public void NonSchema3LogIsRefused()
        {
            Assert.Throws<MalformedFrameException>(() => RunLogReader.Read(new byte[] { 1, 2, 3, 4, 5, 6, 7, 8, 9 }, "bad"));
            var hello = FrameCodec.Encode(FrameType.Hello, new JObject { ["schema"] = 1, ["scenario"] = "x" }, Array.Empty<float[]>());
            Assert.Throws<MalformedFrameException>(() => RunLogReader.Read(Log(hello), "schema1"));
        }

        // A one-chunk run log of the given frames' bytes
        static byte[] Log(params byte[][] frames)
        {
            var raw = frames.SelectMany(f => f).ToArray();
            using var packed = new MemoryStream();
            using (var z = new System.IO.Compression.DeflateStream(packed, System.IO.Compression.CompressionLevel.Optimal, true))
                z.Write(raw, 0, raw.Length);
            var deflate = packed.ToArray();
            var zlib = new byte[] { 0x78, 0x9c }.Concat(deflate).Concat(new byte[4]).ToArray();  // header; Adler-32 unchecked
            return System.Text.Encoding.ASCII.GetBytes("MMRUNLG1").Concat(BitConverter.GetBytes((uint)raw.Length))
                .Concat(BitConverter.GetBytes((uint)zlib.Length)).Concat(zlib).ToArray();
        }

        [Test]
        public void ReplayClockPlaysPausesAndRestarts()
        {
            var clock = new ReplayClock(0.02, 0.40);
            Assert.AreEqual(0.02, clock.T, 1e-12);
            Assert.IsFalse(clock.Playing);
            clock.Advance(1.0);
            Assert.AreEqual(0.02, clock.T, 1e-12, "paused");
            clock.Playing = true;
            clock.Speed = 0.1;
            clock.Advance(1.0);
            Assert.AreEqual(0.12, clock.T, 1e-9);
            clock.Advance(10.0);
            Assert.AreEqual(0.40, clock.T, 1e-12, "stops at the end");
            Assert.IsTrue(clock.AtEnd);
            Assert.IsFalse(clock.Playing);
            clock.Restart();
            Assert.AreEqual(0.02, clock.T, 1e-12);
            Assert.IsFalse(clock.AtEnd);
        }

        [Test]
        public void ReplayFeedAppliesEachFrameOnce()
        {
            var feed = new ReplayFeed(RunLogReader.Read(Fixture(), "f"));
            var seen = new List<Frame>();
            feed.FeedUntil(0.1, seen.Add);
            var first = seen.Count;
            Assert.AreEqual(FrameType.Hello, seen[0].Type);
            Assert.AreEqual(0.1, (double)seen.Last(f => f.Type == FrameType.State).Header["t"], 1e-9);
            feed.FeedUntil(0.1, seen.Add);
            Assert.AreEqual(first, seen.Count, "nothing twice");
            feed.FeedUntil(1.0, seen.Add);
            Assert.AreEqual(25, seen.Count);
            feed.Reset();
            var again = new List<Frame>();
            feed.FeedUntil(0.02, again.Add);
            Assert.AreEqual(FrameType.Hello, again[0].Type);
        }

        static Frame Plans(double t) => new Frame
        {
            Type = FrameType.Plans,
            Header = new JObject { ["t"] = t, ["plan_id"] = 3 },
            Blocks = new[] { new float[9] },
        };

        [Test]
        public void OverlaysWaitForTheDisplayedTime()
        {
            var gate = new FrameGate();
            var applied = new List<Frame>();
            var goal = new Frame { Type = FrameType.Goal, Header = new JObject(), Blocks = Array.Empty<float[]>() };
            gate.Offer(Plans(0.1), applied.Add);
            gate.Offer(goal, applied.Add);
            CollectionAssert.AreEqual(new[] { goal }, applied, "other frames pass at once");
            gate.Release(0.05, applied.Add);
            Assert.AreEqual(1, applied.Count, "the plan is newer than the displayed time");
            gate.Release(0.1, applied.Add);
            Assert.AreEqual(FrameType.Plans, applied.Last().Type);
            gate.Offer(Plans(0.3), applied.Add);
            gate.Clear();
            gate.Release(1.0, applied.Add);
            Assert.AreEqual(2, applied.Count, "cleared");
        }

        [Test]
        public void SampleAtIsExactBetweenStates()
        {
            var s = new ViewerState();
            foreach (var f in RunLogReader.Read(Fixture(), "f").Frames) s.Apply(f, 0.0);
            Assert.IsTrue(s.TrySampleAt(0.13, out var sample));
            Assert.AreEqual(0.13, sample.T, 1e-12);
            Assert.AreEqual(0.065, sample.X, 1e-6);
            Assert.AreEqual(0.968, sample.Joints[1], 1e-6);
            Assert.IsFalse(s.TrySampleAt(0.5, out _), "past the newest State");
            Assert.AreEqual(0.12, (double)s.StateHeaderAt(0.13)["t"], 1e-9);
            Assert.IsNull(s.StateHeaderAt(0.0));
        }

        [Test]
        public void RecordedPlanClearsWhenItsGoalEnds()
        {
            var s = new ViewerState();
            s.Apply(new Frame
            {
                Type = FrameType.Hello,
                Header = new JObject { ["schema"] = 3, ["scenario"] = "v2_door_105_nav", ["config"] = new JObject { ["medium"] = true } },
                Blocks = Array.Empty<float[]>(),
            }, 0.0);
            s.Apply(Plans(1.0), 0.0);
            Assert.IsNotNull(s.Plan);
            s.Apply(new Frame { Type = FrameType.Event, Header = new JObject { ["event"] = "goal_reached", ["goal"] = 0 }, Blocks = Array.Empty<float[]>() }, 0.0);
            Assert.IsNull(s.Plan, "a recorded goal ended");
        }
    }
}
