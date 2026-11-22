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

        // Task 4 live check: fed only up to the clock, the newest State is before it and the robot has no sample
        [Test]
        public void ReplayShowsTheRobotAtTheClock()
        {
            var feed = new ReplayFeed(RunLogReader.Read(Fixture(), "f"));
            var s = new ViewerState();
            foreach (var t in new[] { 0.03, 0.13, 0.27, 0.39 })
            {
                feed.FeedFor(t, f => s.Apply(f, 0.0));
                Assert.IsTrue(s.TrySampleAt(t, out var sample), $"t {t}");
                Assert.AreEqual(0.5 * t, sample.X, 1e-6);
            }
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
            var state = new Frame { Type = FrameType.State, Header = new JObject { ["t"] = 0.12 }, Blocks = Array.Empty<float[]>() };
            gate.Offer(Plans(0.1), applied.Add);
            gate.Offer(state, applied.Add);
            CollectionAssert.AreEqual(new[] { state }, applied, "States pass at once");
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
    
        // Task 2 review I1: a goal's end that came after its plan is applied after it (the gate keeps stream order), so
        // the ended plan does not come back
        [Test]
        public void GateKeepsStreamOrder()
        {
            var gate = new FrameGate();
            var applied = new List<Frame>();
            Frame state(double t) => new Frame { Type = FrameType.State, Header = new JObject { ["t"] = t }, Blocks = Array.Empty<float[]>() };
            var ended = new Frame { Type = FrameType.Event, Header = new JObject { ["event"] = "goal_reached" }, Blocks = Array.Empty<float[]>() };
            gate.Offer(state(0.08), applied.Add);
            gate.Offer(Plans(0.1), applied.Add);
            gate.Offer(ended, applied.Add);  // no time of its own: the stream's, behind the plan
            gate.Release(0.09, applied.Add);
            Assert.AreEqual(1, applied.Count, "the event waits behind the held plan");
            gate.Release(0.1, applied.Add);
            CollectionAssert.AreEqual(new[] { FrameType.State, FrameType.Plans, FrameType.Event }, applied.Select(f => f.Type).ToArray());
            var s = new ViewerState();
            s.Apply(new Frame { Type = FrameType.Hello, Header = new JObject { ["schema"] = 3, ["scenario"] = "x", ["config"] = new JObject { ["medium"] = true } }, Blocks = Array.Empty<float[]>() }, 0.0);
            foreach (var f in applied) s.Apply(f, 0.0);
            Assert.IsNull(s.Plan, "a recorded goal's end clears its plan, in stream order");
        }

        // Spec 4C Task 4: a held plan of the old session never lands in the new one
        [Test]
        public void GateClearsOnASessionHello()
        {
            var gate = new FrameGate();
            var applied = new List<Frame>();
            gate.Offer(Plans(5.0), applied.Add);
            gate.Offer(new Frame { Type = FrameType.Hello, Header = new JObject { ["scenario"] = "x" }, Blocks = Array.Empty<float[]>() }, applied.Add);
            gate.Release(10.0, applied.Add);
            Assert.AreEqual(1, applied.Count);
            Assert.AreEqual(FrameType.Hello, applied[0].Type);
        }

    }
}
