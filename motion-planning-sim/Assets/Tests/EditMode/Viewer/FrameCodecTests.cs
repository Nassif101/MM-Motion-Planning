using System;
using System.Collections.Generic;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class FrameCodecTests
    {
        [Test]
        public void EncodesTheCommittedFixtureByteForByte()
        {
            var header = new JObject { ["t"] = 1.25, ["name"] = "fixture", ["list"] = new JArray(1, 2, 3) };
            var blocks = new List<float[]> { new[] { 1.5f, -2.0f }, Array.Empty<float>(), new[] { 3.25f } };
            var expected = ViewerFixtures.Read("frame_v1.bin");
            CollectionAssert.AreEqual(expected, FrameCodec.Encode(FrameType.State, header, blocks));

            Assert.IsTrue(FrameCodec.TryDecode(expected, out var frame, out var consumed));
            Assert.AreEqual(expected.Length, consumed);
            Assert.AreEqual(FrameType.State, frame.Type);
            Assert.AreEqual("fixture", (string)frame.Header["name"]);
            Assert.AreEqual(3, frame.Blocks.Length);
            CollectionAssert.AreEqual(new[] { 1.5f, -2.0f }, frame.Blocks[0]);
            Assert.AreEqual(0, frame.Blocks[1].Length);
            CollectionAssert.AreEqual(new[] { 3.25f }, frame.Blocks[2]);
        }

        [Test]
        public void PartialBufferIsIncomplete()
        {
            var bytes = ViewerFixtures.Read("frame_v1.bin");
            for (var n = 0; n < bytes.Length; ++n)
                Assert.IsFalse(FrameCodec.TryDecode(new ReadOnlySpan<byte>(bytes, 0, n), out _, out _), $"prefix {n}");
        }

        [Test]
        public void DecodesGoalAndRouteFixtures()
        {
            Assert.IsTrue(FrameCodec.TryDecode(ViewerFixtures.Read("goal_v3.bin"), out var goal, out _));
            Assert.AreEqual(FrameType.Goal, goal.Type);
            Assert.AreEqual("goal_accepted", (string)goal.Header["event"]);
            Assert.AreEqual(4, (long)goal.Header["goal"]["goal_id"]);
            Assert.AreEqual(new[] { 1.5, -2.25, 0.5 }, goal.Header["goal"]["target"].ToObject<double[]>());
            Assert.AreEqual("planning", (string)goal.Header["goal"]["state"]);

            Assert.IsTrue(FrameCodec.TryDecode(ViewerFixtures.Read("goal_rejected_v3.bin"), out var rejected, out _));
            Assert.AreEqual(JTokenType.Null, rejected.Header["goal"].Type);
            Assert.AreEqual("goal_rejected", (string)rejected.Header["event"]);
            Assert.AreEqual(new[] { 101.0, -2.25, 0.5 }, rejected.Header["request"]["target"].ToObject<double[]>());

            Assert.IsTrue(FrameCodec.TryDecode(ViewerFixtures.Read("route_v3.bin"), out var route, out _));
            Assert.AreEqual(FrameType.Route, route.Type);
            Assert.AreEqual(3, (int)route.Header["path_poses"]);
            Assert.AreEqual(9, route.Blocks[0].Length);  // 3 poses x (x, y, yaw)
            CollectionAssert.AreEqual(new[] { 1.5f, -2.25f, 0.25f }, new ArraySegment<float>(route.Blocks[0], 6, 3));
            CollectionAssert.AreEqual(new[] { 0.5f, -0.5f }, new ArraySegment<float>(route.Blocks[1], 2, 2));
        }

        [Test]
        public void OversizedOrInconsistentFramesAreMalformed()
        {
            var oversized = BitConverter.GetBytes((uint)(FrameCodec.MaxFrameBytes + 1));
            Assert.Throws<MalformedFrameException>(() => FrameCodec.TryDecode(oversized, out _, out _));

            var jsonPastEnd = ViewerFixtures.Read("frame_v1.bin");
            BitConverter.GetBytes(10_000u).CopyTo(jsonPastEnd, 6);  // json_len after [u32 length][u16 type]
            Assert.Throws<MalformedFrameException>(() => FrameCodec.TryDecode(jsonPastEnd, out _, out _));

            var unknownType = ViewerFixtures.Read("frame_v1.bin");
            BitConverter.GetBytes((ushort)99).CopyTo(unknownType, 4);
            Assert.Throws<MalformedFrameException>(() => FrameCodec.TryDecode(unknownType, out _, out _));

            var badJson = FrameCodec.Encode(FrameType.Event, new JObject { ["event"] = "x" }, Array.Empty<float[]>());
            badJson[10] = (byte)'#';  // the JSON's first byte
            Assert.Throws<MalformedFrameException>(() => FrameCodec.TryDecode(badJson, out _, out _));
        }
    }
}
