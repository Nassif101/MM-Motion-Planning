using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class StreamClientTests
    {
        static readonly ViewerCommands Commands = new ViewerCommands();

        static bool WaitFor(Func<bool> condition, int timeoutMs = 5000)
        {
            var watch = Stopwatch.StartNew();
            while (watch.ElapsedMilliseconds < timeoutMs)
            {
                if (condition()) return true;
                Thread.Sleep(10);
            }
            return condition();
        }

        static List<Frame> Drain(StreamClient client)
        {
            var frames = new List<Frame>();
            while (client.TryDequeue(out var f)) frames.Add(f);
            return frames;
        }

        static JObject State(double t) => new JObject { ["t"] = t };

        [Test]
        public void HandshakeSendsHelloAfterServerHello()
        {
            using var server = new FakePlantServer();
            using var client = new StreamClient("127.0.0.1", server.Port, Commands.Hello);
            client.Start();
            server.Accept();
            var hello = server.ReadCommand();
            Assert.IsTrue(JToken.DeepEquals(JObject.Parse("{\"cmd\":\"hello\",\"schema\":3,\"accepts\":[\"prediction\"]}"), hello), hello.ToString());
            Assert.IsTrue(WaitFor(() => client.Status == ConnectionStatus.Connected));
            Assert.IsTrue(WaitFor(() => client.TryDequeue(out var f) && f.Type == FrameType.Hello));
        }

        // Task 3 review C1: "localhost" resolves to ::1 first on macOS; the container publishes 127.0.0.1 only
        [Test]
        public void ConnectsThroughLocalhostToAnIPv4Listener()
        {
            using var server = new FakePlantServer();  // listens on 127.0.0.1 only
            using var client = new StreamClient("localhost", server.Port, Commands.Hello);
            client.Start();
            server.Accept(timeoutMs: 5000);
            server.ReadCommand();
            Assert.IsTrue(WaitFor(() => client.Status == ConnectionStatus.Connected));
        }

        [Test]
        public void SchemaMismatchStopsRetrying()
        {
            using var server = new FakePlantServer();
            using var client = new StreamClient("127.0.0.1", server.Port, Commands.Hello, retrySeconds: 0.1);
            client.Start();
            server.Accept(schema: 4);
            Assert.IsTrue(WaitFor(() => client.Status == ConnectionStatus.SchemaMismatch));
            Thread.Sleep(500);  // five retry periods
            Assert.AreEqual(1, server.Accepted);
            Assert.IsFalse(server.HasPending, "the client retried");
            Assert.AreEqual(ConnectionStatus.SchemaMismatch, client.Status);
        }

        [Test]
        public void OverflowKeepsKeptFramesInOrder()
        {
            using var server = new FakePlantServer();
            using var client = new StreamClient("127.0.0.1", server.Port, Commands.Hello, capacity: 8);
            client.Start();
            server.Accept();
            server.ReadCommand();
            var sent = 1;  // the server Hello
            var expected = new List<string>();
            for (var i = 0; i < 40; ++i)
            {
                server.Write(FakePlantServer.Frame(FrameType.State, State(i * 0.02)));
                ++sent;
                if (i % 8 == 3)
                {
                    var name = $"g{expected.Count(e => e[0] == 'g') + 1}";
                    server.Write(FakePlantServer.Frame(FrameType.Goal, new JObject { ["event"] = name }));
                    expected.Add(name);
                    ++sent;
                }
                if (i % 13 == 5)
                {
                    var name = $"e{expected.Count(e => e[0] == 'e') + 1}";
                    server.Write(FakePlantServer.Frame(FrameType.Event, new JObject { ["event"] = name }));
                    expected.Add(name);
                    ++sent;
                }
            }
            Assert.IsTrue(WaitFor(() => client.Received >= sent), $"received {client.Received} of {sent}");
            var frames = Drain(client);
            var kept = frames.Where(f => f.Type == FrameType.Goal || f.Type == FrameType.Event)
                .Select(f => (string)f.Header["event"]).ToList();
            CollectionAssert.AreEqual(expected, kept);
            Assert.Greater(client.Dropped, 0);
            Assert.AreEqual(0.78, (double)frames.Last(f => f.Type == FrameType.State).Header["t"], 1e-9);
        }

        [Test]
        public void ReconnectsWithoutSendingReset()
        {
            using var server = new FakePlantServer();
            using var client = new StreamClient("127.0.0.1", server.Port, Commands.Hello, retrySeconds: 0.1);
            client.Start();
            server.Accept();
            server.ReadCommand();
            server.CloseClient();
            server.Accept(timeoutMs: 3000);
            server.ReadCommand();
            Assert.IsTrue(WaitFor(() => client.Status == ConnectionStatus.Connected));
            Assert.AreEqual(2, server.Accepted);
            CollectionAssert.AreEqual(new[] { "hello", "hello" }, server.Commands.Select(c => (string)c["cmd"]).ToArray());
        }

        [Test]
        public void FragmentedBytesDecode()
        {
            using var server = new FakePlantServer();
            using var client = new StreamClient("127.0.0.1", server.Port, Commands.Hello);
            client.Start();
            server.Accept();
            server.ReadCommand();
            server.WriteOneBytePerCall(ViewerFixtures.Read("frame_v1.bin"));
            Frame state = null;
            Assert.IsTrue(WaitFor(() =>
            {
                while (client.TryDequeue(out var f))
                    if (f.Type == FrameType.State) state = f;
                return state != null;
            }));
            Assert.AreEqual("fixture", (string)state.Header["name"]);
            Assert.AreEqual(0, client.Malformed);
        }
    }
}
