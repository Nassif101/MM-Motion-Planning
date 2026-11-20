using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>A loopback stand-in for plant_demo_server: accepts clients, writes frames, records their commands.</summary>
    internal sealed class FakePlantServer : IDisposable
    {
        readonly TcpListener listener = new TcpListener(IPAddress.Loopback, 0);
        TcpClient current;
        public readonly ConcurrentQueue<JObject> Commands = new ConcurrentQueue<JObject>();
        public int Accepted;

        public FakePlantServer() => listener.Start();
        public int Port => ((IPEndPoint)listener.LocalEndpoint).Port;

        /// <summary>A client is waiting to be accepted (a retry the test did not accept).</summary>
        public bool HasPending => listener.Pending();

        public static byte[] Hello(int schema = 3) =>
            FrameCodec.Encode(FrameType.Hello, new JObject { ["schema"] = schema, ["server"] = "fake" }, Array.Empty<float[]>());

        public static byte[] Frame(FrameType type, JObject header, params float[][] blocks) =>
            FrameCodec.Encode(type, header, blocks);

        /// <summary>Waits for a client, then sends the server Hello.</summary>
        public void Accept(int schema = 3, int timeoutMs = 5000)
        {
            var task = listener.AcceptTcpClientAsync();
            if (!task.Wait(timeoutMs)) throw new TimeoutException("no client connected");
            current = task.Result;
            current.NoDelay = true;
            Interlocked.Increment(ref Accepted);
            Write(Hello(schema));
        }

        public void Write(byte[] bytes) => current.GetStream().Write(bytes, 0, bytes.Length);

        public void WriteOneBytePerCall(byte[] bytes)
        {
            foreach (var b in bytes)
            {
                current.GetStream().WriteByte(b);
                current.GetStream().Flush();
            }
        }

        /// <summary>Reads one Control frame from the current client.</summary>
        public JObject ReadCommand(int timeoutMs = 5000)
        {
            var stream = current.GetStream();
            stream.ReadTimeout = timeoutMs;
            var buffer = new List<byte>();
            var chunk = new byte[4096];
            while (true)
            {
                var array = buffer.ToArray();
                if (FrameCodec.TryDecode(array, out var frame, out _))
                {
                    Commands.Enqueue(frame.Header);
                    return frame.Header;
                }
                var n = stream.Read(chunk, 0, chunk.Length);
                if (n <= 0) throw new InvalidOperationException("client closed");
                for (var i = 0; i < n; ++i) buffer.Add(chunk[i]);
            }
        }

        public void CloseClient() => current?.Close();

        public void Dispose()
        {
            current?.Close();
            listener.Stop();
        }
    }
}
