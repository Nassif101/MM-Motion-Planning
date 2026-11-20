using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Threading;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    public enum ConnectionStatus
    {
        Connecting,
        Connected,
        SchemaMismatch,
        Closed,
    }

    /// <summary>
    /// The viewer's connection to plant_demo_server (design section 5.3). One background thread connects, reads and
    /// decodes; the main thread takes frames with <see cref="TryDequeue"/> and never blocks on the network. The queue
    /// is bounded: past <c>capacity</c>, the oldest State, Plans or Scan frame is discarded; Hello, Event, Goal and
    /// Route frames are kept in order. More than four times the capacity of kept frames drops the connection.
    /// A lost connection is retried every <c>retrySeconds</c>; the client never sends a reset of its own.
    /// </summary>
    public sealed class StreamClient : IDisposable
    {
        readonly string host;
        readonly int port;
        readonly Func<JObject> helloCommand;
        readonly int capacity;
        readonly TimeSpan retry;
        readonly LinkedList<Frame> queue = new LinkedList<Frame>();
        readonly object sendLock = new object();
        readonly ManualResetEventSlim stopping = new ManualResetEventSlim(false);
        Thread thread;
        Socket socket;
        volatile ConnectionStatus status = ConnectionStatus.Connecting;
        long dropped, malformed, received;

        public StreamClient(string host, int port, Func<JObject> helloCommand, int capacity = 256, double retrySeconds = 2.0)
        {
            this.host = host ?? throw new ArgumentNullException(nameof(host));
            this.port = port;
            this.helloCommand = helloCommand ?? throw new ArgumentNullException(nameof(helloCommand));
            if (capacity < 1) throw new ArgumentOutOfRangeException(nameof(capacity));
            if (!(retrySeconds > 0)) throw new ArgumentOutOfRangeException(nameof(retrySeconds));
            this.capacity = capacity;
            retry = TimeSpan.FromSeconds(retrySeconds);
        }

        public ConnectionStatus Status => status;
        public long Dropped => Interlocked.Read(ref dropped);
        public long Malformed => Interlocked.Read(ref malformed);

        /// <summary>Frames decoded since start, dropped ones included.</summary>
        public long Received => Interlocked.Read(ref received);

        public void Start()
        {
            if (thread != null) throw new InvalidOperationException("already started");
            thread = new Thread(Run) { IsBackground = true, Name = "PlantStreamClient" };
            thread.Start();
        }

        public bool TryDequeue(out Frame frame)
        {
            lock (queue)
            {
                if (queue.Count == 0)
                {
                    frame = null;
                    return false;
                }
                frame = queue.First.Value;
                queue.RemoveFirst();
                return true;
            }
        }

        /// <summary>Sends a Control command; false when not connected (the command is not queued).</summary>
        public bool Send(JObject command)
        {
            var bytes = FrameCodec.Encode(FrameType.Control, command, Array.Empty<float[]>());
            lock (sendLock)
            {
                if (status != ConnectionStatus.Connected || socket == null) return false;
                try
                {
                    socket.Send(bytes);
                    return true;
                }
                catch (Exception e) when (e is SocketException || e is ObjectDisposedException)
                {
                    return false;
                }
            }
        }

        void Run()
        {
            while (!stopping.IsSet)
            {
                var outcome = Session();
                if (outcome == ConnectionStatus.SchemaMismatch || stopping.IsSet) break;
                status = ConnectionStatus.Connecting;
                stopping.Wait(retry);
            }
            if (status != ConnectionStatus.SchemaMismatch) status = ConnectionStatus.Closed;
        }

        // One connection, until it ends; returns SchemaMismatch to stop retrying.
        ConnectionStatus Session()
        {
            var s = new Socket(SocketType.Stream, ProtocolType.Tcp) { NoDelay = true };
            try
            {
                s.Connect(host, port);
                lock (sendLock) socket = s;
                var buffer = new byte[64 * 1024];
                var filled = 0;
                var handshaken = false;
                while (!stopping.IsSet)
                {
                    if (filled == buffer.Length) Array.Resize(ref buffer, Math.Min(buffer.Length * 2, FrameCodec.MaxFrameBytes + 4));
                    var n = s.Receive(buffer, filled, buffer.Length - filled, SocketFlags.None);
                    if (n <= 0) return ConnectionStatus.Connecting;
                    filled += n;
                    var at = 0;
                    while (FrameCodec.TryDecode(new ReadOnlySpan<byte>(buffer, at, filled - at), out var frame, out var used))
                    {
                        at += used;
                        Interlocked.Increment(ref received);
                        if (!handshaken)
                        {
                            if (frame.Type != FrameType.Hello) return ConnectionStatus.Connecting;
                            var schema = frame.Header["schema"];
                            if (schema == null || schema.Type != JTokenType.Integer || (int)schema != ViewerCommands.Schema)
                            {
                                status = ConnectionStatus.SchemaMismatch;
                                return ConnectionStatus.SchemaMismatch;
                            }
                            s.Send(FrameCodec.Encode(FrameType.Control, helloCommand(), Array.Empty<float[]>()));
                            handshaken = true;
                            status = ConnectionStatus.Connected;
                        }
                        if (!Enqueue(frame)) return ConnectionStatus.Connecting;
                    }
                    if (at > 0)
                    {
                        Buffer.BlockCopy(buffer, at, buffer, 0, filled - at);
                        filled -= at;
                    }
                }
                return ConnectionStatus.Closed;
            }
            catch (MalformedFrameException)
            {
                Interlocked.Increment(ref malformed);
                return ConnectionStatus.Connecting;
            }
            catch (Exception e) when (e is SocketException || e is ObjectDisposedException)
            {
                return ConnectionStatus.Connecting;
            }
            finally
            {
                lock (sendLock) socket = null;
                s.Close();
            }
        }

        static bool Replaceable(FrameType type) =>
            type == FrameType.State || type == FrameType.Plans || type == FrameType.Scan;

        // False when the kept backlog passes four times the capacity: the connection is dropped.
        bool Enqueue(Frame frame)
        {
            lock (queue)
            {
                var added = queue.AddLast(frame);
                if (queue.Count <= capacity) return true;
                for (var node = queue.First; node != added; node = node.Next)  // the newest frame always stays
                {
                    if (!Replaceable(node.Value.Type)) continue;
                    queue.Remove(node);
                    Interlocked.Increment(ref dropped);
                    return true;
                }
                if (queue.Count <= 4 * capacity) return true;
                Interlocked.Increment(ref dropped);
                return false;
            }
        }

        public void Dispose()
        {
            stopping.Set();
            lock (sendLock) socket?.Close();
            thread?.Join(TimeSpan.FromSeconds(2));
            if (status != ConnectionStatus.SchemaMismatch) status = ConnectionStatus.Closed;
        }
    }
}
