using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Security.Cryptography;
using System.Text;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>A run log read for replay (spec 4C): its frames, its file's hash, and its State time span.</summary>
    public sealed class RunLog
    {
        public string Name { get; internal set; } = "";
        public IReadOnlyList<Frame> Frames { get; internal set; } = Array.Empty<Frame>();

        /// <summary>SHA-256 of the whole file, lowercase hex: a directing journal names its log by it.</summary>
        public string Sha256 { get; internal set; } = "";

        /// <summary>The file ends inside a chunk (a writer that died): the frames are those of the whole chunks.</summary>
        public bool EndsEarly { get; internal set; }

        public double StartT { get; internal set; } = double.NaN;
        public double EndT { get; internal set; } = double.NaN;
    }

    /// <summary>
    /// Reads a run log (stream-protocol.md, Run logs): <c>MMRUNLG1</c>, then chunks
    /// <c>[u32 raw_len][u32 zlib_len][zlib bytes]</c> of frames; the first frame is the session Hello. Schema 3 only, as
    /// the viewer speaks. A truncated or corrupt last chunk ends the log after the whole ones (as plant_stream.RunLog).
    /// </summary>
    public static class RunLogReader
    {
        static readonly byte[] Magic = Encoding.ASCII.GetBytes("MMRUNLG1");

        public static RunLog Load(string path) => Read(File.ReadAllBytes(path), Path.GetFileName(path));

        public static RunLog Read(byte[] data, string name)
        {
            if (data.Length < Magic.Length || !data.AsSpan(0, Magic.Length).SequenceEqual(Magic))
                throw new MalformedFrameException($"{name} is not a run log");
            var frames = new List<Frame>();
            var buffer = new MemoryStream();
            var endsEarly = false;
            var at = Magic.Length;
            while (at + 8 <= data.Length)
            {
                var packedLen = (int)BitConverter.ToUInt32(data, at + 4);
                at += 8;
                if (packedLen < 6 || at + packedLen > data.Length)
                {
                    endsEarly = true;
                    break;
                }
                try
                {
                    // zlib: a 2-byte header, the deflate stream, a 4-byte Adler-32 (not checked)
                    using var z = new DeflateStream(new MemoryStream(data, at + 2, packedLen - 6), CompressionMode.Decompress);
                    buffer.Seek(0, SeekOrigin.End);
                    z.CopyTo(buffer);
                }
                catch (InvalidDataException)
                {
                    endsEarly = true;
                    break;
                }
                at += packedLen;
                var bytes = buffer.ToArray();
                var pos = 0;
                while (FrameCodec.TryDecode(bytes.AsSpan(pos), out var frame, out var used))
                {
                    frames.Add(frame);
                    pos += used;
                }
                buffer = new MemoryStream();
                buffer.Write(bytes, pos, bytes.Length - pos);
            }
            if (at < data.Length) endsEarly = true;
            if (frames.Count == 0 || frames[0].Type != FrameType.Hello)
                throw new MalformedFrameException($"{name} has no session Hello");
            var schema = (int?)frames[0].Header["schema"];
            if (schema != ViewerCommands.Schema)
                throw new MalformedFrameException($"{name}: run log schema {schema?.ToString() ?? "missing"}, the viewer reads {ViewerCommands.Schema}");
            var log = new RunLog { Name = name, Frames = frames, EndsEarly = endsEarly };
            using (var sha = SHA256.Create())
            {
                var hash = sha.ComputeHash(data);
                var hex = new StringBuilder(64);
                foreach (var b in hash) hex.Append(b.ToString("x2"));
                log.Sha256 = hex.ToString();
            }
            foreach (var f in frames)
            {
                if (f.Type != FrameType.State) continue;
                var t = (double?)f.Header["t"] ?? double.NaN;
                if (double.IsNaN(log.StartT)) log.StartT = t;
                log.EndT = t;
            }
            return log;
        }
    }
}
