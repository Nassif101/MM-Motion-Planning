using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>Frame types of the plant stream protocol (stream-protocol.md, schema 3).</summary>
    public enum FrameType : ushort
    {
        Hello = 1,
        Control = 2,
        Event = 3,
        State = 10,
        Plans = 11,
        Scan = 12,
        Goal = 20,
        Route = 21,
    }

    /// <summary>One decoded frame: its type, JSON header and float32 blocks.</summary>
    public sealed class Frame
    {
        public FrameType Type;
        public JObject Header;
        public float[][] Blocks;
    }

    /// <summary>A frame whose lengths disagree, whose type is unknown, whose JSON is bad, or that is too large.</summary>
    public sealed class MalformedFrameException : Exception
    {
        public MalformedFrameException(string message, Exception inner = null) : base(message, inner) { }
    }

    /// <summary>
    /// The wire format <c>[u32 length][u16 type][u32 json_len][json][u32 n_blocks]{[u32 n][float32 x n]}</c>,
    /// little endian, where <c>length</c> counts the bytes after itself.
    /// </summary>
    public static class FrameCodec
    {
        public const int MaxFrameBytes = 16 << 20;

        /// <summary>Encodes with nlohmann's compact, key-sorted JSON, as the C++ and Python codecs do.</summary>
        public static byte[] Encode(FrameType type, JObject header, IReadOnlyList<float[]> blocks)
        {
            var json = Encoding.UTF8.GetBytes(Sorted(header).ToString(Formatting.None));
            var length = 2 + 4 + json.Length + 4;
            foreach (var block in blocks) length += 4 + 4 * block.Length;
            var bytes = new byte[4 + length];
            var at = 0;
            BinaryPrimitives.WriteUInt32LittleEndian(bytes.AsSpan(at), (uint)length); at += 4;
            BinaryPrimitives.WriteUInt16LittleEndian(bytes.AsSpan(at), (ushort)type); at += 2;
            BinaryPrimitives.WriteUInt32LittleEndian(bytes.AsSpan(at), (uint)json.Length); at += 4;
            json.CopyTo(bytes, at); at += json.Length;
            BinaryPrimitives.WriteUInt32LittleEndian(bytes.AsSpan(at), (uint)blocks.Count); at += 4;
            foreach (var block in blocks)
            {
                BinaryPrimitives.WriteUInt32LittleEndian(bytes.AsSpan(at), (uint)block.Length); at += 4;
                foreach (var value in block)
                {
                    BinaryPrimitives.WriteInt32LittleEndian(bytes.AsSpan(at), BitConverter.SingleToInt32Bits(value));
                    at += 4;
                }
            }
            return bytes;
        }

        /// <summary>
        /// Decodes the frame at the start of <paramref name="buffer"/>. False while it is incomplete; throws
        /// <see cref="MalformedFrameException"/> as soon as the buffered bytes show it cannot be valid.
        /// </summary>
        public static bool TryDecode(ReadOnlySpan<byte> buffer, out Frame frame, out int consumed)
        {
            frame = null;
            consumed = 0;
            if (buffer.Length < 4) return false;
            var length = BinaryPrimitives.ReadUInt32LittleEndian(buffer);
            if (length > MaxFrameBytes) throw new MalformedFrameException($"frame of {length} bytes exceeds {MaxFrameBytes}");
            if (length < 2 + 4 + 4) throw new MalformedFrameException($"frame length {length} is too short");
            var end = 4 + (int)length;
            if (buffer.Length < end) return false;

            var body = buffer.Slice(4, (int)length);
            var raw = BinaryPrimitives.ReadUInt16LittleEndian(body);
            if (!Enum.IsDefined(typeof(FrameType), raw)) throw new MalformedFrameException($"unknown frame type {raw}");
            var jsonLength = BinaryPrimitives.ReadUInt32LittleEndian(body.Slice(2));
            var at = 6;
            if (jsonLength > (uint)(body.Length - at - 4)) throw new MalformedFrameException("JSON length past the frame");
            var header = ParseHeader(body.Slice(at, (int)jsonLength));
            at += (int)jsonLength;
            var count = BinaryPrimitives.ReadUInt32LittleEndian(body.Slice(at));
            at += 4;
            if (count > (uint)(body.Length - at) / 4) throw new MalformedFrameException("block count past the frame");
            var blocks = new float[count][];
            for (var b = 0; b < count; ++b)
            {
                if (body.Length - at < 4) throw new MalformedFrameException("block header past the frame");
                var n = BinaryPrimitives.ReadUInt32LittleEndian(body.Slice(at));
                at += 4;
                if (n > (uint)(body.Length - at) / 4) throw new MalformedFrameException("block past the frame");
                var values = new float[n];
                for (var i = 0; i < n; ++i, at += 4)
                    values[i] = BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(body.Slice(at)));
                blocks[b] = values;
            }
            if (at != body.Length) throw new MalformedFrameException("bytes left after the last block");

            frame = new Frame { Type = (FrameType)raw, Header = header, Blocks = blocks };
            consumed = end;
            return true;
        }

        static JToken Sorted(JToken token)
        {
            switch (token)
            {
                case JObject o:
                    var sorted = new JObject();
                    foreach (var p in o.Properties().OrderBy(p => p.Name, StringComparer.Ordinal))
                        sorted.Add(p.Name, Sorted(p.Value));
                    return sorted;
                case JArray a:
                    return new JArray(a.Select(Sorted));
                default:
                    return token.DeepClone();
            }
        }

        static JObject ParseHeader(ReadOnlySpan<byte> json)
        {
            try
            {
                using var reader = new JsonTextReader(new StringReader(Encoding.UTF8.GetString(json)))
                {
                    DateParseHandling = DateParseHandling.None,
                    FloatParseHandling = FloatParseHandling.Double,
                };
                var token = JToken.ReadFrom(reader);
                if (reader.Read()) throw new MalformedFrameException("text after the JSON header");
                return token as JObject ?? throw new MalformedFrameException("the JSON header is not an object");
            }
            catch (JsonException e)
            {
                throw new MalformedFrameException("bad JSON header", e);
            }
        }
    }
}
