using System;
using System.IO;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// The view journal (design section 6.6): one JSON line per viewer action, stamped with simulation and wall time,
    /// so that stage 4C can render a run log with the same camera work, toggles and clicks. It never feeds the
    /// backend. Bounded: thinnable lines (cursor, free-camera pose) take at most half of each wall second's line
    /// budget and stop at 90% of the byte cap; a line over a limit is dropped and counted; the first line refused by
    /// the byte cap closes the journal.
    /// </summary>
    public sealed class ViewJournal : IDisposable
    {
        static readonly string[] IdentityKeys = { "scenario", "seed", "mode", "site_checksum", "routing", "provenance", "epoch" };

        readonly TextWriter writer;
        readonly Func<double> wallClock;
        readonly int maxLinesPerSecond;
        readonly long maxBytes;
        long second = long.MinValue;
        int linesThisSecond;
        long bytes;

        public ViewJournal(TextWriter writer, Func<double> wallClock, int maxLinesPerSecond = 100, long maxBytes = 50_000_000)
        {
            this.writer = writer ?? throw new ArgumentNullException(nameof(writer));
            this.wallClock = wallClock ?? throw new ArgumentNullException(nameof(wallClock));
            if (maxLinesPerSecond < 2) throw new ArgumentOutOfRangeException(nameof(maxLinesPerSecond));
            if (maxBytes < 1) throw new ArgumentOutOfRangeException(nameof(maxBytes));
            this.maxLinesPerSecond = maxLinesPerSecond;
            this.maxBytes = maxBytes;
        }

        /// <summary>Lines written, the session header included.</summary>
        public long Lines { get; private set; }

        /// <summary>Lines dropped by the per-second or byte limits.</summary>
        public long Thinned { get; private set; }

        /// <summary>True once a line hit the byte cap; every later line is refused.</summary>
        public bool Full { get; private set; }

        /// <summary>
        /// Writes the session's identity from its Hello, so a renderer can match the journal to a run log: the identity
        /// keys, <c>hello_sha256</c> and, for a directing pass in replay (spec 4C), <paramref name="extra"/>'s <c>log</c>
        /// and <c>log_sha256</c>. The header is written whatever the limits (spec 4C: a 4A deferred item).
        /// </summary>
        public void WriteHeader(JObject sessionHello, JObject extra = null)
        {
            var line = new JObject { ["kind"] = "session", ["wall"] = wallClock() };
            foreach (var key in IdentityKeys)
                if (sessionHello.TryGetValue(key, out var value)) line[key] = value.DeepClone();
            line["hello_sha256"] = JournalIdentity.HelloSha256(sessionHello);
            if (extra != null)
                foreach (var property in extra.Properties()) line[property.Name] = property.Value.DeepClone();
            var text = line.ToString(Formatting.None) + "\n";
            writer.Write(text);
            writer.Flush();
            bytes += Encoding.UTF8.GetByteCount(text);
            ++Lines;
        }

        /// <summary>Writes one action; false when a limit dropped it.</summary>
        public bool Write(double simTime, string kind, JObject fields = null, bool thinnable = false)
        {
            var line = new JObject { ["t"] = simTime, ["wall"] = wallClock(), ["kind"] = kind };
            if (fields != null)
                foreach (var property in fields.Properties())
                    if (line[property.Name] == null) line[property.Name] = property.Value.DeepClone();
            return Append(line, thinnable);
        }

        bool Append(JObject line, bool thinnable)
        {
            var now = (long)Math.Floor((double)line["wall"]);
            if (now != second)
            {
                second = now;
                linesThisSecond = 0;
            }
            var budget = thinnable ? maxLinesPerSecond / 2 : maxLinesPerSecond;
            var text = line.ToString(Formatting.None) + "\n";
            var size = Encoding.UTF8.GetByteCount(text);
            if (!Full && bytes + size > maxBytes) Full = true;  // the journal stays a gap-free prefix of the session
            var byteBudget = thinnable ? maxBytes * 9 / 10 : maxBytes;
            if (Full || linesThisSecond >= budget || bytes + size > byteBudget)
            {
                ++Thinned;
                return false;
            }
            writer.Write(text);
            writer.Flush();
            bytes += size;
            ++linesThisSecond;
            ++Lines;
            return true;
        }

        public void Dispose() => writer.Dispose();
    }

    /// <summary>How a journal names the session it was made in (spec 4C).</summary>
    public static class JournalIdentity
    {
        /// <summary>SHA-256 (lowercase hex) of the session Hello's JSON as this viewer writes it (compact, in received
        /// key order): the same for a live session and for its run log.</summary>
        public static string HelloSha256(JObject hello)
        {
            using var sha = System.Security.Cryptography.SHA256.Create();
            var hash = sha.ComputeHash(Encoding.UTF8.GetBytes(hello.ToString(Formatting.None)));
            var hex = new StringBuilder(64);
            foreach (var b in hash) hex.Append(b.ToString("x2"));
            return hex.ToString();
        }
    }
}
