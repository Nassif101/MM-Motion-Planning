using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    public enum Severity
    {
        Info,
        Warning,
        Error,
    }

    public sealed class ConsoleEntry
    {
        public Severity Severity;
        public string Subsystem;
        public double? SimT;
        public double Wall;
        public string Text;
        public int Count = 1;
    }

    /// <summary>
    /// The diagnostic console's store (4B-1 design 7.3): a repeat of the latest entry (same severity, subsystem and
    /// text) collapses into its count; at most <c>cap</c> entries, the oldest dropped; at most <c>perFrame</c> adds
    /// between <see cref="BeginFrame"/> calls, the rest dropped. Both drops are counted in <see cref="Dropped"/>.
    /// </summary>
    public sealed class ConsoleLog
    {
        readonly int cap;
        readonly int perFrame;
        readonly List<ConsoleEntry> entries = new List<ConsoleEntry>();
        int addedThisFrame;

        public ConsoleLog(int cap = 2000, int perFrame = 200)
        {
            this.cap = cap;
            this.perFrame = perFrame;
        }

        public IReadOnlyList<ConsoleEntry> Entries => entries;
        public long Dropped { get; private set; }

        /// <summary>Incremented on every change, so the panel rebuilds its text only when the log changed.</summary>
        public int Version { get; private set; }

        public void BeginFrame() => addedThisFrame = 0;

        public bool Add(Severity severity, string subsystem, double? simT, double wall, string text)
        {
            if (addedThisFrame >= perFrame)
            {
                ++Dropped;
                return false;
            }
            ++addedThisFrame;
            ++Version;
            var last = entries.Count > 0 ? entries[entries.Count - 1] : null;
            if (last != null && last.Severity == severity && last.Subsystem == subsystem && last.Text == text)
            {
                ++last.Count;
                last.SimT = simT;
                last.Wall = wall;
                return true;
            }
            entries.Add(new ConsoleEntry { Severity = severity, Subsystem = subsystem, SimT = simT, Wall = wall, Text = text ?? "" });
            if (entries.Count > cap)
            {
                Dropped += entries.Count - cap;
                entries.RemoveRange(0, entries.Count - cap);
            }
            return true;
        }

        /// <param name="subsystems">The subsystems shown, or null for all.</param>
        /// <param name="search">A case-insensitive text filter, or null or empty for none.</param>
        public IEnumerable<ConsoleEntry> Filter(Severity minimum, ISet<string> subsystems, string search) =>
            entries.Where(e => e.Severity >= minimum && (subsystems == null || subsystems.Contains(e.Subsystem)) &&
                               (string.IsNullOrEmpty(search) || e.Text.IndexOf(search, StringComparison.OrdinalIgnoreCase) >= 0));

        public static string Format(ConsoleEntry e)
        {
            var inv = CultureInfo.InvariantCulture;
            var t = e.SimT.HasValue ? "t " + e.SimT.Value.ToString("F2", inv) : "t -";
            var count = e.Count > 1 ? " x" + e.Count.ToString(inv) : "";
            return $"[{e.Severity.ToString().ToLowerInvariant()}] {e.Subsystem} {t} (wall {e.Wall.ToString("F2", inv)}) {e.Text}{count}";
        }

        public static void Export(TextWriter writer, IEnumerable<ConsoleEntry> visible)
        {
            foreach (var e in visible) writer.WriteLine(Format(e));
            writer.Flush();
        }

        /// <summary>A backend Event frame's header as a console line (4B-1 plan, Task 3).</summary>
        public static (Severity severity, string subsystem, string text) FromEvent(JObject e)
        {
            var name = (string)e["event"] ?? "event";
            var rest = new JObject();
            foreach (var p in e.Properties())
                if (p.Name != "event" && p.Name != "t") rest[p.Name] = p.Value;
            if (name == "contact")  // a grazing contact repeats with a new depth each time: the pair alone, so it collapses
                return (Severity.Error, "plant", $"contact {(string)e["robot_geom"]} / {(string)e["other_geom"]}");
            var text = rest.Count == 0 ? name : name + " " + rest.ToString(Formatting.None);
            if (name == "goal_ack")  // a refused submission sends no goal_rejected event: the ack carries the warning
            {
                var action = (string)e["action"];
                var fine = action == "accepted" || action == "applied" || action == "scheduled";
                return (fine ? Severity.Info : Severity.Warning, "goal", text);
            }
            if (name.StartsWith("goal_", StringComparison.Ordinal) || name == "goal_reached" || name == "failed" ||
                name == "timeout" || name == "stalled")
            {
                var warn = name == "goal_rejected" || name == "goal_failed" || name == "goal_timed_out" || name == "failed" ||
                           name == "timeout" || name == "stalled";
                return (warn ? Severity.Warning : Severity.Info, "goal", text);
            }
            if (name.StartsWith("route_", StringComparison.Ordinal) || name == "drive_leg_started") return (Severity.Info, "route", text);
            switch (name)
            {
                case "fallback": return (Severity.Warning, "medium", text);
                case "contact": return (Severity.Error, "plant", text);
                case "behind":
                case "dropped": return (Severity.Warning, "stream", text);
                case "error": return (Severity.Error, "plant", text);
                case "run_stopped": return ((string)e["reason"] == "error" ? Severity.Warning : Severity.Info, "plant", text);
                default: return (Severity.Info, "plant", text);
            }
        }
    }
}
