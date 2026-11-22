using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>The directed view at one simulation time (spec 4C): what the render applies to its frame.</summary>
    public sealed class JournalView
    {
        /// <summary>The camera view's name (Orbit, Chase, Payload, Free), or null before any camera line.</summary>
        public string Camera { get; internal set; }

        /// <summary>The orbit camera's yaw offset (deg), pitch (deg) and distance (m), or null before any orbit line.</summary>
        public (double yaw, double pitch, double distance)? Orbit { get; internal set; }

        /// <summary>The free camera's Unity position and rotation (x, y, z, w), or null before any free-camera line.</summary>
        public double[] FreePosition { get; internal set; }
        public double[] FreeRotation { get; internal set; }

        public string Preset { get; internal set; }

        /// <summary>Overlay and panel toggles by name (as OverlaySettings names them), the latest at or before the time.</summary>
        public IReadOnlyDictionary<string, bool> Toggles { get; internal set; } = new Dictionary<string, bool>();
    }

    /// <summary>
    /// A view journal read for rendering (spec 4C). Applied kinds: <c>camera</c>, <c>orbit</c>, <c>free_camera</c>,
    /// <c>overlay_state</c> and <c>preset</c> (every toggle), <c>overlay</c> and <c>panel</c> (one toggle). Every other kind
    /// (actions, cursor, goals, pace...) is skipped and counted. Orbit and free-camera lines are written at up to 10 Hz
    /// of wall time while the camera moves, and once more, marked <c>"still": true</c>, at the pose where it stops
    /// (Task 3 review I1). Between two lines the camera is interpolated, except after a still line (it rests until the
    /// next) or across more than <see cref="MoveGapWallS"/> of wall time (a fallback for a journal cut short). Lines at
    /// one simulation time (moves made while paused) are cuts: the last one applies.
    /// </summary>
    public sealed class JournalTrack
    {
        public const double MoveGapWallS = 1.0;

        readonly List<JObject> lines = new List<JObject>();
        readonly Dictionary<string, int> skipped = new Dictionary<string, int>();

        public JObject Header { get; private set; } = new JObject();
        public string LogSha256 => (string)Header["log_sha256"];
        public string HelloSha256 => (string)Header["hello_sha256"];
        public IReadOnlyDictionary<string, int> SkippedKinds => skipped;

        static readonly HashSet<string> Applied = new HashSet<string>
            { "camera", "orbit", "free_camera", "overlay_state", "preset", "overlay", "panel" };

        public static JournalTrack Load(string path) => Parse(File.ReadLines(path));

        public static JournalTrack Parse(IEnumerable<string> text)
        {
            var track = new JournalTrack();
            foreach (var raw in text)
            {
                if (string.IsNullOrWhiteSpace(raw)) continue;
                var line = JObject.Parse(raw);
                var kind = (string)line["kind"] ?? "";
                if (kind == "session")
                {
                    if (track.lines.Count == 0) track.Header = line;
                    continue;
                }
                if (!Applied.Contains(kind) || line["t"] == null)
                {
                    track.skipped[kind] = track.skipped.TryGetValue(kind, out var n) ? n + 1 : 1;
                    continue;
                }
                track.lines.Add(line);
            }
            // in simulation time; a stable sort keeps the written order of lines at one time (the last one wins)
            var ordered = track.lines.Select((l, i) => (l, i)).OrderBy(x => (double)x.l["t"]).ThenBy(x => x.i).Select(x => x.l).ToList();
            track.lines.Clear();
            track.lines.AddRange(ordered);
            return track;
        }

        public JournalView ViewAt(double simT)
        {
            var view = new JournalView();
            var toggles = new Dictionary<string, bool>();
            JObject orbit = null, orbitNext = null, free = null, freeNext = null;
            foreach (var line in lines)
            {
                var t = (double)line["t"];
                var kind = (string)line["kind"];
                if (t > simT + 1e-9)
                {
                    if (kind == "orbit" && orbitNext == null) orbitNext = line;
                    if (kind == "free_camera" && freeNext == null) freeNext = line;
                    continue;
                }
                switch (kind)
                {
                    case "camera":
                        view.Camera = (string)line["view"];
                        break;
                    case "orbit":
                        orbit = line;
                        break;
                    case "free_camera":
                        free = line;
                        break;
                    case "overlay_state":
                    case "preset":
                        view.Preset = (string)line["preset"];
                        foreach (var group in new[] { "overlays", "panels" })
                            if (line[group] is JObject g)
                                foreach (var p in g.Properties()) toggles[p.Name] = (bool)p.Value;
                        break;
                    case "overlay":
                    case "panel":
                        toggles[(string)line["toggle"]] = (bool)line["on"];
                        if (line["preset"] != null) view.Preset = (string)line["preset"];
                        break;
                }
            }
            view.Toggles = toggles;
            if (orbit != null)
            {
                var f = Fraction(orbit, orbitNext, simT);
                double V(JObject l, string k) => (double)l[k];
                view.Orbit = f <= 0.0
                    ? (V(orbit, "yaw"), V(orbit, "pitch"), V(orbit, "distance"))
                    : (Lerp(V(orbit, "yaw"), V(orbitNext, "yaw"), f), Lerp(V(orbit, "pitch"), V(orbitNext, "pitch"), f),
                        Lerp(V(orbit, "distance"), V(orbitNext, "distance"), f));
            }
            if (free != null)
            {
                var f = Fraction(free, freeNext, simT);
                var p0 = free["position"].ToObject<double[]>();
                var q0 = free["rotation"].ToObject<double[]>();
                if (f <= 0.0)
                {
                    view.FreePosition = p0;
                    view.FreeRotation = q0;
                }
                else
                {
                    var p1 = freeNext["position"].ToObject<double[]>();
                    var q1 = freeNext["rotation"].ToObject<double[]>();
                    view.FreePosition = new[] { Lerp(p0[0], p1[0], f), Lerp(p0[1], p1[1], f), Lerp(p0[2], p1[2], f) };
                    var q = System.Numerics.Quaternion.Slerp(
                        new System.Numerics.Quaternion((float)q0[0], (float)q0[1], (float)q0[2], (float)q0[3]),
                        new System.Numerics.Quaternion((float)q1[0], (float)q1[1], (float)q1[2], (float)q1[3]), (float)f);
                    view.FreeRotation = new double[] { q.X, q.Y, q.Z, q.W };
                }
            }
            return view;
        }

        /// <summary>How far <paramref name="simT"/> is from <paramref name="a"/> to <paramref name="b"/>; 0 (hold
        /// <paramref name="a"/>) without a next line, across a still period, or at a cut.</summary>
        static double Fraction(JObject a, JObject b, double simT)
        {
            if (b == null) return 0.0;
            double ta = (double)a["t"], tb = (double)b["t"];
            var wallGap = (double)b["wall"] - (double)a["wall"];
            if (tb <= ta || wallGap > MoveGapWallS || ((bool?)a["still"] ?? false)) return 0.0;
            return Math.Max(0.0, Math.Min(1.0, (simT - ta) / (tb - ta)));
        }

        static double Lerp(double a, double b, double f) => a + f * (b - a);
    }

    /// <summary>The render's checks and frame times (spec 4C).</summary>
    public static class RenderPlan
    {
        public const int FramesPerSecond = 60;

        /// <summary>
        /// A directing journal (with <c>log_sha256</c>) renders only with its own log. A live journal (with
        /// <c>hello_sha256</c> only) renders when its session Hello matches the log's, with a warning: identical
        /// sessions (same scenario and seed) cannot be told apart. A journal with neither is refused.
        /// </summary>
        public static (bool ok, string warning, string refusal) Check(RunLog log, JournalTrack journal)
        {
            if (journal.LogSha256 != null)
                return journal.LogSha256 == log.Sha256
                    ? (true, "", "")
                    : (false, "", $"the journal was directed on another log ({(string)journal.Header["log"]}), not {log.Name}");
            if (journal.HelloSha256 != null)
            {
                var hello = log.Frames.Count > 0 ? log.Frames[0].Header : new JObject();
                return journal.HelloSha256 == JournalIdentity.HelloSha256(hello)
                    ? (true, "a live journal: matched by its session only (identical sessions cannot be told apart)", "")
                    : (false, "", "the journal's session is not the log's");
            }
            return (false, "", "the journal has no log or session hash (a journal older than 4C)");
        }

        public static (int count, Func<int, double> simTimeOf) Frames(RunLog log, double from, double to, double speed) =>
            Frames(log.StartT, log.EndT, from, to, speed);

        /// <summary>Frame n at <c>from + n speed / 60</c>, the range clamped to the log's States; none when it is empty.</summary>
        public static (int count, Func<int, double> simTimeOf) Frames(double startT, double endT, double from, double to, double speed)
        {
            if (!(speed > 0.0)) throw new ArgumentOutOfRangeException(nameof(speed));
            var a = Math.Max(startT, double.IsNaN(from) ? startT : from);
            var b = Math.Min(endT, double.IsNaN(to) ? endT : to);
            if (!(b >= a)) return (0, n => a);
            var step = speed / FramesPerSecond;
            var count = (int)Math.Floor((b - a) / step + 1e-9) + 1;
            return (count, n => a + n * step);
        }
    }
}
