using System;
using System.Collections.Generic;
using System.Globalization;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// The viewer's command line (spec 4A). Connection: <c>-host</c> (localhost), <c>-port</c> (7700). Recording:
    /// <c>-journal-dir</c> (default ~/Movies/PlantViewer), <c>-frame-times FILE</c> (per-frame times for the FPS
    /// measurement). Rendering: <c>-quality NAME</c> (one of the project's quality levels; default Balanced), <c>-fps N</c>. A
    /// scripted demo, for reproducible measurements and takes: <c>-scenario NAME</c> [<c>-seed N</c>] [<c>-goal x,y,yaw</c>]
    /// [<c>-speed S</c>] [<c>-minimap</c>] resets once after connecting, submits the goal and runs.
    /// </summary>
    public sealed class ViewerOptions
    {
        public string Host { get; private set; } = "localhost";
        public int Port { get; private set; } = 7700;
        public string JournalDir { get; private set; }
        /// <summary>
        /// The quality level; Balanced by default, the level that held the 30 FPS target on the MacBook Air M4 at
        /// 1920 x 1080 with a screen recorder (High Fidelity: p99 34.0 ms; Balanced: 30.0 ms; plan 4A Task 7).
        /// </summary>
        public string Quality { get; private set; } = "Balanced";
        public string FrameTimes { get; private set; }
        public string Scenario { get; private set; }
        public int Seed { get; private set; } = 1;
        public (double x, double y, double yaw)? Goal { get; private set; }
        public double Speed { get; private set; } = 1.0;
        public bool Minimap { get; private set; }

        /// <summary>A frame-rate cap (<c>-fps N</c>, N &gt; 0); 0 renders at the display's refresh (vsync).</summary>
        public int Fps { get; private set; }

        /// <summary><c>-fps 30</c>: lock to every second display refresh (vsync), the steady 30 (4B-1 design 7.5).</summary>
        public bool VsyncLock30 => Fps == 30;

        /// <summary>
        /// <c>-overlays 1,4</c>: exactly these overlays on after the start preset (1-6 as their keys; "none" for none),
        /// for measuring one overlay's cost (4B-1 Task 6); null when not given.
        /// </summary>
        public IReadOnlyList<Overlay> Overlays { get; private set; }

        /// <summary>The overlay preset at start (<c>-preset off|demo|diagnose</c>, default off).</summary>
        public Preset StartPreset { get; private set; } = Preset.Off;

        /// <summary>Parses the arguments (the first is the program); throws ArgumentException naming the option.</summary>
        public static ViewerOptions Parse(string[] args)
        {
            var o = new ViewerOptions();
            for (var i = 1; i < args.Length; ++i)
            {
                string Value()
                {
                    if (i + 1 >= args.Length) throw new ArgumentException($"{args[i]} needs a value");
                    return args[++i];
                }
                switch (args[i])
                {
                    case "-host": o.Host = Value(); break;
                    case "-port": o.Port = Integer("-port", Value()); break;
                    case "-journal-dir": o.JournalDir = Value(); break;
                    case "-quality": o.Quality = Value(); break;
                    case "-frame-times": o.FrameTimes = Value(); break;
                    case "-scenario": o.Scenario = Value(); break;
                    case "-seed": o.Seed = Integer("-seed", Value()); break;
                    case "-speed": o.Speed = Number("-speed", Value()); break;
                    case "-minimap": o.Minimap = true; break;
                    case "-overlays":
                        var list = new List<Overlay>();
                        var spec = Value();
                        if (spec != "none")
                            foreach (var part in spec.Split(','))
                            {
                                var k = Integer("-overlays", part.Trim());
                                if (k < 1 || k > 6) throw new ArgumentException($"-overlays: {k} is not an overlay (1-6)");
                                list.Add((Overlay)(k - 1));
                            }
                        o.Overlays = list;
                        break;
                    case "-preset":
                        var name = Value();
                        if (!Enum.TryParse(name, true, out Preset preset) || int.TryParse(name, out _))
                            throw new ArgumentException($"-preset: '{name}' is not off, demo or diagnose");
                        o.StartPreset = preset;
                        break;
                    case "-fps":
                        o.Fps = Integer("-fps", Value());
                        if (o.Fps <= 0) throw new ArgumentException("-fps takes a positive frame rate");
                        break;
                    case "-goal":
                        var parts = Value().Split(',');
                        if (parts.Length != 3) throw new ArgumentException("-goal takes x,y,yaw (m, m, rad)");
                        o.Goal = (Number("-goal", parts[0]), Number("-goal", parts[1]), Number("-goal", parts[2]));
                        break;
                }
            }
            if (o.Goal.HasValue && o.Scenario == null)
                throw new ArgumentException("-goal needs -scenario: the scripted demo resets to it first");
            return o;
        }

        static int Integer(string option, string s) =>
            int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out var v)
                ? v : throw new ArgumentException($"{option}: '{s}' is not an integer");

        static double Number(string option, string s) =>
            double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) && !double.IsNaN(v) && !double.IsInfinity(v)
                ? v : throw new ArgumentException($"{option}: '{s}' is not a number");
    }
}
