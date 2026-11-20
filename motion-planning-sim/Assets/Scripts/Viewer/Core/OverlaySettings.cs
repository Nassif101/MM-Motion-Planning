using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>The 4B-1 overlays (design 7.1), numbered as their keys 1-6.</summary>
    public enum Overlay
    {
        Route,
        Travelled,
        PlanPath,
        Ghosts,
        Sweep,
        Lidar,
    }

    /// <summary>The viewer's panels: 4A's status line (Hud) and minimap, and 4B-1's telemetry, console and legend.</summary>
    public enum Panel
    {
        Hud,
        Minimap,
        Telemetry,
        Console,
        Legend,
    }

    public enum Preset
    {
        Off,
        Demo,
        Diagnose,
    }

    /// <summary>
    /// Overlay and panel toggles with the presets of design 7.4. <see cref="Apply"/> and the toggles return their
    /// journal fields (thinnable: no). Starts at <see cref="Preset.Off"/>, the 4A picture.
    /// </summary>
    public sealed class OverlaySettings
    {
        readonly Dictionary<Overlay, bool> overlays = new Dictionary<Overlay, bool>();
        readonly Dictionary<Panel, bool> panels = new Dictionary<Panel, bool>();

        public OverlaySettings()
        {
            foreach (Overlay o in Enum.GetValues(typeof(Overlay))) overlays[o] = false;
            foreach (Panel p in Enum.GetValues(typeof(Panel))) panels[p] = false;
            Apply(Preset.Off);
        }

        public bool this[Overlay o] => overlays[o];
        public bool this[Panel p] => panels[p];
        public Preset Current { get; private set; }
        public bool Modified { get; private set; }

        /// <summary>Incremented on every change.</summary>
        public int Version { get; private set; }

        public static Preset Next(Preset p) => p == Preset.Off ? Preset.Demo : p == Preset.Demo ? Preset.Diagnose : Preset.Off;

        public JObject Apply(Preset preset)
        {
            foreach (Overlay o in Enum.GetValues(typeof(Overlay)))
                overlays[o] = preset == Preset.Diagnose || o == Overlay.Route ||
                              (preset == Preset.Demo && (o == Overlay.Travelled || o == Overlay.Ghosts || o == Overlay.Sweep));
            panels[Panel.Hud] = preset != Preset.Demo;
            panels[Panel.Telemetry] = preset == Preset.Diagnose;
            panels[Panel.Console] = preset == Preset.Diagnose;
            panels[Panel.Legend] = preset == Preset.Diagnose;
            if (preset == Preset.Demo) panels[Panel.Minimap] = false;  // Off and Diagnose leave the minimap as it was
            Current = preset;
            Modified = false;
            ++Version;
            var line = new JObject { ["preset"] = preset.ToString(), ["overlays"] = new JObject(), ["panels"] = new JObject() };
            foreach (var kv in overlays) line["overlays"][kv.Key.ToString()] = kv.Value;
            foreach (var kv in panels) line["panels"][kv.Key.ToString()] = kv.Value;
            return line;
        }

        public JObject Toggle(Overlay o)
        {
            overlays[o] = !overlays[o];
            return Toggled(o.ToString(), overlays[o]);
        }

        public JObject Toggle(Panel p)
        {
            panels[p] = !panels[p];
            return Toggled(p.ToString(), panels[p]);
        }

        JObject Toggled(string name, bool on)
        {
            Modified = true;
            ++Version;
            return new JObject { ["toggle"] = name, ["on"] = on, ["preset"] = Current.ToString(), ["modified"] = true };
        }
    }
}
