using MotionPlanningSim.Viewer.Core;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>4B-1 Task 3: overlay toggles and presets (design 7.4).</summary>
    public sealed class OverlaySettingsTests
    {
        // 4B-2: B5's prediction (overlay 7) is a diagnosis view
        [Test]
        public void PresetsIncludePredictionInDiagnoseOnly()
        {
            var o = new OverlaySettings();
            Assert.IsFalse(o[Overlay.Prediction], "Off");
            o.Apply(Preset.Demo);
            Assert.IsFalse(o[Overlay.Prediction], "Demo");
            o.Apply(Preset.Diagnose);
            Assert.IsTrue(o[Overlay.Prediction], "Diagnose");
            Assert.AreEqual(6, (int)Overlay.Prediction, "key 7");
        }

        [Test]
        public void PresetsSetTheDesignedToggles()
        {
            var o = new OverlaySettings();
            Assert.AreEqual(Preset.Off, o.Current);  // the 4A picture at start
            Assert.IsTrue(o[Overlay.Route]);
            Assert.IsTrue(o[Panel.Hud]);

            o.Toggle(Panel.Minimap);  // the user opened the minimap
            var demo = o.Apply(Preset.Demo);
            foreach (var on in new[] { Overlay.Route, Overlay.Travelled, Overlay.Ghosts, Overlay.Sweep }) Assert.IsTrue(o[on], on.ToString());
            foreach (var off in new[] { Overlay.PlanPath, Overlay.Lidar }) Assert.IsFalse(o[off], off.ToString());
            foreach (var off in new[] { Panel.Hud, Panel.Minimap, Panel.Telemetry, Panel.Console, Panel.Legend }) Assert.IsFalse(o[off], off.ToString());
            Assert.AreEqual("Demo", (string)demo["preset"]);
            Assert.AreEqual(true, (bool)demo["overlays"]["Ghosts"]);
            Assert.AreEqual(false, (bool)demo["panels"]["Hud"]);

            o.Apply(Preset.Diagnose);
            foreach (Overlay x in System.Enum.GetValues(typeof(Overlay))) Assert.IsTrue(o[x], x.ToString());
            foreach (var on in new[] { Panel.Hud, Panel.Telemetry, Panel.Console, Panel.Legend }) Assert.IsTrue(o[on], on.ToString());

            o.Toggle(Panel.Minimap);  // open
            o.Apply(Preset.Off);
            Assert.IsTrue(o[Overlay.Route]);
            foreach (var off in new[] { Overlay.Travelled, Overlay.PlanPath, Overlay.Ghosts, Overlay.Sweep, Overlay.Lidar }) Assert.IsFalse(o[off]);
            Assert.IsTrue(o[Panel.Hud]);
            Assert.IsTrue(o[Panel.Minimap], "Off leaves the minimap as it was");
            Assert.IsFalse(o[Panel.Telemetry]);
            Assert.IsFalse(o[Panel.Console]);
            Assert.AreEqual(Preset.Demo, OverlaySettings.Next(Preset.Off));
            Assert.AreEqual(Preset.Off, OverlaySettings.Next(Preset.Diagnose));
        }

        // Task 3 review I3: a start option (-minimap) sets a panel without marking the preset modified, so the settings
        // are the one source of the picture
        [Test]
        public void AStartOptionSetsAPanelWithoutModifyingThePreset()
        {
            var o = new OverlaySettings();
            o.Set(Panel.Minimap, true);
            Assert.IsTrue(o[Panel.Minimap]);
            Assert.IsFalse(o.Modified);
        }

        [Test]
        public void AToggleAfterAPresetMarksItModified()
        {
            var o = new OverlaySettings();
            o.Apply(Preset.Demo);
            Assert.IsFalse(o.Modified);
            var line = o.Toggle(Overlay.Lidar);
            Assert.IsTrue(o[Overlay.Lidar]);
            Assert.IsTrue(o.Modified);
            Assert.AreEqual("Lidar", (string)line["toggle"]);
            Assert.AreEqual(true, (bool)line["on"]);
            Assert.AreEqual(true, (bool)line["modified"]);
            o.Apply(Preset.Demo);
            Assert.IsFalse(o.Modified);
        }
    }
}
