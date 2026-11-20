using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MotionPlanningSim.Viewer.Core;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// The diagnostic console (4B-1 design 7.3), IMGUI, bottom of the screen: severity and subsystem filters, a text
    /// search, pause autoscroll, copy to the clipboard and export to the session's journal folder. The filtered list
    /// is cached by the log's version and the filter settings, not rebuilt per IMGUI event.
    /// </summary>
    public sealed class PlantConsolePanel : MonoBehaviour
    {
        public const string SearchControl = "ConsoleSearch";
        static readonly string[] Subsystems = { "goal", "route", "medium", "b5", "plant", "stream", "viewer" };

        public bool Visible;
        public ConsoleLog Log;

        /// <summary>The folder exports go to (the session's journal folder), or null.</summary>
        public Func<string> ExportFolder;

        /// <summary>Called with the export's path or error, for the console itself and the journal.</summary>
        public Action<string> Exported;

        private Severity minimum = Severity.Info;
        private readonly HashSet<string> shown = new HashSet<string>(Subsystems);
        private string search = "";
        private bool autoscroll = true;
        private Vector2 scroll;
        private List<ConsoleEntry> filtered = new List<ConsoleEntry>();
        private (int version, Severity minimum, int subsystems, string search) cached = (-1, Severity.Info, 0, null);
        private GUIStyle line, error, warning;

        /// <summary>The console's search field has keyboard focus: hotkeys must not fire.</summary>
        public bool Typing => Visible && GUI.GetNameOfFocusedControl() == SearchControl;

        void OnGUI()
        {
            if (!Visible || Log == null) return;
            line ??= new GUIStyle(GUI.skin.label) { fontSize = 12, wordWrap = false, richText = false };
            warning ??= new GUIStyle(line) { normal = { textColor = new Color(1f, 0.8f, 0.3f) } };
            error ??= new GUIStyle(line) { normal = { textColor = new Color(1f, 0.4f, 0.35f) } };
            var height = Mathf.Min(320, Screen.height / 2);
            GUILayout.BeginArea(new Rect(8, Screen.height - height - 8, Screen.width - 16, height), GUI.skin.box);

            GUILayout.BeginHorizontal();
            foreach (Severity s in Enum.GetValues(typeof(Severity)))
                if (GUILayout.Toggle(minimum == s, s.ToString(), GUI.skin.button, GUILayout.Width(70))) minimum = s;
            GUILayout.Space(8);
            foreach (var sub in Subsystems)
            {
                var on = GUILayout.Toggle(shown.Contains(sub), sub, GUI.skin.button, GUILayout.Width(62));
                if (on) shown.Add(sub);
                else shown.Remove(sub);
            }
            GUILayout.Space(8);
            GUI.SetNextControlName(SearchControl);
            search = GUILayout.TextField(search ?? "", GUILayout.Width(180));
            autoscroll = GUILayout.Toggle(autoscroll, "autoscroll", GUILayout.Width(90));
            if (GUILayout.Button("Copy", GUILayout.Width(50))) GUIUtility.systemCopyBuffer = string.Join("\n", filtered.Select(ConsoleLog.Format));
            if (GUILayout.Button("Export", GUILayout.Width(60))) Export();
            GUILayout.EndHorizontal();
            GUILayout.Label($"{filtered.Count} shown of {Log.Entries.Count} entries, {Log.Dropped} dropped", line);

            Refilter();
            scroll = GUILayout.BeginScrollView(scroll);
            foreach (var e in filtered)
                GUILayout.Label(ConsoleLog.Format(e), e.Severity == Severity.Error ? error : e.Severity == Severity.Warning ? warning : line);
            GUILayout.EndScrollView();
            if (autoscroll && Event.current.type == EventType.Repaint) scroll.y = float.MaxValue;
            GUILayout.EndArea();
        }

        void Refilter()
        {
            var mask = 0;
            for (var i = 0; i < Subsystems.Length; ++i)
                if (shown.Contains(Subsystems[i])) mask |= 1 << i;
            var key = (Log.Version, minimum, mask, search);
            if (key == cached) return;
            cached = key;
            filtered = Log.Filter(minimum, new HashSet<string>(shown), search).ToList();
        }

        void Export()
        {
            try
            {
                var folder = ExportFolder?.Invoke() ?? Application.temporaryCachePath;
                Directory.CreateDirectory(folder);
                var path = Path.Combine(folder, $"console-{DateTime.UtcNow:yyyyMMdd'T'HHmmss'Z'}.txt");
                using (var w = new StreamWriter(path, false)) ConsoleLog.Export(w, filtered);
                Exported?.Invoke(path);
            }
            catch (Exception e)
            {
                Exported?.Invoke("export failed: " + e.Message);
            }
        }
    }
}
