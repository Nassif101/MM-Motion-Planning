using System;
using System.Collections.Generic;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// The one-line status HUD and the scenario picker (design 6.4), drawn with IMGUI. It only shows what the
    /// bootstrap gives it; a click in the picker is reported back through <see cref="PickedScenario"/>.
    /// </summary>
    public sealed class PlantViewerHud : MonoBehaviour
    {
        public string Status = "";
        public string Warning = "";
        public string Message = "";
        public bool PickerOpen;

        /// <summary>The status, warning and message lines (4B-1: H, and the presets); the picker draws regardless.</summary>
        public bool Visible = true;
        public readonly List<string> Scenarios = new List<string>();

        /// <summary>The scenario clicked in the picker since the last read, or null.</summary>
        public string PickedScenario { get; set; }

        private GUIStyle line, warning;

        void OnGUI()
        {
            line ??= new GUIStyle(GUI.skin.box) { alignment = TextAnchor.MiddleLeft, fontSize = 14, richText = false, wordWrap = true };
            warning ??= new GUIStyle(line) { normal = { textColor = new Color(1f, 0.35f, 0.3f) } };
            if (Visible)
            {
                GUILayout.BeginArea(new Rect(8, 8, Screen.width - 16, 120));
                GUILayout.Label(Status, line);
                if (!string.IsNullOrEmpty(Warning)) GUILayout.Label(Warning, warning);
                if (!string.IsNullOrEmpty(Message)) GUILayout.Label(Message, line);
                GUILayout.EndArea();
            }

            if (!PickerOpen) return;
            var height = Math.Min(Screen.height - 160, 40 + 26 * Math.Max(1, Scenarios.Count));
            GUILayout.BeginArea(new Rect(8, 136, 360, height), GUI.skin.window);
            GUILayout.Label(Scenarios.Count == 0 ? "Waiting for the scenario list..." : "Reset to scenario (seed 1):");
            foreach (var s in Scenarios)
                if (GUILayout.Button(s))
                {
                    PickedScenario = s;
                    PickerOpen = false;
                }
            GUILayout.EndArea();
        }
    }
}
