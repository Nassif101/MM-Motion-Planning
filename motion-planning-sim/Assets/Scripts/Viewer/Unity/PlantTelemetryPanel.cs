using System.Collections.Generic;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// The telemetry panel (4B-1 design 7.2), IMGUI, top right. It draws the lines the bootstrap gives it; the
    /// bootstrap builds them only while the panel is open.
    /// </summary>
    public sealed class PlantTelemetryPanel : MonoBehaviour
    {
        public bool Visible;
        public readonly List<string> Lines = new List<string>();

        private GUIStyle label;
        private Vector2 scroll;

        void OnGUI()
        {
            if (!Visible) return;
            label ??= new GUIStyle(GUI.skin.label) { fontSize = 12, wordWrap = false, richText = false };
            var height = Mathf.Min(Screen.height - 150, 24 + 17 * Lines.Count);
            var area = new Rect(Screen.width - 468, 140, 460, height);
            GUILayout.BeginArea(area, GUI.skin.box);
            scroll = GUILayout.BeginScrollView(scroll);
            foreach (var line in Lines) GUILayout.Label(line, label);
            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }
    }
}
