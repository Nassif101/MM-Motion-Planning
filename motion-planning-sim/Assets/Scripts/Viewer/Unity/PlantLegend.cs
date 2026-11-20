using System.Collections.Generic;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>The overlay legend (4B-1 design 7.4): each overlay's key, colour, state and notes; IMGUI, bottom left.</summary>
    public sealed class PlantLegend : MonoBehaviour
    {
        public bool Visible;
        public string Title = "";
        public readonly List<(string key, string name, Color colour, bool on, string note)> Rows =
            new List<(string, string, Color, bool, string)>();

        private GUIStyle label;

        void OnGUI()
        {
            if (!Visible) return;
            label ??= new GUIStyle(GUI.skin.label) { fontSize = 12, richText = false };
            var height = 28 + 20 * Rows.Count;
            var area = new Rect(8, Screen.height - height - 8, 430, height);
            GUI.Box(area, GUIContent.none);
            GUI.Label(new Rect(area.x + 8, area.y + 4, area.width - 16, 20), Title, label);
            for (var i = 0; i < Rows.Count; ++i)
            {
                var (key, name, colour, on, note) = Rows[i];
                var y = area.y + 24 + 20 * i;
                var tint = GUI.color;
                GUI.color = on ? colour : new Color(colour.r, colour.g, colour.b, 0.3f);
                GUI.DrawTexture(new Rect(area.x + 8, y + 5, 14, 10), Texture2D.whiteTexture);
                GUI.color = tint;
                GUI.Label(new Rect(area.x + 28, y, area.width - 36, 20),
                    $"{key}  {name}  {(on ? "on" : "off")}{(string.IsNullOrEmpty(note) ? "" : "  (" + note + ")")}", label);
            }
        }
    }
}
