using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// Where the IMGUI panels go, so that open panels never overlap (Task 5 review I2: a click on the telemetry passed
    /// through to the minimap and set a goal). The minimap is top right; the console fills the bottom left of it; the
    /// telemetry and the legend sit side by side below the status line, above the console.
    /// </summary>
    public static class PanelLayout
    {
        const float Top = 140f, Gap = 8f, TelemetryWidth = 440f, LegendWidth = 300f;

        public static Rect Minimap(float w, float h, int mapWidth, int mapHeight)
        {
            var width = Mathf.Min(w * 0.36f, 760f);
            var height = width * mapHeight / mapWidth;
            var room = h - 136f - 60f;
            if (height > room)
            {
                height = Mathf.Max(80f, room);
                width = height * mapWidth / mapHeight;
            }
            return new Rect(w - width - 16, 136, width + 8, height + 52);
        }

        /// <param name="rightLimit">The minimap's left edge while it is open, else the screen width.</param>
        public static Rect Console(float w, float h, float rightLimit)
        {
            var height = Mathf.Min(320f, h / 3f);
            return new Rect(Gap, h - height - Gap, Mathf.Max(200f, rightLimit - 2 * Gap), height);
        }

        /// <summary>The telemetry and the legend fit side by side left of <paramref name="rightLimit"/>.</summary>
        static bool SideBySide(float rightLimit) => rightLimit - Gap >= Gap + TelemetryWidth + Gap + LegendWidth;

        /// <param name="bottomLimit">The console's top while it is open, else the screen height.</param>
        /// <param name="rightLimit">The minimap's left edge while it is open, else the screen width.</param>
        public static Rect Telemetry(float w, float h, float bottomLimit, float rightLimit)
        {
            var room = Mathf.Max(60f, bottomLimit - Top - Gap);
            return SideBySide(rightLimit)
                ? new Rect(Gap, Top, TelemetryWidth, room)
                : new Rect(Gap, Top, Mathf.Min(TelemetryWidth, rightLimit - 2 * Gap), Mathf.Max(40f, room * 0.6f - Gap));
        }

        /// <summary>Beside the telemetry when it fits; else stacked below it, in the left column.</summary>
        public static Rect Legend(float w, float h, int rows, float bottomLimit, float rightLimit)
        {
            var wanted = 28f + 18f * rows;
            var room = Mathf.Max(60f, bottomLimit - Top - Gap);
            if (SideBySide(rightLimit)) return new Rect(Gap + TelemetryWidth + Gap, Top, LegendWidth, Mathf.Min(wanted, room));
            var top = Top + room * 0.6f;
            return new Rect(Gap, top, Mathf.Min(LegendWidth, rightLimit - 2 * Gap), Mathf.Max(20f, Mathf.Min(wanted, bottomLimit - top - Gap)));
        }
    }
}
