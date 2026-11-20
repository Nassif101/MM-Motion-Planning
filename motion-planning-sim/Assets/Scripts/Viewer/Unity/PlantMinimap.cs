using System.Collections.Generic;
using MotionPlanningSim.ROS;
using MotionPlanningSim.Viewer.Core;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// The minimap overlay (design 6.5), drawn with IMGUI: a top-down render of the viewer scene made once at
    /// startup, the routing map's occupancy over it at 40%, the robot, the route and the goal marker. A left press
    /// sets a destination and a drag its heading; the gestures go to the bootstrap through <see cref="Gestures"/>,
    /// which submits, journals and draws nothing itself. Shown while <see cref="Open"/>.
    /// </summary>
    public sealed class PlantMinimap : MonoBehaviour
    {
        const float OccupancyOpacity = 0.4f;
        const int WarmUpRenders = 16;

        public bool Open;
        public bool GoalsEnabled;
        public string Note = "";
        public (double x, double y, double yaw)? Robot;
        public GoalMarkerModel Goal;
        public IReadOnlyList<(double x, double y, double yaw)> Route;

        /// <summary>Press, drag and release since the last read, in map coordinates (m).</summary>
        public readonly List<(string kind, double x, double y)> Gestures = new List<(string, double, double)>();

        /// <summary>The map point under the mouse while it is over the open minimap.</summary>
        public (double x, double y)? Cursor { get; private set; }

        private MapFrame map;
        private Texture2D occupancy;
        private RenderTexture topDown;
        private Rect mapRect;
        private (double x, double y)? press, drag;
        private GUIStyle label, warning;

        public void Initialize(MapFrame frame)
        {
            map = frame;
            occupancy = new Texture2D(map.Width, map.Height, TextureFormat.RGBA32, false) { filterMode = FilterMode.Point };
            var pixels = new Color32[map.Width * map.Height];
            for (var row = 0; row < map.Height; ++row)  // the texture's row 0 is the bottom; the PGM's is the top
            for (var col = 0; col < map.Width; ++col)
            {
                var v = map.Pixels[(map.Height - 1 - row) * map.Width + col];
                pixels[row * map.Width + col] = v < 100 ? new Color32(0, 0, 0, 255)
                    : v < 250 ? new Color32(90, 90, 90, 160) : new Color32(0, 0, 0, 0);
            }
            occupancy.SetPixels32(pixels);
            occupancy.Apply(false, true);
        }

        /// <summary>One orthographic top-down render of the scene over the map's extent (design 6.5).</summary>
        public void RenderTopDown()
        {
            if (map == null) return;
            var go = new GameObject("MinimapCamera");
            try
            {
                var cam = go.AddComponent<Camera>();
                cam.enabled = false;
                cam.orthographic = true;
                cam.orthographicSize = (float)(map.Height * map.Resolution / 2);
                cam.aspect = map.Width / (float)map.Height;
                cam.nearClipPlane = 0.5f;
                cam.farClipPlane = 200f;
                var center = RosMapPose.ToUnityPosition(map.OriginX + map.Width * map.Resolution / 2,
                    map.OriginY + map.Height * map.Resolution / 2, 80f);
                // looking down; image up is map +y (Unity -x), image right is map +x (Unity +z)
                cam.transform.SetPositionAndRotation(center, Quaternion.Euler(90f, -90f, 0f));
                topDown = new RenderTexture(map.Width, map.Height, 24, RenderTextureFormat.ARGB32) { name = "MinimapTopDown" };
                topDown.Create();
                // A new HDRP camera starts with no exposure or temporal history: its first frame is nearly black and
                // blurred. Measured live: the image settles within 10 renders.
                cam.targetTexture = topDown;
                for (var k = 0; k < WarmUpRenders; ++k) cam.Render();
                cam.targetTexture = null;
            }
            finally
            {
                Destroy(go);
            }
        }

        void OnGUI()
        {
            if (!Open || map == null)
            {
                Cursor = null;
                press = null;
                return;
            }
            label ??= new GUIStyle(GUI.skin.label) { fontSize = 13, wordWrap = true };
            warning ??= new GUIStyle(label) { normal = { textColor = new Color(1f, 0.4f, 0.35f) } };
            var width = Mathf.Min(Screen.width * 0.42f, 760f);
            var height = width * map.Height / map.Width;
            var room = Screen.height - 136f - 60f;
            if (height > room)
            {
                height = Mathf.Max(80f, room);
                width = height * map.Width / map.Height;
            }
            var panel = new Rect(Screen.width - width - 16, 136, width + 8, height + 52);
            GUI.Box(panel, GUIContent.none);
            mapRect = new Rect(panel.x + 4, panel.y + 4, width, height);
            HandleMouse(Event.current);
            if (Event.current.type == EventType.Repaint) Draw();
            var text = new Rect(panel.x + 6, mapRect.yMax + 4, width - 4, 44);
            if (!GoalsEnabled) GUI.Label(text, "Goals disabled: " + Note, warning);
            else GUI.Label(text, GoalLine(), Goal != null && Goal.State == MarkerState.Rejected ? warning : label);
        }

        string GoalLine()
        {
            if (Goal == null || Goal.State == MarkerState.None) return "Click to set a destination; drag to set its heading.";
            var reason = string.IsNullOrEmpty(Goal.Reason) ? "" : ": " + Goal.Reason;
            return $"Goal {Goal.State}{reason}";
        }

        void HandleMouse(Event e)
        {
            var over = mapRect.Contains(e.mousePosition);
            if (e.type == EventType.MouseMove || e.type == EventType.MouseDrag || e.type == EventType.Repaint)
                Cursor = over ? ToWorld(e.mousePosition) : ((double, double)?)null;
            if (!GoalsEnabled || e.button != 0) return;
            switch (e.type)
            {
                case EventType.MouseDown when over:
                    press = drag = ToWorld(e.mousePosition);
                    Gestures.Add(("press", press.Value.x, press.Value.y));
                    e.Use();
                    break;
                case EventType.MouseDrag when press.HasValue:
                    drag = ToWorld(e.mousePosition);
                    Gestures.Add(("drag", drag.Value.x, drag.Value.y));
                    e.Use();
                    break;
                case EventType.MouseUp when press.HasValue:
                    var release = ToWorld(e.mousePosition);  // also off the panel: the drag only sets a heading
                    Gestures.Add(("release", release.x, release.y));
                    press = drag = null;
                    e.Use();
                    break;
            }
        }

        void Draw()
        {
            if (topDown != null) GUI.DrawTexture(mapRect, topDown, ScaleMode.StretchToFill, false);
            else GUI.DrawTexture(mapRect, Texture2D.grayTexture);
            var tint = GUI.color;
            GUI.color = new Color(1f, 1f, 1f, OccupancyOpacity);
            GUI.DrawTexture(mapRect, occupancy, ScaleMode.StretchToFill, true);
            GUI.color = tint;

            if (Route != null)
                for (var i = 1; i < Route.Count; ++i)
                    Line(ToGui(Route[i - 1].x, Route[i - 1].y), ToGui(Route[i].x, Route[i].y), new Color(0.3f, 0.95f, 0.95f), 2f);
            if (Goal?.Target is { } g && Goal.State != MarkerState.None)
                Pose(g.x, g.y, g.yaw, PlantGoalPresenter.ColorOf(Goal.State), 8f);
            if (Robot is { } r) Pose(r.x, r.y, r.yaw, new Color(1f, 0.6f, 0.1f), 10f);
            if (press is { } p && drag is { } d)
            {
                var robot = Robot.HasValue ? (Robot.Value.x, Robot.Value.y) : p;
                var t = MinimapGesture.Target(p, d, robot);
                Pose(t.x, t.y, t.yaw, Color.white, 8f);
            }
        }

        void Pose(double x, double y, double yaw, Color color, float size)
        {
            var at = ToGui(x, y);
            var tip = at + new Vector2(Mathf.Cos((float)yaw), -Mathf.Sin((float)yaw)) * size * 1.8f;  // GUI y is down
            Line(at, tip, color, 2f);
            Dot(at, size, color);
        }

        static void Dot(Vector2 at, float size, Color color)
        {
            var tint = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(new Rect(at.x - size / 2, at.y - size / 2, size, size), Texture2D.whiteTexture);
            GUI.color = tint;
        }

        static void Line(Vector2 a, Vector2 b, Color color, float width)
        {
            var d = b - a;
            if (d.sqrMagnitude < 1e-4f) return;
            var matrix = GUI.matrix;
            var tint = GUI.color;
            GUI.color = color;
            GUIUtility.RotateAroundPivot(Mathf.Atan2(d.y, d.x) * Mathf.Rad2Deg, a);
            GUI.DrawTexture(new Rect(a.x, a.y - width / 2, d.magnitude, width), Texture2D.whiteTexture);
            GUI.matrix = matrix;
            GUI.color = tint;
        }

        Vector2 ToGui(double x, double y)
        {
            var (px, py) = map.WorldToPixel(x, y);
            return new Vector2(mapRect.x + (float)(px / map.Width) * mapRect.width,
                mapRect.y + (float)(py / map.Height) * mapRect.height);
        }

        (double x, double y) ToWorld(Vector2 gui) =>
            map.PixelToWorld((gui.x - mapRect.x) / mapRect.width * map.Width, (gui.y - mapRect.y) / mapRect.height * map.Height);

        void OnDestroy()
        {
            if (topDown != null) topDown.Release();
            if (occupancy != null) Destroy(occupancy);
        }
    }
}
