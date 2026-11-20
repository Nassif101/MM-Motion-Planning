using MotionPlanningSim.ROS;
using MotionPlanningSim.Viewer.Core;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>Line drawing shared by the scene presenters: unlit, shadowless, colour per instance.</summary>
    internal static class ViewerLines
    {
        public static LineRenderer Create(Transform parent, string name, Material material, float width, bool loop = false)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var line = go.AddComponent<LineRenderer>();
            line.sharedMaterial = material;
            line.useWorldSpace = true;
            line.loop = loop;
            line.widthMultiplier = width;
            line.positionCount = 0;
            line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            line.receiveShadows = false;
            line.numCapVertices = 2;
            line.alignment = LineAlignment.TransformZ;  // flat on the ground: the object looks down
            go.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
            return line;
        }

        public static void SetColor(LineRenderer line, Color color, MaterialPropertyBlock block)
        {
            line.GetPropertyBlock(block);
            block.SetColor("_UnlitColor", color);
            block.SetColor("_Color", color);
            line.SetPropertyBlock(block);
        }
    }

    /// <summary>
    /// The goal marker in the scene (design 6.5): a ring and a heading arrow at the target, coloured by the marker
    /// state the backend reports; hidden when there is no goal.
    /// </summary>
    public sealed class PlantGoalPresenter : MonoBehaviour
    {
        const float Height = 0.04f, Radius = 0.45f;

        public Material LineMaterial;

        private LineRenderer ring, arrow;
        private MaterialPropertyBlock block;  // created in Awake: not during deserialization
        private readonly Vector3[] ringPoints = new Vector3[32];

        public static Color ColorOf(MarkerState s) => s switch
        {
            MarkerState.Pending => new Color(0.9f, 0.9f, 0.9f),
            MarkerState.Planning => new Color(1f, 0.82f, 0.2f),
            MarkerState.Tracking => new Color(0.25f, 0.65f, 1f),
            MarkerState.Reached => new Color(0.25f, 0.85f, 0.35f),
            MarkerState.Rejected or MarkerState.Failed => new Color(1f, 0.3f, 0.25f),
            MarkerState.TimedOut => new Color(1f, 0.55f, 0.15f),
            _ => new Color(0.55f, 0.55f, 0.55f),  // superseded, canceled
        };

        void Awake()
        {
            block = new MaterialPropertyBlock();
            ring = ViewerLines.Create(transform, "GoalRing", LineMaterial, 0.06f, loop: true);
            arrow = ViewerLines.Create(transform, "GoalArrow", LineMaterial, 0.08f);
        }

        public void Present(GoalMarkerModel goal)
        {
            var target = goal.Target;
            var show = target.HasValue && goal.State != MarkerState.None;
            ring.enabled = arrow.enabled = show;
            if (!show) return;
            var (x, y, yaw) = target.Value;
            for (var i = 0; i < ringPoints.Length; ++i)
            {
                var a = 2.0 * System.Math.PI * i / ringPoints.Length;
                ringPoints[i] = RosMapPose.ToUnityPosition(x + Radius * System.Math.Cos(a), y + Radius * System.Math.Sin(a), Height);
            }
            ring.positionCount = ringPoints.Length;
            ring.SetPositions(ringPoints);
            double c = System.Math.Cos(yaw), s = System.Math.Sin(yaw);
            arrow.positionCount = 5;
            arrow.SetPositions(new[]
            {
                RosMapPose.ToUnityPosition(x, y, Height),
                RosMapPose.ToUnityPosition(x + 0.9 * c, y + 0.9 * s, Height),
                RosMapPose.ToUnityPosition(x + 0.65 * c - 0.18 * s, y + 0.65 * s + 0.18 * c, Height),
                RosMapPose.ToUnityPosition(x + 0.9 * c, y + 0.9 * s, Height),
                RosMapPose.ToUnityPosition(x + 0.65 * c + 0.18 * s, y + 0.65 * s - 0.18 * c, Height),
            });
            var color = ColorOf(goal.State);
            ViewerLines.SetColor(ring, color, block);
            ViewerLines.SetColor(arrow, color, block);
        }
    }
}
