using System.Collections.Generic;
using MotionPlanningSim.ROS;
using MotionPlanningSim.Viewer.Core;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>The accepted route as a ground line (design 6.5); empty when its goal ends or no route is drawn.</summary>
    public sealed class PlantRoutePresenter : MonoBehaviour
    {
        const float Height = 0.03f;

        public Material LineMaterial;

        private LineRenderer line;
        private MaterialPropertyBlock block;  // created in Awake: not during deserialization
        private readonly List<Vector3> points = new List<Vector3>();
        private long shownRoute = -1;
        private int shownCount = -1;

        void Awake()
        {
            block = new MaterialPropertyBlock();
            line = ViewerLines.Create(transform, "RouteLine", LineMaterial, 0.07f);
            ViewerLines.SetColor(line, new Color(0.3f, 0.95f, 0.95f), block);
        }

        public void Present(ViewerState state, bool visible = true)
        {
            line.enabled = visible;  // overlay 1 (4B-1): the 4A route line behind a toggle
            if (!visible) return;
            var path = state.RoutePath;
            if (state.RouteId == shownRoute && path.Count == shownCount) return;
            shownRoute = state.RouteId;
            shownCount = path.Count;
            points.Clear();
            foreach (var (x, y, _) in path) points.Add(RosMapPose.ToUnityPosition(x, y, Height));
            line.positionCount = points.Count;
            line.SetPositions(points.ToArray());
        }
    }
}
