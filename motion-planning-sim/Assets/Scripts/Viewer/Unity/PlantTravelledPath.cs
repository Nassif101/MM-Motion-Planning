using System.Collections.Generic;
using MotionPlanningSim.ROS;
using MotionPlanningSim.Viewer.Core;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>The travelled path (4B-1 overlay 2): the displayed base as a ground line; redrawn only when it changed.</summary>
    public sealed class PlantTravelledPath : MonoBehaviour
    {
        const float Height = 0.025f;

        public Material LineMaterial;

        private LineRenderer line;
        private MaterialPropertyBlock block;
        private readonly List<Vector3> points = new List<Vector3>();
        private int shownVersion = -1;

        void Awake()
        {
            block = new MaterialPropertyBlock();
            line = ViewerLines.Create(transform, "TravelledPath", LineMaterial, 0.05f);
            ViewerLines.SetColor(line, new Color(0.95f, 0.95f, 0.95f), block);
        }

        public void Present(PathHistory path, bool visible)
        {
            line.enabled = visible && path.Points.Count > 1;
            if (!line.enabled || path.Version == shownVersion) return;
            shownVersion = path.Version;
            points.Clear();
            foreach (var (x, y) in path.Points) points.Add(RosMapPose.ToUnityPosition(x, y, Height));
            line.positionCount = points.Count;
            line.SetPositions(points.ToArray());
        }
    }
}
