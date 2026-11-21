using System.Collections.Generic;
using MotionPlanningSim.ROS;
using MotionPlanningSim.Viewer.Core;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// B5's prediction as overlay 7 (spec 4B-2): the predicted base path of B5's solved 2 s plan, and arm-and-panel
    /// ghosts at 0.5, 1, 1.5 and 2 s, in violet (F2's plan is blue). A plan B5 does not execute (it holds or brakes) is
    /// drawn in red; a prediction more than 1 s old in grey. Rebuilt only when a new prediction arrives (10 Hz) or the
    /// toggle changes; hidden, it does no work.
    /// </summary>
    public sealed class PlantPredictionOverlay : MonoBehaviour
    {
        const float LineHeight = 0.08f;  // above F2's plan path (0.05 m)

        public static readonly double[] GhostTimes = { 0.5, 1.0, 1.5, 2.0 };

        public PlantRobotView Robot;
        public Material LineMaterial;
        public Material GhostMaterial;

        private readonly List<GameObject> ghosts = new List<GameObject>();
        private readonly List<LineRenderer> outlines = new List<LineRenderer>();
        private readonly List<Material> ghostMaterials = new List<Material>();
        private LineRenderer pathLine;
        private MaterialPropertyBlock block;
        private Vector3[] baseOutline;
        private B5Prediction shownPrediction;
        private bool shownStale;
        private bool shownVisible;

        public static Color Colour(bool held, bool stale) =>
            stale ? new Color(0.55f, 0.55f, 0.55f) : held ? new Color(1f, 0.25f, 0.2f) : new Color(0.75f, 0.45f, 1f);

        /// <summary>The states at <see cref="GhostTimes"/> within the prediction's horizon (none without one).</summary>
        public static List<double[]> GhostStates(B5Prediction prediction)
        {
            var states = new List<double[]>();
            if (prediction == null || prediction.States.Count == 0) return states;
            var horizon = prediction.Stages * prediction.StageDt;
            foreach (var t in GhostTimes)
                if (t <= horizon + 1e-6) states.Add(prediction.StateAt(t));
            return states;
        }

        void Awake()
        {
            block = new MaterialPropertyBlock();
            baseOutline = PlantPlanOverlay.BaseOutline(Robot);
            pathLine = ViewerLines.Create(transform, "PredictionPath", LineMaterial, 0.05f);
            pathLine.enabled = false;
        }

        /// <summary>Draws or hides the prediction; rebuilds only when the prediction, its staleness or the toggle change.</summary>
        public void Present(B5Prediction prediction, bool stale, bool visible)
        {
            visible = visible && prediction != null;
            if (ReferenceEquals(prediction, shownPrediction) && stale == shownStale && visible == shownVisible) return;
            shownPrediction = prediction;
            shownStale = stale;
            shownVisible = visible;

            pathLine.enabled = visible;
            var states = visible ? GhostStates(prediction) : new List<double[]>();
            for (var g = states.Count; g < ghosts.Count; ++g) ghosts[g].SetActive(false);
            if (!visible) return;

            var colour = Colour(prediction.Held, stale);
            pathLine.positionCount = prediction.States.Count;
            for (var i = 0; i < prediction.States.Count; ++i)
                pathLine.SetPosition(i, RosMapPose.ToUnityPosition(prediction.States[i][0], prediction.States[i][1], LineHeight));
            ViewerLines.SetColor(pathLine, colour, block);

            for (var g = 0; g < states.Count; ++g)
            {
                while (ghosts.Count <= g)
                {
                    var (ghost, material, outline) = PlantPlanOverlay.MakeGhost(Robot, transform, "PredictionGhost" + ghosts.Count,
                        GhostMaterial, LineMaterial, baseOutline);
                    ghosts.Add(ghost);
                    ghostMaterials.Add(material);
                    outlines.Add(outline);
                }
                ghosts[g].SetActive(true);
                PlantPlanOverlay.Pose(ghosts[g], states[g]);
                var fade = 1f - 0.5f * g / GhostTimes.Length;  // nearer ghosts are stronger
                var c = new Color(colour.r, colour.g, colour.b, (stale ? 0.18f : 0.35f) * fade);
                ghostMaterials[g].SetColor("_UnlitColor", c);
                ghostMaterials[g].SetColor("_Color", c);
                ViewerLines.SetColor(outlines[g], c, block);
            }
        }

        void OnDestroy()
        {
            foreach (var m in ghostMaterials) Destroy(m);
        }
    }
}
