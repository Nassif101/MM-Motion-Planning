using System.Collections.Generic;
using System.Linq;
using MotionPlanningSim.ROS;
using MotionPlanningSim.Viewer.Core;
using UnityEngine;
using Num = System.Numerics;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// F2's plan as overlays 3-5 (4B-1 design 7.1): the plan path, translucent arm-and-panel ghosts with a base outline
    /// at evenly spaced plan times (4B-2 display samples; evenly spaced samples from an older server), and the panel's
    /// planned sweep through the display samples (F2's samples, approximate, from an older server). A hidden probe copy of the view robot,
    /// posed at each sample, gives the panel's corners and the ghosts' poses. Everything is rebuilt only when the plan
    /// or the toggles change; hidden overlays do no work (design 7.5).
    /// </summary>
    public sealed class PlantPlanOverlay : MonoBehaviour
    {
        public const int MaxGhosts = 12;
        const float LineHeight = 0.05f;

        public PlantRobotView Robot;
        public Material LineMaterial;
        public Material GhostMaterial;

        [SerializeField] private int ghostCount = 6;

        private PlantRobotView probe;
        private Transform probePanel;
        private readonly List<GameObject> ghosts = new List<GameObject>();
        private readonly List<LineRenderer> outlines = new List<LineRenderer>();
        private readonly List<Material> ghostMaterials = new List<Material>();
        private LineRenderer planLine;
        private MeshFilter sweepFilter;
        private MeshRenderer sweepRenderer;
        private Mesh sweepMesh;
        private Material sweepMaterial;
        private MaterialPropertyBlock block;
        private Vector3[] baseOutline;
        private (long plan, bool stale, bool path, bool ghosts, bool sweep) shown = (-1, false, false, false, false);

        /// <summary>The panel's planned sweep as last built (map frame, ROS); empty when hidden.</summary>
        public IReadOnlyList<Num.Vector3[]> SweepRings { get; private set; } = new List<Num.Vector3[]>();

        public int GhostCount
        {
            get => ghostCount;
            set
            {
                ghostCount = ClampGhosts(value);
                shown.plan = -1;  // rebuild
            }
        }

        public static int ClampGhosts(int n) => Mathf.Clamp(n, 1, MaxGhosts);

        /// <summary>tool0/PayloadPanel of a view robot.</summary>
        public static Transform FindPanel(PlantRobotView robot) =>
            robot.PayloadAnchor != null ? robot.PayloadAnchor.Find("PayloadPanel") : null;

        /// <summary>
        /// The panel's four corners on its mid-plane (map frame, ROS) with <paramref name="probe"/> posed at a plan
        /// sample (x, y, yaw, six joints). The panel's thin axis is found from its mesh bounds.
        /// </summary>
        public static Num.Vector3[] PanelCorners(PlantRobotView probe, Transform panel, double[] sample)
        {
            var joints = new double[6];
            for (var j = 0; j < 6; ++j) joints[j] = sample[3 + j];
            probe.Apply(new RobotSample(0, sample[0], sample[1], sample[2], 0, 0, joints), 0f);
            var bounds = panel.GetComponent<MeshFilter>().sharedMesh.bounds;
            var scale = panel.lossyScale;  // the panel is a scaled unit mesh: its thin axis is in the scale
            var size = Vector3.Scale(bounds.size, new Vector3(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z)));
            int thin = size.x <= size.y && size.x <= size.z ? 0 : size.y <= size.z ? 1 : 2;
            int a = thin == 0 ? 1 : 0, b = thin == 2 ? 1 : 2;
            var corners = new Num.Vector3[4];
            var signs = new[] { (1f, 1f), (-1f, 1f), (-1f, -1f), (1f, -1f) };
            for (var k = 0; k < 4; ++k)
            {
                var local = bounds.center;
                local[a] += signs[k].Item1 * bounds.extents[a];
                local[b] += signs[k].Item2 * bounds.extents[b];
                var w = panel.TransformPoint(local);
                corners[k] = new Num.Vector3(w.z, -w.x, w.y);  // Unity -> ROS map
            }
            return corners;
        }

        void Awake()
        {
            block = new MaterialPropertyBlock();
            probe = MakeCopy("PlanProbe");
            foreach (var r in probe.GetComponentsInChildren<Renderer>(true)) r.enabled = false;
            probePanel = FindPanel(probe);
            baseOutline = BaseOutline(Robot);
            planLine = ViewerLines.Create(transform, "PlanPath", LineMaterial, 0.06f);
            var sweep = new GameObject("PanelSweep");
            sweep.transform.SetParent(transform, false);
            sweepFilter = sweep.AddComponent<MeshFilter>();
            sweepRenderer = sweep.AddComponent<MeshRenderer>();
            sweepMesh = new Mesh { name = "PanelSweep", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            sweepFilter.sharedMesh = sweepMesh;
            sweepMaterial = new Material(GhostMaterial);
            SetColor(sweepMaterial, new Color(1f, 0.55f, 0.1f, 0.22f));
            sweepRenderer.sharedMaterial = sweepMaterial;
            sweepRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            sweepRenderer.receiveShadows = false;
        }

        PlantRobotView MakeCopy(string name) => Copy(Robot, transform, name);

        internal static PlantRobotView Copy(PlantRobotView robot, Transform parent, string name)
        {
            var copy = Instantiate(robot.gameObject, parent);
            copy.name = name;
            copy.SetActive(true);
            return copy.GetComponent<PlantRobotView>();
        }

        /// <summary>
        /// A ghost (4B-1, shared with 4B-2's prediction): a copy of <paramref name="robot"/> whose arm and panel are drawn
        /// in its own translucent material, the base as an outline in the robot's local frame.
        /// </summary>
        internal static (GameObject ghost, Material material, LineRenderer outline) MakeGhost(PlantRobotView robot,
            Transform parent, string name, Material ghostMaterial, Material lineMaterial, Vector3[] baseOutline)
        {
            var copy = Copy(robot, parent, name);
            var arm = ArmRoot(copy);
            var material = new Material(ghostMaterial);
            foreach (var r in copy.GetComponentsInChildren<Renderer>(true))
            {
                if (arm == null || !r.transform.IsChildOf(arm))
                {
                    r.enabled = false;  // the base is drawn as an outline
                    continue;
                }
                r.sharedMaterials = Enumerable.Repeat(material, r.sharedMaterials.Length).ToArray();
                r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                r.receiveShadows = false;
            }
            var outline = ViewerLines.Create(copy.transform, "BaseOutline", lineMaterial, 0.03f, loop: true);
            outline.useWorldSpace = false;  // follows the ghost; positions in the robot's local frame
            outline.alignment = LineAlignment.View;
            outline.transform.localRotation = Quaternion.identity;
            outline.positionCount = baseOutline.Length;
            outline.SetPositions(baseOutline);
            return (copy.gameObject, material, outline);
        }

        /// <summary>Poses a ghost at a plan state (x, y, yaw, six joints).</summary>
        internal static void Pose(GameObject ghost, double[] s)
        {
            var joints = new double[6];
            for (var j = 0; j < 6; ++j) joints[j] = s[3 + j];
            ghost.GetComponent<PlantRobotView>().Apply(new RobotSample(0, s[0], s[1], s[2], 0, 0, joints), 0f);
        }

        /// <summary>
        /// The states F2's ghosts show, nearest first and the plan's end last: <paramref name="count"/> evenly spaced
        /// plan times from the display samples (4B-2), or up to <paramref name="count"/> evenly spaced samples after
        /// the first without them (4B-1).
        /// </summary>
        public static List<double[]> GhostStates(MediumPlan plan, int count)
        {
            var states = new List<double[]>();
            if (plan == null || count <= 0) return states;
            if (plan.HasDisplaySamples && plan.DisplayHorizonS > 0.0)
            {
                for (var g = 1; g <= count; ++g) states.Add(plan.StateAt(g * plan.DisplayHorizonS / count));
                return Distinct(states);
            }
            var samples = plan.Samples;
            var drawn = Mathf.Min(count, Mathf.Max(0, samples.Count - 1));
            for (var g = 0; g < drawn; ++g)
                states.Add(samples[Mathf.RoundToInt((g + 1) * (samples.Count - 1) / (float)drawn)]);  // evenly spaced
            return Distinct(states);
        }

        /// <summary>
        /// Drops a ghost that would sit on the one before it (spec 4C, a 4B-2 deferred item: a plan that stands still
        /// stacked six ghosts on one spot): within 0.05 m and 0.05 rad of base motion and 0.05 rad of every joint. The
        /// plan's end stays last, replacing the ghost before it when the two coincide.
        /// </summary>
        static List<double[]> Distinct(List<double[]> states)
        {
            var kept = new List<double[]>(states.Count);
            foreach (var s in states)
            {
                if (kept.Count > 0 && Near(kept[kept.Count - 1], s))
                {
                    if (ReferenceEquals(s, states[states.Count - 1])) kept[kept.Count - 1] = s;
                    continue;
                }
                kept.Add(s);
            }
            return kept;
        }

        static bool Near(double[] a, double[] b)
        {
            if (System.Math.Sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1])) > 0.05) return false;
            if (System.Math.Abs(System.Math.IEEERemainder(a[2] - b[2], 2 * System.Math.PI)) > 0.05) return false;
            for (var j = 3; j < 9; ++j)
                if (System.Math.Abs(a[j] - b[j]) > 0.05) return false;
            return true;
        }

        /// <summary>The base's footprint rectangle in the robot's local frame, from its renderers off the arm.</summary>
        internal static Vector3[] BaseOutline(PlantRobotView robot)
        {
            var arm = ArmRoot(robot);
            var b = new Bounds();
            var any = false;
            var inv = robot.transform.worldToLocalMatrix;
            foreach (var r in robot.GetComponentsInChildren<MeshRenderer>(true))
            {
                if (arm != null && r.transform.IsChildOf(arm)) continue;
                var mf = r.GetComponent<MeshFilter>();
                if (mf == null || mf.sharedMesh == null) continue;
                foreach (var c in Corners(mf.sharedMesh.bounds))
                {
                    var p = inv.MultiplyPoint3x4(r.transform.TransformPoint(c));
                    if (!any) b = new Bounds(p, Vector3.zero);
                    else b.Encapsulate(p);
                    any = true;
                }
            }
            if (!any) b = new Bounds(Vector3.zero, new Vector3(1f, 0f, 1.4f));
            return new[]
            {
                new Vector3(b.min.x, LineHeight, b.min.z), new Vector3(b.max.x, LineHeight, b.min.z),
                new Vector3(b.max.x, LineHeight, b.max.z), new Vector3(b.min.x, LineHeight, b.max.z),
            };
        }

        static IEnumerable<Vector3> Corners(Bounds b)
        {
            for (var i = 0; i < 8; ++i)
                yield return new Vector3(i % 2 == 0 ? b.min.x : b.max.x, (i / 2) % 2 == 0 ? b.min.y : b.max.y, i < 4 ? b.min.z : b.max.z);
        }

        /// <summary>The first arm link (the shoulder pan joint's body): everything under it is arm and panel.</summary>
        static Transform ArmRoot(PlantRobotView robot) =>
            robot.GetComponentsInChildren<ViewJoint>(true).FirstOrDefault(j => j.JointName == "shoulder_pan_joint")?.transform;

        GameObject Ghost(int i)
        {
            while (ghosts.Count <= i)
            {
                var (ghost, material, outline) = MakeGhost(Robot, transform, "PlanGhost" + ghosts.Count, GhostMaterial, LineMaterial, baseOutline);
                ghostMaterials.Add(material);
                outlines.Add(outline);
                ghosts.Add(ghost);
            }
            return ghosts[i];
        }

        /// <summary>Draws or hides the plan overlays; rebuilds only when the plan, its staleness or the toggles change.</summary>
        public void Present(MediumPlan plan, bool stale, bool showPath, bool showGhosts, bool showSweep)
        {
            var key = (plan?.PlanId ?? -1, stale, showPath && plan != null, showGhosts && plan != null, showSweep && plan != null);
            if (key == shown) return;
            shown = key;
            var samples = plan?.Samples ?? (IReadOnlyList<double[]>)new List<double[]>();

            planLine.enabled = key.Item3;
            if (key.Item3)
            {
                planLine.positionCount = samples.Count;
                for (var i = 0; i < samples.Count; ++i)
                    planLine.SetPosition(i, RosMapPose.ToUnityPosition(samples[i][0], samples[i][1], LineHeight));
                ViewerLines.SetColor(planLine, PlanColor(plan.Status, stale), block);
            }

            var states = key.Item4 ? GhostStates(plan, ghostCount) : new List<double[]>();
            var drawn = states.Count;
            for (var g = 0; g < Mathf.Max(drawn, ghosts.Count); ++g)
            {
                if (g >= drawn)
                {
                    ghosts[g].SetActive(false);
                    continue;
                }
                var ghost = Ghost(g);
                ghost.SetActive(true);
                Pose(ghost, states[g]);
                var fade = 1f - 0.6f * g / Mathf.Max(1, drawn);  // nearer ghosts are stronger
                var colour = stale ? new Color(0.6f, 0.6f, 0.6f, 0.18f * fade) : new Color(0.35f, 0.75f, 1f, 0.35f * fade);
                SetColor(ghostMaterials[g], colour);
                ViewerLines.SetColor(outlines[g], colour, block);
            }

            sweepRenderer.enabled = key.Item5;
            if (key.Item5)
            {
                SweepRings = PanelSweep.Rings(plan.SweepSamples, s => PanelCorners(probe, probePanel, s));
                BuildSweep(SweepRings);
                SetColor(sweepMaterial, stale ? new Color(0.6f, 0.6f, 0.6f, 0.12f) : new Color(1f, 0.55f, 0.1f, 0.22f));
            }
            else SweepRings = new List<Num.Vector3[]>();
        }

        static Color PlanColor(string status, bool stale) =>
            stale ? new Color(0.55f, 0.55f, 0.55f)
            : status == "ok" ? new Color(0.35f, 0.75f, 1f)
            : status == "stale_reused" ? new Color(1f, 0.85f, 0.3f)
            : new Color(1f, 0.35f, 0.3f);  // infeasible

        /// <summary>A two-sided band through the four corner tracks: one quad per panel edge between rings.</summary>
        void BuildSweep(IReadOnlyList<Num.Vector3[]> rings)
        {
            sweepMesh.Clear();
            if (rings.Count < 2) return;
            var vertices = new Vector3[rings.Count * 4];
            for (var i = 0; i < rings.Count; ++i)
                for (var k = 0; k < 4; ++k)
                {
                    var p = rings[i][k];
                    vertices[i * 4 + k] = new Vector3(-p.Y, p.Z, p.X);  // ROS map -> Unity
                }
            var triangles = new int[(rings.Count - 1) * 4 * 12];
            var t = 0;
            for (var i = 0; i + 1 < rings.Count; ++i)
                for (var k = 0; k < 4; ++k)
                {
                    int a = i * 4 + k, b = i * 4 + (k + 1) % 4, c = a + 4, d = b + 4;
                    foreach (var v in new[] { a, b, d, a, d, c, a, d, b, a, c, d }) triangles[t++] = v;  // both faces
                }
            sweepMesh.vertices = vertices;
            sweepMesh.triangles = triangles;
            sweepMesh.RecalculateBounds();
        }

        static void SetColor(Material m, Color c)
        {
            m.SetColor("_UnlitColor", c);
            m.SetColor("_Color", c);
        }

        void OnDestroy()
        {
            foreach (var m in ghostMaterials) Destroy(m);
            if (sweepMaterial != null) Destroy(sweepMaterial);
            if (sweepMesh != null) Destroy(sweepMesh);
        }
    }
}
