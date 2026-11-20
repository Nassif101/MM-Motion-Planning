using System;
using System.Collections.Generic;
using MotionPlanningSim.Viewer.Core;
using UnityEngine;
using Num = System.Numerics;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// Lidar points (4B-1 overlay 6, approximate placement): small camera-facing squares, one submesh and colour per
    /// hit class. A scan waits until the State history covers its times (the scan can arrive before the States of its
    /// last cycle), and is skipped and counted after 1 s of simulation time. Hidden, it does no work: pending scans
    /// and the cloud are dropped, and the next scans refill it.
    /// </summary>
    public sealed class PlantLidarOverlay : MonoBehaviour
    {
        public const int Classes = 7;  // site, scenario, mover, ground, robot, panel, other
        const double GiveUpS = 1.0;
        static readonly Color[] ClassColours =
        {
            new Color(0.75f, 0.75f, 0.8f), new Color(1f, 0.6f, 0.2f), new Color(1f, 0.25f, 0.85f), new Color(0.4f, 0.4f, 0.4f),
            new Color(0.3f, 0.9f, 0.4f), new Color(1f, 0.95f, 0.2f), new Color(0.5f, 0.8f, 1f),
        };

        public PlantRobotView Robot;
        public Material LineMaterial;
        public float PointSize = 0.03f;

        private readonly LidarCloud cloud = new LidarCloud();
        private readonly List<(double tStart, double tEnd, float[] xyz, float[] hit)> pending = new List<(double, double, float[], float[])>();
        private Matrix4x4 livoxInRobot = Matrix4x4.identity;
        private Mesh mesh;
        private MeshRenderer meshRenderer;
        private Material[] materials;
        private int shownVersion = -1;
        private bool wasVisible;
        private readonly List<Vector3>[] verts = new List<Vector3>[Classes];
        private readonly List<int>[] tris = new List<int>[Classes];

        public LidarCloud Cloud => cloud;
        public static Color ColourOf(int hitClass) => ClassColours[Mathf.Clamp(hitClass, 0, Classes - 1)];

        void Awake()
        {
            var livox = FindDeep(Robot.transform, "livox_frame");
            if (livox != null) livoxInRobot = Robot.transform.worldToLocalMatrix * livox.localToWorldMatrix;
            var go = new GameObject("LidarPoints");
            go.transform.SetParent(transform, false);
            mesh = new Mesh { name = "LidarPoints", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            meshRenderer = go.AddComponent<MeshRenderer>();
            meshRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            meshRenderer.receiveShadows = false;
            materials = new Material[Classes];
            for (var c = 0; c < Classes; ++c)
            {
                materials[c] = new Material(LineMaterial);
                materials[c].SetColor("_UnlitColor", ClassColours[c]);
                verts[c] = new List<Vector3>();
                tris[c] = new List<int>();
            }
            meshRenderer.sharedMaterials = materials;
        }

        static Transform FindDeep(Transform t, string name)
        {
            if (t.name == name) return t;
            foreach (Transform child in t)
            {
                var hit = FindDeep(child, name);
                if (hit != null) return hit;
            }
            return null;
        }

        /// <summary>A point in the lidar frame (ROS) to the base frame (ROS), through the view robot's mount.</summary>
        Num.Vector3 LivoxToBase(Num.Vector3 p)
        {
            var u = livoxInRobot.MultiplyPoint3x4(new Vector3(-p.Y, p.Z, p.X));  // ROS -> Unity, then into the robot
            return new Num.Vector3(u.z, -u.x, u.y);                            // Unity -> ROS
        }

        /// <summary>Takes a Scan frame (stream-protocol.md, Scan): header t_start, t_end; blocks points, hit classes.</summary>
        public void Queue(Frame frame, bool visible)
        {
            if (!visible || frame.Blocks.Length < 2) return;
            pending.Add(((double?)frame.Header["t_start"] ?? 0, (double?)frame.Header["t_end"] ?? 0, frame.Blocks[0], frame.Blocks[1]));
            if (pending.Count > 8) pending.RemoveAt(0);
        }

        public void Clear()
        {
            pending.Clear();
            cloud.Clear();
        }

        public void Present(ViewerState state, bool visible)
        {
            if (!visible)
            {
                if (wasVisible) Clear();
                wasVisible = false;
                meshRenderer.enabled = false;
                return;
            }
            wasVisible = true;
            meshRenderer.enabled = true;
            var newest = state.NewestStateT;
            for (var i = 0; i < pending.Count;)
            {
                var (ts, te, xyz, hit) = pending[i];
                var covered = state.TryPoseAt(ts, out _) && state.TryPoseAt(te, out _);
                if (!covered && !(newest - te > GiveUpS))
                {
                    ++i;
                    continue;
                }
                cloud.AddScan(ts, te, xyz, hit, t => state.TryPoseAt(t, out var p) ? p : ((double, double, double)?)null, LivoxToBase);
                pending.RemoveAt(i);
            }
            if (cloud.Version != shownVersion) Rebuild();
        }

        void Rebuild()
        {
            shownVersion = cloud.Version;
            var cam = Camera.main != null ? Camera.main.transform : transform;
            var right = cam.right * PointSize;
            var up = cam.up * PointSize;
            for (var c = 0; c < Classes; ++c)
            {
                verts[c].Clear();
                tris[c].Clear();
            }
            foreach (var (p, hit) in cloud.Visible())
            {
                var c = Mathf.Clamp(hit, 0, Classes - 1);
                var centre = new Vector3(-p.Y, p.Z, p.X);
                var v = verts[c];
                var b = v.Count;
                v.Add(centre - right - up);
                v.Add(centre + right - up);
                v.Add(centre + right + up);
                v.Add(centre - right + up);
                var t = tris[c];
                t.Add(b); t.Add(b + 2); t.Add(b + 1); t.Add(b); t.Add(b + 3); t.Add(b + 2);
            }
            mesh.Clear();
            var all = new List<Vector3>();
            var offsets = new int[Classes];
            for (var c = 0; c < Classes; ++c)
            {
                offsets[c] = all.Count;
                all.AddRange(verts[c]);
            }
            mesh.SetVertices(all);
            mesh.subMeshCount = Classes;
            for (var c = 0; c < Classes; ++c)
            {
                var list = tris[c];
                for (var k = 0; k < list.Count; ++k) list[k] += offsets[c];
                mesh.SetTriangles(list, c, false);
            }
            mesh.RecalculateBounds();
        }

        void OnDestroy()
        {
            if (materials != null) foreach (var m in materials) Destroy(m);
            if (mesh != null) Destroy(mesh);
        }
    }
}
