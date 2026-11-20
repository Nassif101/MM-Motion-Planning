using System.Collections.Generic;
using MotionPlanningSim.ROS;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// The session's scenario objects (design 6.1), drawn as plain boxes from the backend: Hello <c>boxes</c>
    /// (except those the site model already has), movers from State <c>movers</c>, and reserved bodies from State
    /// <c>bodies</c> (hidden until placed). No colliders: the boxes are display only.
    /// </summary>
    public sealed class PlantEnvironmentView : MonoBehaviour
    {
        public PlantViewerSite Site;

        private Transform boxRoot, moverRoot, bodyRoot;
        private readonly Dictionary<string, (Transform t, Vector3 size)> movers = new Dictionary<string, (Transform, Vector3)>();
        private readonly Dictionary<string, (Transform t, Vector3 size)> bodies = new Dictionary<string, (Transform, Vector3)>();

        public void ApplySession(JObject hello)
        {
            Clear();
            if (hello["boxes"] is JArray boxes)
                foreach (var b in boxes)
                {
                    var name = (string)b["name"];
                    if (Site != null && Site.HasSiteGeom("site/" + name)) continue;
                    var c = b["center"];
                    var box = Box(boxRoot, name, Size(b["size"]));
                    box.SetPositionAndRotation(RosMapPose.ToUnityPosition((double)c[0], (double)c[1], (float)c[2]),
                        RosMapPose.ToUnityRotation((double?)b["yaw"] ?? 0.0));
                }
            Parked(hello["movers"], moverRoot, movers);
            Parked(hello["reserved"], bodyRoot, bodies);
        }

        public void ApplyState(ViewerState state)
        {
            foreach (var (name, x, y) in state.Movers)
                if (movers.TryGetValue(name, out var m)) Place(m, x, y, 0.0);
            foreach (var (name, x, y, yaw) in state.Bodies)
                if (bodies.TryGetValue(name, out var b)) Place(b, x, y, yaw);
        }

        private static void Place((Transform t, Vector3 size) item, double x, double y, double yaw)
        {
            item.t.gameObject.SetActive(true);
            item.t.SetPositionAndRotation(RosMapPose.ToUnityPosition(x, y, item.size.y / 2), RosMapPose.ToUnityRotation(yaw));
        }

        private static void Parked(JToken list, Transform root, Dictionary<string, (Transform, Vector3)> into)
        {
            if (!(list is JArray items)) return;
            foreach (var item in items)
            {
                var name = (string)item["name"];
                var size = Size(item["size"]);
                var t = Box(root, name, size);
                t.gameObject.SetActive(false);
                into[name] = (t, size);
            }
        }

        // ROS size (x forward, y left, z up) as a Unity scale (x right, y up, z forward).
        private static Vector3 Size(JToken s) => new Vector3((float)s[1], (float)s[2], (float)s[0]);

        private static Transform Box(Transform parent, string name, Vector3 unitySize)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Object.DestroyImmediate(go.GetComponent<Collider>());
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localScale = unitySize;
            return go.transform;
        }

        private void Clear()
        {
            movers.Clear();
            bodies.Clear();
            for (var i = transform.childCount - 1; i >= 0; --i) Object.DestroyImmediate(transform.GetChild(i).gameObject);
            boxRoot = Root("box");
            moverRoot = Root("mover");
            bodyRoot = Root("body");
        }

        private Transform Root(string name)
        {
            var t = new GameObject(name).transform;
            t.SetParent(transform, false);
            return t;
        }
    }
}
