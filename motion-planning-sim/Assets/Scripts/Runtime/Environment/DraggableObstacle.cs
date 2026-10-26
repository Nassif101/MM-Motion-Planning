using MotionPlanningSim.ROS;
using UnityEngine;

namespace MotionPlanningSim.Environment
{
    /// <summary>
    /// A box obstacle to drag in the Scene view while B5 drives (tools/demo.py drag-box). While enabled it is published
    /// with its ground-truth pose on <see cref="DraggableObstaclePublisher.Topic"/>, which only the demo's B5 reads
    /// (`draggables_topic`): a demonstration aid ("known draggables"), never part of a scenario run.
    /// </summary>
    [RequireComponent(typeof(BoxCollider))]
    [DisallowMultipleComponent]
    public sealed class DraggableObstacle : MonoBehaviour
    {
        /// <summary>The marker id, stable while the box exists (assigned when it is first enabled).</summary>
        public int Id { get; internal set; }

        /// <summary>The box in the ROS map frame: its yaw when upright, its world bounds when tilted.</summary>
        public RosBox ToRos() => ToRos(GetComponent<BoxCollider>());

        public static RosBox ToRos(BoxCollider box)
        {
            var t = box.transform;
            var scale = t.lossyScale;
            var size = Vector3.Scale(box.size, new Vector3(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z)));
            return PlanningBoxGeometry.TryUnityOrientedBoxToRos(t.TransformPoint(box.center), size, t.rotation, out var ros)
                ? ros
                : PlanningBoxGeometry.UnityBoundsToRos(box.bounds);
        }

        private void OnEnable() => DraggableObstaclePublisher.Register(this);

        private void OnDisable() => DraggableObstaclePublisher.Unregister(this);
    }
}
