using UnityEngine;

namespace MotionPlanningSim.Environment
{
    // An axis-aligned box in the ROS map frame (FLU, metres).
    public readonly struct RosBox
    {
        public RosBox(Vector3 center, Vector3 size)
        {
            Center = center;
            Size = size;
        }

        public Vector3 Center { get; }
        public Vector3 Size { get; }
    }

    // 3D collision boxes for the MoveIt planning scene, from the same navigation colliders
    // as the Nav2 map (ROS x = Unity z, y = -Unity x, z = Unity y).
    public static class PlanningBoxGeometry
    {
        public static RosBox UnityBoundsToRos(Bounds unityWorldBounds)
        {
            var c = unityWorldBounds.center;
            var s = unityWorldBounds.size;
            return new RosBox(new Vector3(c.z, -c.x, c.y), new Vector3(s.z, s.x, s.y));
        }
    }
}
