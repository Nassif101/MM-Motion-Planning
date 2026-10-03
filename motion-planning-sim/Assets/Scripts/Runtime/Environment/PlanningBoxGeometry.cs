using UnityEngine;

namespace MotionPlanningSim.Environment
{
    // A box in the ROS map frame (FLU, metres): centre, full size along its own axes, and the
    // yaw of those axes about ROS +z (radians).
    public readonly struct RosBox
    {
        public RosBox(Vector3 center, Vector3 size, float yaw = 0.0f)
        {
            Center = center;
            Size = size;
            Yaw = yaw;
        }

        public Vector3 Center { get; }
        public Vector3 Size { get; }
        public float Yaw { get; }
    }

    // 3D collision boxes for the MoveIt planning scene, from the same navigation colliders
    // as the Nav2 map (ROS x = Unity z, y = -Unity x, z = Unity y).
    public static class PlanningBoxGeometry
    {
        private const float MaxTiltDegrees = 0.01f;

        // World-axis bounds: exact for axis-aligned colliders, conservative otherwise.
        public static RosBox UnityBoundsToRos(Bounds unityWorldBounds)
        {
            var c = unityWorldBounds.center;
            var s = unityWorldBounds.size;
            return new RosBox(new Vector3(c.z, -c.x, c.y), new Vector3(s.z, s.x, s.y));
        }

        // A box collider rotated only about Unity +Y (vertical) as an oriented ROS box. Unity's
        // left-handed yaw turns forward (+z, ROS +x) toward right (+x, ROS -y), so the ROS yaw is
        // its negative. Returns false for tilted boxes, which keep their world bounds.
        public static bool TryUnityOrientedBoxToRos(Vector3 worldCenter, Vector3 worldSize, Quaternion rotation,
            out RosBox box)
        {
            box = default;
            if (Vector3.Angle(rotation * Vector3.up, Vector3.up) > MaxTiltDegrees)
                return false;
            var forward = rotation * Vector3.forward;
            var unityYaw = Mathf.Atan2(forward.x, forward.z);
            var c = worldCenter;
            var s = worldSize;
            box = new RosBox(new Vector3(c.z, -c.x, c.y), new Vector3(s.z, s.x, s.y), -unityYaw);
            return true;
        }
    }
}
