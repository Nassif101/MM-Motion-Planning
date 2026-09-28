using System;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    /// <summary>
    /// Converts planar ROS map poses (REP-103: +X forward, +Y left, yaw CCW about +Z) to
    /// Unity world poses. The Unity world origin is the ROS <c>map</c> origin (ADR 0001):
    /// ROS X = Unity Z, ROS Y = -Unity X.
    /// </summary>
    public static class RosMapPose
    {
        public static Vector3 ToUnityPosition(double rosX, double rosY, float unityHeight)
        {
            if (!double.IsFinite(rosX) || !double.IsFinite(rosY) || !float.IsFinite(unityHeight))
            {
                throw new ArgumentOutOfRangeException(nameof(rosX), "ROS map pose must be finite.");
            }

            return new Vector3((float)-rosY, unityHeight, (float)rosX);
        }

        /// <summary>Unity rotation whose forward (+Z) axis points along ROS heading <paramref name="rosYaw"/>.</summary>
        public static Quaternion ToUnityRotation(double rosYaw)
        {
            if (!double.IsFinite(rosYaw))
            {
                throw new ArgumentOutOfRangeException(nameof(rosYaw), "ROS yaw must be finite.");
            }

            // Unity yaw is clockwise about +Y viewed from above; ROS yaw is counter-clockwise.
            return Quaternion.Euler(0.0f, (float)(-rosYaw * 180.0 / Math.PI), 0.0f);
        }
    }
}
