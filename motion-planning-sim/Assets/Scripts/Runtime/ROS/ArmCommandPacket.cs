using System;
using RosMessageTypes.Trajectory;

namespace MotionPlanningSim.ROS
{
    /// <summary>/arm/command (ADR 0014): one trajectory point with the JTC's commanded positions, velocities and
    /// accelerations, stamped with the controller's simulation time.</summary>
    public static class ArmCommandPacket
    {
        public static (string[] names, double[] q, double[] v, double[] a, double stamp) Read(JointTrajectoryMsg msg)
        {
            if (msg?.points == null || msg.points.Length != 1)
                throw new ArgumentException("An arm command must hold exactly one trajectory point.");
            var point = msg.points[0];
            return (msg.joint_names, point.positions, point.velocities, point.accelerations,
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);
        }
    }
}
