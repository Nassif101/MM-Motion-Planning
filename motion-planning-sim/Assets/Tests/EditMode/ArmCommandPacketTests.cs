using System;
using MotionPlanningSim.ROS;
using NUnit.Framework;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Trajectory;
namespace MotionPlanningSim.Tests
{
    public class ArmCommandPacketTests
    {
        static JointTrajectoryMsg Packet(params JointTrajectoryPointMsg[] points) =>
            new JointTrajectoryMsg(new HeaderMsg(new TimeMsg(12, 340000000), ""), new[]{"a","b"}, points);
        static JointTrajectoryPointMsg Point() =>
            new JointTrajectoryPointMsg(new[]{0.1,0.2}, new[]{0.3,0.4}, new[]{0.5,0.6}, Array.Empty<double>(), new DurationMsg());

        [Test] public void ReadsTheSinglePoint()
        {
            var c = ArmCommandPacket.Read(Packet(Point()));
            Assert.That(c.names, Is.EqualTo(new[]{"a","b"}));
            Assert.That(c.q, Is.EqualTo(new[]{0.1,0.2}));
            Assert.That(c.v, Is.EqualTo(new[]{0.3,0.4}));
            Assert.That(c.a, Is.EqualTo(new[]{0.5,0.6}));
            Assert.That(c.stamp, Is.EqualTo(12.34).Within(1e-9));
        }
        [Test] public void RejectsAnythingButOnePoint()
        {
            Assert.Throws<ArgumentException>(()=>ArmCommandPacket.Read(Packet()));
            Assert.Throws<ArgumentException>(()=>ArmCommandPacket.Read(Packet(Point(), Point())));
        }
    }
}
