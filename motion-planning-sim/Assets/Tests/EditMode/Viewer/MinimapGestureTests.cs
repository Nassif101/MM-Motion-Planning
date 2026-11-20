using MotionPlanningSim.Viewer.Core;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class MinimapGestureTests
    {
        [Test]
        public void DragSetsHeading()
        {
            var t = MinimapGesture.Target((0, 0), (1, 1), (5, 5));
            Assert.AreEqual((0.0, 0.0), (t.x, t.y));
            Assert.AreEqual(Mathf.PI / 4, t.yaw, 1e-6);
        }

        [Test]
        public void ClickWithoutDragUsesRobotToPointHeading()
        {
            var t = MinimapGesture.Target((2, 0), (2, 0), (0, 0));
            Assert.AreEqual((2.0, 0.0, 0.0), t);
            Assert.AreEqual(System.Math.PI / 2, MinimapGesture.Target((0, 2), (0.1, 2.1), (0, 0)).yaw, 1e-9,
                "a drag under 0.3 m is a click");
            Assert.AreEqual(0.0, MinimapGesture.Target((0, 0), (0, 0), (0, 0)).yaw, "on the robot: heading 0");
        }

        [Test]
        public void PressOutsideMapStillSubmits()
        {
            var t = MinimapGesture.Target((-30, 0), (-30, 0), (0, 0));
            Assert.AreEqual(-30.0, t.x);  // no clamping: the backend rejects it with its reason
            Assert.AreEqual(0.0, t.y);
        }
    }
}
