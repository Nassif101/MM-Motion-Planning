using MotionPlanningSim.Environment;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests
{
    public class PlanningBoxGeometryTests
    {
        [Test]
        public void UnityBoundsMapToRosAxes()
        {
            var box = PlanningBoxGeometry.UnityBoundsToRos(
                new Bounds(new Vector3(1, 2, 3), new Vector3(0.5f, 1.0f, 2.0f)));
            Assert.That(box.Center, Is.EqualTo(new Vector3(3, -1, 2)));
            Assert.That(box.Size, Is.EqualTo(new Vector3(2.0f, 0.5f, 1.0f)));
        }
    }
}
