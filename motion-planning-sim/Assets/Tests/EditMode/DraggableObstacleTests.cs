using MotionPlanningSim.Environment;
using MotionPlanningSim.ROS;
using NUnit.Framework;
using RosMessageTypes.Std;
using RosMessageTypes.Visualization;
using UnityEngine;

namespace MotionPlanningSim.Tests
{
    public class DraggableObstacleTests
    {
        private GameObject box;

        [TearDown]
        public void TearDown() => Object.DestroyImmediate(box);

        private BoxCollider Cube(Vector3 position, Quaternion rotation, Vector3 scale)
        {
            box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.transform.SetPositionAndRotation(position, rotation);
            box.transform.localScale = scale;
            Physics.SyncTransforms();
            return box.GetComponent<BoxCollider>();
        }

        [Test]
        public void AnUprightScaledBoxIsItsRosCentreSizeAndYaw()
        {
            // Unity (x, y, z) = (1, 1.3, 2) is ROS (2, -1, 1.3); scale (x, y, z) = (0.3, 0.2, 0.5) is ROS size
            // (0.5, 0.3, 0.2); Unity yaw +30 deg is ROS yaw -30 deg.
            var ros = DraggableObstacle.ToRos(Cube(new Vector3(1, 1.3f, 2), Quaternion.Euler(0, 30, 0),
                new Vector3(0.3f, 0.2f, 0.5f)));
            Assert.That(Vector3.Distance(ros.Center, new Vector3(2, -1, 1.3f)), Is.LessThan(1e-5f));
            Assert.That(Vector3.Distance(ros.Size, new Vector3(0.5f, 0.3f, 0.2f)), Is.LessThan(1e-5f));
            Assert.That(ros.Yaw, Is.EqualTo(-30 * Mathf.Deg2Rad).Within(1e-5f));
        }

        [Test]
        public void ATiltedBoxIsItsWorldBounds()
        {
            var collider = Cube(new Vector3(0, 1, 0), Quaternion.Euler(45, 0, 0), new Vector3(0.4f, 0.4f, 0.4f));
            var ros = DraggableObstacle.ToRos(collider);
            Assert.That(ros.Yaw, Is.EqualTo(0.0f));
            Assert.That(ros.Size.x, Is.EqualTo(collider.bounds.size.z).Within(1e-5f));
            Assert.That(ros.Size.z, Is.GreaterThan(0.4f + 1e-3f));
        }

        [Test]
        public void TheMarkerIsACubeWithTheBoxsPoseAndSize()
        {
            var marker = DraggableObstaclePublisher.Marker(new RosBox(new Vector3(2, -1, 1.3f), new Vector3(0.5f, 0.3f, 0.2f),
                Mathf.PI / 2), 7, new HeaderMsg());
            Assert.That(marker.type, Is.EqualTo(MarkerMsg.CUBE));
            Assert.That(marker.action, Is.EqualTo(MarkerMsg.ADD));
            Assert.That(marker.id, Is.EqualTo(7));
            Assert.That(marker.pose.position.x, Is.EqualTo(2).Within(1e-6));
            Assert.That(marker.pose.position.z, Is.EqualTo(1.3).Within(1e-6));
            Assert.That(marker.pose.orientation.z, Is.EqualTo(Mathf.Sin(Mathf.PI / 4)).Within(1e-6));
            Assert.That(marker.scale.x, Is.EqualTo(0.5).Within(1e-6));
        }
    }
}
