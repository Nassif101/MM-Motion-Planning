using System;
using MotionPlanningSim.ROS;
using MotionPlanningSim.Viewer;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class PlantRobotViewTests
    {
        static readonly double[] Arm = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };

        static PlantRobotView NewRobot()
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(ViewerSceneContractTests.PrefabPath);
            return ((GameObject)PrefabUtility.InstantiatePrefab(prefab)).GetComponent<PlantRobotView>();
        }

        // tool0's origin in base_footprint, from the URDF alone (every joint origin has rpy 0): an independent check
        // of the recorded axes and their signs
        static Vector3 ToolInFootprint(double[] q)
        {
            var r = Matrix4x4.identity;
            var p = Vector3.zero;
            void Move(float x, float y, float z) => p += (Vector3)(r * new Vector4(x, y, z, 0));
            void Rot(Vector3 axis, double angle) => r *= RosRotation(axis, angle);
            Move(0, 0, 0.21f);           // base_link
            Move(-0.08f, 0, 0.11f);      // arm_mount_link
            Move(0, 0, 0.18f); Rot(Vector3.forward, q[0]);  // shoulder_pan about z
            Move(0, 0, 0.12f); Rot(Vector3.up, q[1]);       // shoulder_lift about y
            Move(0, 0, 0.32f); Rot(Vector3.up, q[2]);       // elbow about y
            Move(0, 0, 0.28f); Rot(Vector3.forward, q[3]);  // wrist_1 about z
            Move(0, 0, 0.10f); Rot(Vector3.up, q[4]);       // wrist_2 about y
            Move(0, 0, 0.10f); Rot(Vector3.forward, q[5]);  // wrist_3 about z
            Move(0, 0, 0.09f);           // tool0
            return p;
        }

        // A right-handed rotation about a ROS axis (x, y, z as given), as a matrix acting on ROS coordinates.
        static Matrix4x4 RosRotation(Vector3 axis, double angle)
        {
            var c = (float)Math.Cos(angle);
            var s = (float)Math.Sin(angle);
            var m = Matrix4x4.identity;
            if (axis == Vector3.forward) { m.m00 = c; m.m01 = -s; m.m10 = s; m.m11 = c; }  // about ROS z
            else { m.m00 = c; m.m02 = s; m.m20 = -s; m.m22 = c; }                            // about ROS y
            return m;
        }

        [Test]
        public void FixturePosePlacesBaseAndArm()
        {
            var robot = NewRobot();
            robot.Apply(new RobotSample(1.0, 2.0, -1.0, Math.PI / 2, 0, 0, Arm), 0f);
            var root = robot.transform;
            Assert.Less(Vector3.Distance(RosMapPose.ToUnityPosition(2.0, -1.0, 0f), root.position), 1e-5f);
            Assert.Less(Quaternion.Angle(RosMapPose.ToUnityRotation(Math.PI / 2), root.rotation), 0.01f);

            var tool = root.InverseTransformPoint(robot.PayloadAnchor.position);
            var ros = new Vector3(tool.z, -tool.x, tool.y);  // Unity (x, y, z) -> ROS (z, -x, y)
            var expected = ToolInFootprint(Arm);
            Assert.Less(Vector3.Distance(expected, ros), 1e-3f, $"tool0 at {ros}, URDF says {expected}");
        }

        [Test]
        public void WheelsSpinFromTwist()
        {
            var robot = NewRobot();
            robot.Apply(new RobotSample(0, 0, 0, 0, 0.14, 0, Arm), 1f);
            foreach (var j in robot.GetComponentsInChildren<ViewJoint>())
                if (j.JointName.EndsWith("wheel_joint", StringComparison.Ordinal)) Assert.AreEqual(1.0, j.Angle, 1e-4, j.JointName);

            robot = NewRobot();
            robot.Apply(new RobotSample(0, 0, 0, 0, 0, 0.1, Arm), 1f);
            foreach (var j in robot.GetComponentsInChildren<ViewJoint>())
            {
                if (!j.JointName.EndsWith("wheel_joint", StringComparison.Ordinal)) continue;
                var expected = j.JointName.Contains("left") ? -0.1 * 1.5 / 0.14 : 0.1 * 1.5 / 0.14;
                Assert.AreEqual(expected, j.Angle, 1e-4, j.JointName);
            }
        }

        [Test]
        public void EnvironmentDrawsScenarioBoxesAndMoversOnly()
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var site = new GameObject("site").AddComponent<PlantViewerSite>();
            site.Configure("sha256:x", new[] { "site/existing" });
            var env = new GameObject("env").AddComponent<PlantEnvironmentView>();
            env.Site = site;
            env.ApplySession(JObject.Parse(
                "{\"scenario\":\"s\",\"boxes\":[{\"name\":\"existing\",\"center\":[0,0,0.5],\"size\":[1,1,1],\"yaw\":0}," +
                "{\"name\":\"crate\",\"center\":[1,2,0.5],\"size\":[0.4,0.6,1.0],\"yaw\":0.5}]," +
                "\"movers\":[{\"name\":\"worker\",\"size\":[0.5,0.6,1.8]}],\"reserved\":[{\"name\":\"reserved/0\",\"size\":[1,1,1]}]}"));
            var crate = env.transform.Find("box/crate");
            Assert.IsNotNull(crate);
            Assert.IsNull(env.transform.Find("box/existing"), "a box the site already has is not drawn twice");
            Assert.Less(Vector3.Distance(RosMapPose.ToUnityPosition(1, 2, 0.5f), crate.position), 1e-5f);
            Assert.Less(Vector3.Distance(new Vector3(0.6f, 1.0f, 0.4f), crate.localScale), 1e-5f);  // ROS (x,y,z) size
            Assert.Less(Quaternion.Angle(RosMapPose.ToUnityRotation(0.5), crate.rotation), 0.01f);

            var state = new ViewerState();
            state.Apply(new Frame
            {
                Type = FrameType.State,
                Header = JObject.Parse("{\"t\":1.0,\"movers\":[[\"worker\",3.0,4.0]],\"bodies\":[]}"),
                Blocks = new[] { new float[9] },
            }, 0.0);
            env.ApplyState(state);
            var worker = env.transform.Find("mover/worker");
            Assert.IsTrue(worker.gameObject.activeSelf);
            Assert.Less(Vector3.Distance(RosMapPose.ToUnityPosition(3, 4, 0.9f), worker.position), 1e-5f);
            var parked = System.Linq.Enumerable.First(System.Linq.Enumerable.Cast<Transform>(env.transform.Find("body")), t => t.name == "reserved/0");
            Assert.IsFalse(parked.gameObject.activeSelf, "parked until placed");
            Assert.IsEmpty(env.GetComponentsInChildren<Collider>(true));
        }
    }
}
