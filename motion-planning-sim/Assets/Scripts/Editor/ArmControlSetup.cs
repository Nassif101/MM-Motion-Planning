using System;
using System.Globalization;
using System.Linq;
using System.Xml.Linq;
using MotionPlanningSim.Control;
using MotionPlanningSim.ROS;
using Unity.Pipeline.Commands;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    public static class ArmControlSetup
    {
        private static readonly float[] Stiffness={2000,6000,3500,500,650,300};
        private static readonly float[] Damping={240,360,180,40,35,28};
        [CliCommand("setup_arm_control", "Wire finite arm actuators and restore gravity in the active scene", MainThreadRequired=true)]
        public static string Setup()
        {
            if(Application.isPlaying) throw new InvalidOperationException("Exit Play before authoring arm setup.");
            var root=GameObject.Find("MobileManipulator");
            if(root==null) throw new InvalidOperationException("MobileManipulator scene root is missing.");
            var connections=UnityEngine.Object.FindObjectsByType<ROSConnection>(FindObjectsInactive.Include,FindObjectsSortMode.None);
            if(connections.Length!=1) throw new InvalidOperationException("Scene requires one explicit ROSConnection.");
            ConfigureRobot(root,connections[0]);
            EditorSceneManager.MarkSceneDirty(root.scene);
            EditorSceneManager.SaveScene(root.scene);
            return "Arm controller wired; gravity enabled throughout arm subtree; existing base controller retained.";
        }
        public static void ConfigureRobot(GameObject root, ROSConnection connection)
        {
            var old=root.GetComponent<ArmJointHoldController>();
            if(old!=null) UnityEngine.Object.DestroyImmediate(old);
            var model=XDocument.Load("Assets/Robots/MobileManipulator/urdf/mobile_manipulator.urdf");
            var urdf=model.Root.Elements("joint").Where(j=>(string)j.Attribute("type")=="revolute").ToArray();
            if(urdf.Length!=6) throw new InvalidOperationException("Expected six URDF arm joints.");
            var imported=root.GetComponentsInChildren<UrdfJoint>(true).ToDictionary(j=>j.jointName);
            var config=new ArmJointActuator[6];
            for(int i=0;i<6;++i)
            {
                var j=urdf[i]; var name=(string)j.Attribute("name"); var limit=j.Element("limit");
                var body=imported[name].GetComponent<ArticulationBody>();
                config[i]=new ArmJointActuator {jointName=name,body=body,sign=1,
                    lower=Number(limit,"lower"),upper=Number(limit,"upper"),maximumVelocity=Number(limit,"velocity"),
                    torqueLimit=Number(limit,"effort"),stiffness=Stiffness[i],damping=Damping[i]};
                imported[name].EffortLimit=config[i].torqueLimit;
                imported[name].VelocityLimit=config[i].maximumVelocity;
                // Generic body drag is not active drive damping. Preserve URDF joint friction.
                body.angularDamping=0.05f;
                var drive=body.xDrive;
                drive.driveType=ArticulationDriveType.Force;
                drive.stiffness=Stiffness[i]; drive.damping=Damping[i]; drive.forceLimit=config[i].torqueLimit;
                body.xDrive=drive;
                body.maxJointVelocity=config[i].maximumVelocity;
            }
            foreach(var body in imported["arm_mount_joint"].GetComponentsInChildren<ArticulationBody>(true))
                body.useGravity=true;
            foreach(var body in root.GetComponentsInChildren<ArticulationBody>(true))
            { body.solverIterations=12; body.solverVelocityIterations=4; }
            var arm=root.GetComponent<ArmActuatorController>() ?? root.AddComponent<ArmActuatorController>();
            arm.Configure(config);
            var transport=root.GetComponent<ArmRosTransport>() ?? root.AddComponent<ArmRosTransport>();
            transport.Configure(arm,connection);
            var recorder=root.GetComponent<ArmTrackingRecorder>() ?? root.AddComponent<ArmTrackingRecorder>();
            recorder.Configure(arm,root.GetComponentsInChildren<ArticulationBody>().Single(b=>b.name=="base_link"),
                root.GetComponentsInChildren<BoxCollider>(true).Single(c=>c.name=="PayloadPanel"));
            EditorUtility.SetDirty(root);
        }
        private static float Number(XElement element,string attribute)=>float.Parse((string)element.Attribute(attribute),CultureInfo.InvariantCulture);
    }
}
