using System;
using MotionPlanningSim.Control;
using RosMessageTypes.Sensor;
using RosMessageTypes.Trajectory;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    [DefaultExecutionOrder(-50), DisallowMultipleComponent]
    public sealed class ArmRosTransport : MonoBehaviour
    {
        [SerializeField] private ArmActuatorController actuator;
        [SerializeField] private ROSConnection connection;
        [SerializeField] private string commandTopic = "/arm/command";
        [SerializeField] private string stateTopic = "/arm/state";
        private readonly object gate = new object();
        private JointTrajectoryMsg pending;
        private double receipt;
        private bool subscribed;
        private bool quitting;
        public long PublishedStates { get; private set; }
        public void Configure(ArmActuatorController controller, ROSConnection ros) {actuator=controller; connection=ros;}
        private void OnEnable()
        {
            if (actuator==null || connection==null) {Debug.LogError("Arm transport requires explicit actuator and ROSConnection references.",this); return;}
            connection.RegisterPublisher<JointStateMsg>(stateTopic, queue_size: 1);
            connection.Subscribe<JointTrajectoryMsg>(commandTopic, Receive);
            subscribed=true;
        }
        private void Receive(JointTrajectoryMsg msg)
        {
            // Single newest packet; no Unity APIs here. Message fields are never mutated.
            lock(gate) {pending=msg; receipt=ArmActuatorController.WallTime;}
        }
        private void FixedUpdate()
        {
            if (!subscribed || actuator.State==ArmActuatorState.INITIALIZING || actuator.State==ArmActuatorState.FAULT) return;
            JointTrajectoryMsg msg; double received;
            lock(gate) {msg=pending; received=receipt; pending=null;}
            if (msg!=null)
            {
                // A malformed packet reaches Accept as nulls: it is rejected and counted there.
                (string[] names, double[] q, double[] v, double[] a, double stamp) c;
                try {c=ArmCommandPacket.Read(msg);} catch (ArgumentException) {c=(null,null,null,null,double.NaN);}
                actuator.Accept(c.names,c.q,c.v,c.a,c.stamp,received);
            }
            var names=new string[6]; var q=new double[6]; var v=new double[6];
            for(int i=0;i<6;++i) {names[i]=actuator.Joints[i].jointName; q[i]=actuator.Position(i); v[i]=actuator.Velocity(i);}
            connection.Publish(stateTopic,new JointStateMsg(RosTimeUtility.Header(RosTimeUtility.PhysicsTimeSeconds),names,q,v,Array.Empty<double>()));
            ++PublishedStates;
        }
        private void OnDisable()
        {
            if (subscribed && connection!=null) connection.Unsubscribe(commandTopic);
            subscribed=false;
            lock(gate) pending=null;
            if (!quitting && actuator!=null) actuator.CaptureHold();
        }
        private void OnApplicationQuit() { quitting=true; }
    }
}
