using System;
using MotionPlanningSim.Control;
using RosMessageTypes.Sensor;
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
        private JointStateMsg pending;
        private double receipt;
        private bool subscribed;
        private bool quitting;
        public long PublishedStates { get; private set; }
        public void Configure(ArmActuatorController controller, ROSConnection ros) {actuator=controller; connection=ros;}
        private void OnEnable()
        {
            if (actuator==null || connection==null) {Debug.LogError("Arm transport requires explicit actuator and ROSConnection references.",this); return;}
            connection.RegisterPublisher<JointStateMsg>(stateTopic, queue_size: 1);
            connection.Subscribe<JointStateMsg>(commandTopic, Receive);
            subscribed=true;
        }
        private void Receive(JointStateMsg msg)
        {
            // Single newest packet; no Unity APIs here. Message fields are never mutated.
            lock(gate) {pending=msg; receipt=ArmActuatorController.WallTime;}
        }
        private void FixedUpdate()
        {
            if (!subscribed || actuator.State==ArmActuatorState.INITIALIZING || actuator.State==ArmActuatorState.FAULT) return;
            JointStateMsg msg; double received;
            lock(gate) {msg=pending; received=receipt; pending=null;}
            if (msg!=null)
                actuator.Accept(msg.name,msg.position,msg.velocity,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,received);
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
