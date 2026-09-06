using System;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace MotionPlanningSim.Control
{
    [DefaultExecutionOrder(100), DisallowMultipleComponent]
    public sealed class ArmTrackingRecorder : MonoBehaviour
    {
        [SerializeField] private ArmActuatorController actuator;
        [SerializeField] private ArticulationBody baseBody;
        [SerializeField] private string startupRecordingPath = "";
        [SerializeField] private BoxCollider payload;
        private readonly Collider[] overlaps = new Collider[64];
        private Collider[] robotColliders;
        private StreamWriter writer;
        private void Start() { if (!string.IsNullOrEmpty(startupRecordingPath)) Begin(startupRecordingPath); }
        private double previousV, previousW;
        public string OutputPath {get; private set;}
        public void Configure(ArmActuatorController arm, ArticulationBody chassis, BoxCollider panel=null) {actuator=arm; baseBody=chassis; payload=panel;}
        public void Begin(string path)
        {
            if (!Application.isPlaying) throw new InvalidOperationException("Recording requires Play mode.");
            End(); OutputPath=path; Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path)));
            writer=new StreamWriter(path);
            robotColliders=actuator.GetComponentsInChildren<Collider>();
            writer.WriteLine("sim_time,wall_time,state,joint,desired_position,actual_position,position_error,desired_velocity,actual_velocity,velocity_error,command_age,base_velocity,base_yaw_rate,base_acceleration,base_yaw_acceleration,drive_effort,estimated_saturated,panel_bottom,panel_width_world_x,panel_length_world_z,base_tilt_degrees,panel_penetration_m,overlap_buffer_full,base_x,base_z,physics_ticks,accepted_commands,rejected_commands,robot_min_x,robot_max_x,robot_min_z,robot_max_z");
        }
        public void End() {writer?.Dispose(); writer=null;}
        private void FixedUpdate()
        {
            if(actuator==null || baseBody==null || actuator.State==ArmActuatorState.INITIALIZING || actuator.State==ArmActuatorState.FAULT) return;
            double v=Vector3.Dot(baseBody.linearVelocity,baseBody.transform.forward);
            double w=-Vector3.Dot(baseBody.angularVelocity,baseBody.transform.up);
            double a=(v-previousV)/Time.fixedDeltaTime, alpha=(w-previousW)/Time.fixedDeltaTime;
            previousV=v; previousW=w;
            if(writer==null) return;
            double penetration=0; bool full=false;
            if(payload!=null && payload.enabled)
            {
                var transform=payload.transform;
                int count=Physics.OverlapBoxNonAlloc(transform.TransformPoint(payload.center),
                    Vector3.Scale(payload.size*0.5f,transform.lossyScale),overlaps,transform.rotation,~0,QueryTriggerInteraction.Ignore);
                full=count==overlaps.Length;
                for(int k=0;k<count;++k)
                {
                    var other=overlaps[k];
                    if(other==payload) continue;
                    if(Physics.ComputePenetration(payload,transform.position,transform.rotation,
                        other,other.transform.position,other.transform.rotation,out _,out float distance))
                        penetration=Math.Max(penetration,distance);
                }
            }
            var bounds=payload!=null && payload.enabled ? payload.bounds : new Bounds();
            var robotBounds=new Bounds(baseBody.transform.position,Vector3.zero);
            foreach(var collider in robotColliders)
                if(collider!=null && collider.enabled && !collider.isTrigger && collider.gameObject.activeInHierarchy)
                    robotBounds.Encapsulate(collider.bounds);
            for(int i=0;i<6;++i)
            {
                var j=actuator.Joints[i]; double q=actuator.Position(i), qv=actuator.Velocity(i);
                // Unity driveForce is the solver drive contribution, not net joint reaction torque.
                double effort=j.body.driveForce[0]*j.sign;
                writer.WriteLine(string.Join(",",new[] {
                    F(Time.fixedTimeAsDouble),F(ArmActuatorController.WallTime),actuator.State.ToString(),j.jointName,
                    F(actuator.DesiredPosition[i]),F(q),F(actuator.DesiredPosition[i]-q),
                    F(actuator.DesiredVelocity[i]),F(qv),F(actuator.DesiredVelocity[i]-qv),F(actuator.CommandAge),
                    F(v),F(w),F(a),F(alpha),F(effort),Math.Abs(effort)>=j.torqueLimit*0.99 ? "1":"0",
                    F(bounds.min.y),F(bounds.size.x),F(bounds.size.z),F(Vector3.Angle(baseBody.transform.up,Vector3.up)),
                    F(penetration),full ? "1":"0",F(baseBody.transform.position.x),F(baseBody.transform.position.z),
                    F(actuator.PhysicsTicks),F(actuator.AcceptedPackets),F(actuator.RejectedPackets),
                    F(robotBounds.min.x),F(robotBounds.max.x),F(robotBounds.min.z),F(robotBounds.max.z)}));
            }
        }
        private static string F(double value)=>value.ToString("R",CultureInfo.InvariantCulture);
        private void OnDisable()=>End();
    }
}
