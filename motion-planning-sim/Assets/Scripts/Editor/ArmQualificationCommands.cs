using System;
using System.IO;
using System.Linq;
using MotionPlanningSim.Control;
using MotionPlanningSim.ROS;
using Unity.Pipeline.Commands;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    // Compiled commands avoid Roslyn eval stalling the physics thread during measurements.
    public static class ArmQualificationCommands
    {
        private static ArmActuatorController Arm()
        {
            if(!Application.isPlaying) throw new InvalidOperationException("Enter Play before qualification.");
            return UnityEngine.Object.FindFirstObjectByType<ArmActuatorController>()
                ?? throw new InvalidOperationException("Arm missing.");
        }
        [CliCommand("arm_test_snapshot", "Read arm qualification state without runtime C# compilation", MainThreadRequired=true)]
        public static object Snapshot()
        {
            var a=Arm(); var b=a.GetComponentsInChildren<ArticulationBody>().Single(x=>x.name=="base_link");
            var c=UnityEngine.Object.FindFirstObjectByType<SimulationClockPublisher>();
            var p=a.GetComponentsInChildren<BoxCollider>().Single(x=>x.name=="PayloadPanel");
            return new {playing=Application.isPlaying,state=a.State.ToString(),age=a.CommandAge,speed=b.linearVelocity.magnitude,
                time=RosTimeUtility.PhysicsTimeSeconds,unityTime=Time.fixedTimeAsDouble,dt=Time.fixedDeltaTime,
                physicsTicks=a.PhysicsTicks,clockTicks=c.PublishedTicks,clockTime=c.LastPublishedTime,
                accepted=a.AcceptedPackets,rejected=a.RejectedPackets,
                publishedStates=a.GetComponent<ArmRosTransport>().PublishedStates,
                q=Enumerable.Range(0,6).Select(a.Position).ToArray(),
                baseX=b.transform.position.x,baseZ=b.transform.position.z,
                tilt=Vector3.Angle(b.transform.up,Vector3.up),panelBottom=p.bounds.min.y,panelWidth=p.bounds.size.x};
        }
        [CliCommand("arm_test_record", "Start a qualification CSV under the repository experiment directory", MainThreadRequired=true)]
        public static string Record(string name)
        {
            if(string.IsNullOrEmpty(name) || name.Any(c=>!char.IsLetterOrDigit(c) && c!='-'))
                throw new ArgumentException("Recording name must be alphanumeric with optional hyphens.");
            string root=Directory.GetParent(Application.dataPath).Parent.FullName;
            string path=Path.Combine(root,"docs","experiments","arm-controller","qualification",name+".csv");
            if(File.Exists(path)) throw new IOException("Recording already exists: "+name);
            Arm().GetComponent<ArmTrackingRecorder>().Begin(path);
            return name;
        }
        [CliCommand("arm_test_end", "Close the current qualification recording", MainThreadRequired=true)]
        public static string End() {Arm().GetComponent<ArmTrackingRecorder>().End(); return "closed";}

        [CliCommand("arm_test_place_gate", "Place the stopped vertical-carry robot at the gate test start in Play only", MainThreadRequired=true)]
        public static string PlaceGate()
        {
            var a=Arm(); var b=a.GetComponentsInChildren<ArticulationBody>().Single(x=>x.name=="base_link");
            double[] q={Math.PI/2,0,0,0,Math.PI/2,0};
            if(a.State!=ArmActuatorState.HOLD || a.CommandAge<0 || a.CommandAge>.5 || b.linearVelocity.magnitude>.05 ||
                Enumerable.Range(0,6).Any(i=>Math.Abs(a.Position(i)-q[i])>.04))
                throw new InvalidOperationException("Requires fresh, stopped vertical-carry HOLD.");
            b.TeleportRoot(new Vector3(7.725f,.21f,-5.3f),Quaternion.Euler(0,180,0));
            b.linearVelocity=Vector3.zero; b.angularVelocity=Vector3.zero;
            return "Gate fixture placed; saved scene unchanged.";
        }
    }
}
