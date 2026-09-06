using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

namespace MotionPlanningSim.Control
{
    public enum ArmActuatorState { INITIALIZING, HOLD, EXTERNAL_CONTROL, WATCHDOG_HOLD, FAULT }

    [Serializable]
    public sealed class ArmJointActuator
    {
        public string jointName;
        public ArticulationBody body;
        public float sign = 1;
        public float lower, upper, maximumVelocity, torqueLimit, stiffness, damping;
    }

    public static class ArmCommandValidation
    {
        public static float DriveDegrees(double radians, float sign) => (float)(radians * sign * 180.0 / Math.PI);
        public static double RosRadians(float reducedCoordinate, float sign) => reducedCoordinate * sign;

        public static int[] Map(string[] required, string[] names, double[] q, double[] v)
        {
            if (names == null || q == null || v == null || names.Length != required.Length ||
                q.Length != names.Length || v.Length != names.Length)
                throw new ArgumentException("Arm packet requires all six names, positions and velocities.");
            var lookup = new Dictionary<string, int>(StringComparer.Ordinal);
            for (int i = 0; i < names.Length; ++i)
            {
                if (string.IsNullOrEmpty(names[i]) || !lookup.TryAdd(names[i], i) ||
                    !double.IsFinite(q[i]) || !double.IsFinite(v[i]))
                    throw new ArgumentException("Arm packet has duplicate/empty names or non-finite values.");
            }
            var result = new int[required.Length];
            for (int i = 0; i < required.Length; ++i)
                if (!lookup.TryGetValue(required[i], out result[i]))
                    throw new ArgumentException($"Missing arm joint {required[i]}; packet contains an unknown joint.");
            return result;
        }
    }

    /// <summary>Instantaneous physical actuator. Trajectory timing belongs entirely to ROS.</summary>
    [DisallowMultipleComponent]
    public sealed class ArmActuatorController : MonoBehaviour
    {
        [SerializeField] private ArmJointActuator[] joints = Array.Empty<ArmJointActuator>();
        [SerializeField, Min(0.05f)] private float watchdogSeconds = 0.5f;
        [SerializeField, Min(0.02f)] private float maximumPacketAgeSeconds = 0.25f;
        [SerializeField, Min(1)] private int solverIterations = 12;
        [SerializeField, Min(1)] private int solverVelocityIterations = 4;
        public ArmActuatorState State { get; private set; } = ArmActuatorState.INITIALIZING;
        public string LastError { get; private set; } = "";
        public int RejectedPackets { get; private set; }
        public long AcceptedPackets { get; private set; }
        public long PhysicsTicks { get; private set; }
        public ArmJointActuator[] Joints => joints;
        public double[] DesiredPosition { get; private set; } = new double[6];
        public double[] DesiredVelocity { get; private set; } = new double[6];
        public double CommandAge => lastReceipt == 0 ? -1 : WallTime - lastReceipt;
        public static double WallTime => (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
        private string[] names;
        private double lastReceipt, lastStamp = -1;
        private bool ready;
        private bool quitting;
        private readonly double[] holdPositions = new double[6];

        public void Configure(ArmJointActuator[] configuration) { joints = configuration; }

        private void Start()
        {
            try
            {
                ValidateConfiguration();
                // These articulation solver properties do not persist in scene serialization.
                foreach (var body in GetComponentsInChildren<ArticulationBody>())
                { body.solverIterations=solverIterations; body.solverVelocityIterations=solverVelocityIterations; }
                names = Array.ConvertAll(joints, j => j.jointName);
                ready = true;
                if (!CaptureHold(ArmActuatorState.HOLD))
                    throw new InvalidOperationException("Arm physics is unavailable during initialization.");
                ApplyDrives();
            }
            catch (Exception e) { ready = false; State = ArmActuatorState.FAULT; LastError = e.Message; UnityEngine.Debug.LogError(e.Message, this); }
        }

        private void ValidateConfiguration()
        {
            if (joints.Length != 6) throw new InvalidOperationException("Arm requires six explicit joint mappings.");
            var bodies = new HashSet<ArticulationBody>();
            var uniqueNames = new HashSet<string>(StringComparer.Ordinal);
            foreach (var j in joints)
            {
                if (j.body == null || !bodies.Add(j.body) || !uniqueNames.Add(j.jointName) ||
                    j.body.GetComponent<UrdfJoint>()?.jointName != j.jointName ||
                    j.body.jointType != ArticulationJointType.RevoluteJoint || j.body.dofCount != 1 ||
                    (j.sign != 1 && j.sign != -1) || !j.body.useGravity || j.body.immovable)
                    throw new InvalidOperationException($"Invalid mapping/physics for arm joint {j.jointName}.");
                foreach (float value in new[] {j.lower,j.upper,j.maximumVelocity,j.torqueLimit,j.stiffness,j.damping})
                    if (!float.IsFinite(value)) throw new InvalidOperationException($"Non-finite configuration: {j.jointName}");
                if (j.lower >= j.upper || j.maximumVelocity <= 0 || j.torqueLimit <= 0 || j.stiffness <= 0 || j.damping <= 0)
                    throw new InvalidOperationException($"Invalid actuator limits/gains: {j.jointName}");
                var d = j.body.xDrive;
                float low = ArmCommandValidation.DriveDegrees(j.sign > 0 ? j.lower : j.upper, j.sign);
                float high = ArmCommandValidation.DriveDegrees(j.sign > 0 ? j.upper : j.lower, j.sign);
                if (Mathf.Abs(d.lowerLimit-low)>0.01f || Mathf.Abs(d.upperLimit-high)>0.01f)
                    throw new InvalidOperationException($"URDF/Unity position-limit mismatch: {j.jointName}");
            }
            var legacy = GetComponent<ArmJointHoldController>();
            if (legacy != null && legacy.enabled) throw new InvalidOperationException("Remove legacy arm hold: two drive owners are forbidden.");
        }

        // Called by the transport's FixedUpdate on the main thread, never by a network callback.
        public bool Accept(string[] packetNames, double[] q, double[] v, double stamp, double receipt)
        {
            if (!ready || quitting || !isActiveAndEnabled || State == ArmActuatorState.FAULT) return false;
            try
            {
                var map = ArmCommandValidation.Map(names, packetNames, q, v);
                if (!double.IsFinite(stamp) || stamp <= lastStamp || WallTime-receipt > watchdogSeconds ||
                    MotionPlanningSim.ROS.RosTimeUtility.PhysicsTimeSeconds-stamp > maximumPacketAgeSeconds ||
                    stamp-MotionPlanningSim.ROS.RosTimeUtility.PhysicsTimeSeconds > maximumPacketAgeSeconds)
                    throw new ArgumentException("Stale, repeated, future, or wrong-epoch arm command.");
                for (int i=0;i<6;++i)
                {
                    var j=joints[i]; int k=map[i];
                    if (q[k]<j.lower-1e-6 || q[k]>j.upper+1e-6 || Math.Abs(v[k])>j.maximumVelocity+1e-6)
                        throw new ArgumentException($"Arm target exceeds physical limits: {j.jointName}");
                }
                for (int i=0;i<6;++i) {DesiredPosition[i]=q[map[i]]; DesiredVelocity[i]=v[map[i]];}
                lastReceipt=receipt; lastStamp=stamp;
                ++AcceptedPackets;
                State=Array.TrueForAll(v, value => Math.Abs(value)<1e-6) ? ArmActuatorState.HOLD : ArmActuatorState.EXTERNAL_CONTROL;
                return true;
            }
            catch (ArgumentException e)
            {
                ++RejectedPackets; LastError=e.Message;
                if (RejectedPackets == 1 || RejectedPackets % 100 == 0) UnityEngine.Debug.LogWarning(e.Message, this);
                return false; // invalid data never refreshes watchdog or partially updates targets
            }
        }

        public bool CaptureHold(ArmActuatorState state = ArmActuatorState.HOLD)
        {
            // OnDisable also runs after physics teardown. A surviving managed body reference
            // does not imply that its native reduced-coordinate buffer still has a DOF.
            if (!ready || quitting || !Application.isPlaying) return false;
            for (int i=0;i<6;++i)
            {
                var j=joints[i];
                if (j.body==null || !j.body.gameObject.activeInHierarchy) return false;
                var position=j.body.jointPosition;
                if (position.dofCount!=1 || !float.IsFinite(position[0])) return false;
                holdPositions[i]=ArmCommandValidation.RosRadians(position[0],j.sign);
            }
            // Commit only a complete snapshot; teardown cannot leave partial hold targets.
            for (int i=0;i<6;++i) {DesiredPosition[i]=holdPositions[i]; DesiredVelocity[i]=0;}
            State=state;
            return true;
        }
        public double Position(int i) => ArmCommandValidation.RosRadians(joints[i].body.jointPosition[0], joints[i].sign);
        public double Velocity(int i) => ArmCommandValidation.RosRadians(joints[i].body.jointVelocity[0], joints[i].sign);

        private void FixedUpdate()
        {
            ++PhysicsTicks;
            if (!ready || State==ArmActuatorState.FAULT) return;
            if ((State==ArmActuatorState.EXTERNAL_CONTROL || State==ArmActuatorState.HOLD) && lastReceipt>0 && CommandAge>watchdogSeconds)
            {
                if (!CaptureHold(ArmActuatorState.WATCHDOG_HOLD))
                {
                    ready=false; State=ArmActuatorState.FAULT;
                    LastError="Arm physics became unavailable while capturing watchdog hold.";
                    UnityEngine.Debug.LogError(LastError,this);
                    return;
                }
            }
            ApplyDrives();
        }
        private void ApplyDrives()
        {
            for (int i=0;i<6;++i)
            {
                var j=joints[i]; var d=j.body.xDrive;
                d.driveType=ArticulationDriveType.Force;
                d.stiffness=j.stiffness; d.damping=j.damping; d.forceLimit=j.torqueLimit;
                d.target=ArmCommandValidation.DriveDegrees(DesiredPosition[i],j.sign);
                d.targetVelocity=ArmCommandValidation.DriveDegrees(DesiredVelocity[i],j.sign);
                j.body.maxJointVelocity=j.maximumVelocity;
                j.body.xDrive=d;
            }
        }
        private void OnDisable()
        {
            // Leave finite drives holding even when the transport/component is disabled.
            if (CaptureHold()) ApplyDrives();
        }
        private void OnApplicationQuit() { quitting=true; ready=false; }
    }
}
