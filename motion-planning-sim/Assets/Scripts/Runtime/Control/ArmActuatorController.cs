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

        // A still command is a hold. The JTC keeps a finished trajectory's last point, acceleration included, so a hold
        // ignores the commanded acceleration (no motion feedforward or setpoint preview while holding).
        public static bool IsHold(double[] v) => Array.TrueForAll(v, x => Math.Abs(x) < 1e-6);
        public static double HoldAcceleration(bool hold, double a) => hold ? 0.0 : a;

        public static int[] Map(string[] required, string[] names, double[] q, double[] v, double[] a)
        {
            if (names == null || q == null || v == null || a == null || names.Length != required.Length ||
                q.Length != names.Length || v.Length != names.Length || a.Length != names.Length)
                throw new ArgumentException("Arm packet requires all six names, positions, velocities and accelerations.");
            var lookup = new Dictionary<string, int>(StringComparer.Ordinal);
            for (int i = 0; i < names.Length; ++i)
            {
                if (string.IsNullOrEmpty(names[i]) || !lookup.TryAdd(names[i], i) ||
                    !double.IsFinite(q[i]) || !double.IsFinite(v[i]) || !double.IsFinite(a[i]))
                    throw new ArgumentException("Arm packet has duplicate/empty names or non-finite values.");
            }
            var result = new int[required.Length];
            for (int i = 0; i < required.Length; ++i)
                if (!lookup.TryGetValue(required[i], out result[i]))
                    throw new ArgumentException($"Missing arm joint {required[i]}; packet contains an unknown joint.");
            return result;
        }
    }

    // Gravity feedforward (Phase 4B-B; on by default since 2026-10-23, off is the frozen arm): the torque that holds each joint against
    // gravity at its measured configuration. The PD drive supplies it: its position target shifts by torque /
    // stiffness, so at zero error the drive pushes exactly the holding torque and its force limit still caps the
    // actuator's total (for an unsaturated drive the same as adding the torque). Writing ArticulationBody.jointForce
    // instead held the arm still but made every moving joint lag (observed 2026-10-23, cause in Unity's jointForce
    // path unknown: elbow path error 0.007 -> 0.018 rad, wrist 2 lagging with no torque of its own). The torque is
    // the statics of the bodies each joint carries, as for a fixed base: GetJointGravityForces treats the mobile
    // (floating) root as in free fall and returns zero (measured 2026-10-23), while the wheels hold the base up.
    public static class ArmGravityFeedforward
    {
        // The drive-target shift (rad) at which the drive pushes `holdingTorque` (clamped to the torque limit) at zero
        // error; zero for a non-finite torque. Unity's revolute drive stiffness is per radian.
        public static float TargetOffset(float holdingTorque, float torqueLimit, float stiffness) =>
            float.IsFinite(holdingTorque) ? Mathf.Clamp(holdingTorque, -torqueLimit, torqueLimit) / stiffness : 0f;

        // The torque about `axis` (through `pointOnAxis`) that holds the carried masses against gravity.
        public static float HoldingTorque(Vector3 axis, Vector3 pointOnAxis, float[] masses, Vector3[] centers,
                                          Vector3 gravity)
        {
            var torque = Vector3.zero;
            for (int k = 0; k < masses.Length; ++k) torque += Vector3.Cross(centers[k] - pointOnAxis, masses[k] * gravity);
            return -Vector3.Dot(axis.normalized, torque);
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
        [SerializeField] private bool gravityFeedforward = true;
        public bool GravityFeedforward { get => gravityFeedforward; set => gravityFeedforward = value; }
        // Motion feedforward (Phase 4B-B, ADR 0014; on by default since 2026-10-24): the inverse dynamics of the commanded joint
        // velocities and accelerations and of the base's reference motion, with gravity when that is on.
        [SerializeField] private bool motionFeedforward = true;
        public bool MotionFeedforward { get => motionFeedforward; set => motionFeedforward = value; }
        // Setpoint preview (Phase 4B-B, ADR 0014; on by default since 2026-10-24): drive toward the command carried to the end of the
        // physics step (ArmSetpointPreview), at most maximumPreviewSeconds ahead; holds have zero rates.
        [SerializeField] private bool setpointPreview = true;
        [SerializeField, Min(0f)] private float maximumPreviewSeconds = 0.1f;
        public bool SetpointPreview { get => setpointPreview; set => setpointPreview = value; }
        /// <summary>How far ahead of its stamp the last applied command was carried (s); zero while off.</summary>
        public double PreviewSeconds { get; private set; }
        /// <summary>The motion part of the feedforward per joint (N m, ROS sign): the total less the gravity part.</summary>
        public double[] MotionFeedforwardTorque { get; } = new double[6];
        /// <summary>The gravity feedforward the drive supplies per joint (N m, ROS sign); zero while switched off.</summary>
        public double[] FeedforwardTorque { get; } = new double[6];
        public ArmActuatorState State { get; private set; } = ArmActuatorState.INITIALIZING;
        public string LastError { get; private set; } = "";
        public int RejectedPackets { get; private set; }
        public long AcceptedPackets { get; private set; }
        public long PhysicsTicks { get; private set; }
        public ArmJointActuator[] Joints => joints;
        public double[] DesiredPosition { get; private set; } = new double[6];
        public double[] DesiredVelocity { get; private set; } = new double[6];
        /// <summary>The JTC's commanded acceleration (rad/s², ROS sign); zero in holds.</summary>
        public double[] DesiredAcceleration { get; private set; } = new double[6];
        public double CommandAge => lastReceipt == 0 ? -1 : WallTime - lastReceipt;
        public static double WallTime => (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
        private string[] names;
        private double lastReceipt, lastStamp = -1;
        private bool ready;
        private bool quitting;
        private readonly double[] holdPositions = new double[6];
        // The world direction of a positive reduced coordinate is this sign times the anchor's x axis (verified live
        // by the feedforward removing the hold sag, not doubling it).
        private const float JointAxisSign = 1f;
        private ArticulationBody[][] carried;
        private float[][] carriedMasses;
        private Vector3[][] carriedCenters;
        // Motion feedforward: every body the arm moves with its deepest moving joint, the base for its reference motion.
        private ArticulationBody[] armBodies;
        private ArmLinkBody[] linkBodies;
        private ArticulationBody baseBody;
        private SkidSteerBaseController baseController;
        private readonly Vector3[] axes = new Vector3[6], points = new Vector3[6];
        private readonly float[] rates = new float[6], accelerations = new float[6], motionTorques = new float[6];

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
                carried = Array.ConvertAll(joints, j => j.body.GetComponentsInChildren<ArticulationBody>());
                carriedMasses = Array.ConvertAll(carried, c => new float[c.Length]);
                carriedCenters = Array.ConvertAll(carried, c => new Vector3[c.Length]);
                armBodies = carried[0];
                linkBodies = Array.ConvertAll(armBodies, b => new ArmLinkBody {
                    joint = Array.FindLastIndex(carried, c => Array.IndexOf(c, b) >= 0)});
                baseBody = Array.Find(GetComponentsInChildren<ArticulationBody>(), b => b.isRoot);
                baseController = GetComponentInChildren<SkidSteerBaseController>() ?? GetComponentInParent<SkidSteerBaseController>();
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
        public bool Accept(string[] packetNames, double[] q, double[] v, double[] a, double stamp, double receipt)
        {
            if (!ready || quitting || !isActiveAndEnabled || State == ArmActuatorState.FAULT) return false;
            try
            {
                var map = ArmCommandValidation.Map(names, packetNames, q, v, a);
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
                bool hold=ArmCommandValidation.IsHold(v);
                for (int i=0;i<6;++i)
                {
                    DesiredPosition[i]=q[map[i]]; DesiredVelocity[i]=v[map[i]];
                    DesiredAcceleration[i]=ArmCommandValidation.HoldAcceleration(hold,a[map[i]]);
                }
                lastReceipt=receipt; lastStamp=stamp;
                ++AcceptedPackets;
                State=hold ? ArmActuatorState.HOLD : ArmActuatorState.EXTERNAL_CONTROL;
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
            for (int i=0;i<6;++i) {DesiredPosition[i]=holdPositions[i]; DesiredVelocity[i]=0; DesiredAcceleration[i]=0;}
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
            bool gravity=gravityFeedforward && carried!=null, motion=motionFeedforward && carried!=null;
            bool feedforward=gravity || motion;
            PreviewSeconds=setpointPreview && lastStamp>=0 && State==ArmActuatorState.EXTERNAL_CONTROL
                ? Math.Clamp(MotionPlanningSim.ROS.RosTimeUtility.PhysicsTimeSeconds+Time.fixedDeltaTime-lastStamp,0,maximumPreviewSeconds) : 0;
            if (motion) MotionTorques(gravity);
            for (int i=0;i<6;++i)
            {
                var j=joints[i]; var d=j.body.xDrive;
                float gravityPart=gravity ? HoldingTorque(i) : 0f, torque=motion ? motionTorques[i] : gravityPart;
                float offset=feedforward ? ArmGravityFeedforward.TargetOffset(torque,j.torqueLimit,j.stiffness) : 0f;
                FeedforwardTorque[i]=ArmCommandValidation.RosRadians(offset*j.stiffness,j.sign);
                MotionFeedforwardTorque[i]=motion ? FeedforwardTorque[i]-ArmCommandValidation.RosRadians(gravityPart,j.sign) : 0;
                d.driveType=ArticulationDriveType.Force;
                d.stiffness=j.stiffness; d.damping=j.damping; d.forceLimit=j.torqueLimit;
                var (q,v)=ArmSetpointPreview.Ahead(DesiredPosition[i],DesiredVelocity[i],DesiredAcceleration[i],PreviewSeconds,maximumPreviewSeconds);
                if (PreviewSeconds>0) {q=Math.Clamp(q,j.lower,j.upper); v=Math.Clamp(v,-j.maximumVelocity,j.maximumVelocity);}
                d.target=ArmCommandValidation.DriveDegrees(q,j.sign);
                // Reduced coordinates: the offset needs no ROS sign (the switched-off target is unchanged).
                if (feedforward) d.target+=offset*Mathf.Rad2Deg;
                d.targetVelocity=ArmCommandValidation.DriveDegrees(v,j.sign);
                j.body.maxJointVelocity=j.maximumVelocity;
                j.body.xDrive=d;
            }
        }
        private float HoldingTorque(int i)
        {
            var body=joints[i].body;
            for (int k=0;k<carried[i].Length;++k)
            { carriedMasses[i][k]=carried[i][k].mass; carriedCenters[i][k]=carried[i][k].worldCenterOfMass; }
            var axis=JointAxisSign*(body.transform.rotation*body.anchorRotation*Vector3.right);
            return ArmGravityFeedforward.HoldingTorque(axis,body.transform.TransformPoint(body.anchorPosition),
                carriedMasses[i],carriedCenters[i],Physics.gravity);
        }
        /// <summary>Verification probe (Play): the inverse-dynamics torques (N m, reduced coordinates) for given joint
        /// rates in reduced coordinates at the current configuration, without gravity or base motion.</summary>
        public float[] ProbeDynamics(float[] velocity, float[] acceleration)
        {
            if (armBodies==null) throw new InvalidOperationException("The arm is not initialized (or faulted).");
            var torques=new float[6];
            for (int i=0;i<6;++i)
            {
                var body=joints[i].body;
                axes[i]=JointAxisSign*(body.transform.rotation*body.anchorRotation*Vector3.right);
                points[i]=body.transform.TransformPoint(body.anchorPosition);
            }
            for (int k=0;k<armBodies.Length;++k)
            {
                var b=armBodies[k];
                linkBodies[k].mass=b.mass; linkBodies[k].center=b.worldCenterOfMass; linkBodies[k].principalInertia=b.inertiaTensor;
                linkBodies[k].principalRotation=b.transform.rotation*b.inertiaTensorRotation;
            }
            ArmInverseDynamics.JointTorques(axes,points,velocity,acceleration,linkBodies,Vector3.zero,new ArmBaseMotion{origin=points[0]},torques);
            return torques;
        }
        private void MotionTorques(bool gravity)
        {
            for (int i=0;i<6;++i)
            {
                var j=joints[i]; var body=j.body;
                axes[i]=JointAxisSign*(body.transform.rotation*body.anchorRotation*Vector3.right);
                points[i]=body.transform.TransformPoint(body.anchorPosition);
                // Reduced coordinates carry the joint's sign; ROS radians times the sign are reduced radians.
                rates[i]=(float)(DesiredVelocity[i]*j.sign); accelerations[i]=(float)(DesiredAcceleration[i]*j.sign);
            }
            for (int k=0;k<armBodies.Length;++k)
            {
                var b=armBodies[k];
                linkBodies[k].mass=b.mass; linkBodies[k].center=b.worldCenterOfMass; linkBodies[k].principalInertia=b.inertiaTensor;
                linkBodies[k].principalRotation=b.transform.rotation*b.inertiaTensorRotation;
            }
            var root=baseController!=null && baseBody!=null
                ? ArmBaseMotion.FromBaseCommand(baseBody.transform.position,baseBody.transform.forward,baseBody.transform.up,
                    baseController.LimitedLinearMetresPerSecond,baseController.LimitedLinearMetresPerSecondSquared,
                    baseController.LimitedAngularRadiansPerSecond,baseController.LimitedAngularRadiansPerSecondSquared)
                : new ArmBaseMotion {origin=points[0]};
            ArmInverseDynamics.JointTorques(axes,points,rates,accelerations,linkBodies,gravity ? Physics.gravity : Vector3.zero,
                root,motionTorques);
        }
        private void OnDisable()
        {
            // Leave finite drives holding even when the transport/component is disabled.
            if (CaptureHold()) ApplyDrives();
        }
        private void OnApplicationQuit() { quitting=true; ready=false; }
    }
}
