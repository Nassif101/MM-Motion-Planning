using System;
using System.Diagnostics;
using System.Globalization;
using System.Threading;
using MotionPlanningSim.Environment;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace MotionPlanningSim.Control
{
    /// <summary>
    /// Low-level ROS Twist-to-ArticulationDrive adapter for the four-wheel base.
    /// Planning, localization, path following, and command arbitration remain in ROS 2.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SkidSteerBaseController : MonoBehaviour
    {
        [Header("ROS")]
        [SerializeField]
        private string commandTopic = "/cmd_vel";

        [SerializeField, Min(0.01f)]
        private float watchdogTimeoutSeconds =
            MobileManipulatorPhysicalContract.CommandWatchdogSeconds;

        [Header("Robot Geometry")]
        [SerializeField, Min(0.001f)]
        private float wheelRadiusMetres = MobileManipulatorPhysicalContract.WheelRadiusMetres;

        [SerializeField, Min(0.001f)]
        private float geometricTrackWidthMetres =
            MobileManipulatorPhysicalContract.WheelTrackMetres;

        [SerializeField, Min(0.001f), Tooltip(
            "Experimentally identified kinematic track used for skid-steer command mapping. " +
            "This does not change the physical wheel spacing.")]
        private float effectiveTrackWidthMetres =
            MobileManipulatorPhysicalContract.EffectiveSkidSteerTrackMetres;

        [Header("Chassis Limits")]
        [SerializeField, Min(0.0f)]
        private float maximumForwardMetresPerSecond =
            MobileManipulatorPhysicalContract.MaximumForwardMetresPerSecond;

        [SerializeField, Min(0.0f)]
        private float maximumReverseMetresPerSecond =
            MobileManipulatorPhysicalContract.MaximumReverseMetresPerSecond;

        [SerializeField, Min(0.0f)]
        private float maximumAngularRadiansPerSecond =
            MobileManipulatorPhysicalContract.MaximumAngularRadiansPerSecond;

        [SerializeField, Min(0.0f)]
        private float linearAccelerationMetresPerSecondSquared =
            MobileManipulatorPhysicalContract.LinearAccelerationMetresPerSecondSquared;

        [SerializeField, Min(0.0f)]
        private float linearDecelerationMetresPerSecondSquared =
            MobileManipulatorPhysicalContract.LinearDecelerationMetresPerSecondSquared;

        [SerializeField, Min(0.0f)]
        private float angularAccelerationRadiansPerSecondSquared =
            MobileManipulatorPhysicalContract.AngularAccelerationRadiansPerSecondSquared;

        [SerializeField, Min(0.0f)]
        private float angularDecelerationRadiansPerSecondSquared =
            MobileManipulatorPhysicalContract.AngularDecelerationRadiansPerSecondSquared;

        [Header("Wheel Limits")]
        [SerializeField, Min(0.0f)]
        private float maximumWheelRadiansPerSecond =
            MobileManipulatorPhysicalContract.MaximumOperatingWheelRadiansPerSecond;

        [SerializeField, Min(0.0f)]
        private float hardJointVelocityRadiansPerSecond =
            MobileManipulatorPhysicalContract.HardWheelJointVelocityRadiansPerSecond;

        [Header("Articulation Drive")]
        [SerializeField, Min(0.0f)]
        private float driveDampingNewtonMetreSecondsPerRadian =
            MobileManipulatorPhysicalContract.WheelDriveDampingNewtonMetreSecondsPerRadian;

        [SerializeField, Min(0.0f)]
        private float driveTorqueLimitNewtonMetres =
            MobileManipulatorPhysicalContract.WheelDriveTorqueLimitNewtonMetres;

        [SerializeField, Min(0.0f)]
        private float jointFriction = MobileManipulatorPhysicalContract.WheelJointFriction;

        [SerializeField, Min(0.0f)]
        private float wheelBodyDamping = MobileManipulatorPhysicalContract.WheelBodyDamping;

        [Header("Wheel References")]
        [SerializeField]
        private ArticulationBody chassis;

        [SerializeField]
        private ArticulationBody frontLeftWheel;

        [SerializeField]
        private ArticulationBody rearLeftWheel;

        [SerializeField]
        private ArticulationBody frontRightWheel;

        [SerializeField]
        private ArticulationBody rearRightWheel;

        [Header("URDF Joint Direction")]
        [SerializeField, Tooltip("Use +1 when positive URDF joint velocity rolls the wheel forward; otherwise -1.")]
        private float frontLeftDirection = 1.0f;

        [SerializeField]
        private float rearLeftDirection = 1.0f;

        [SerializeField]
        private float frontRightDirection = 1.0f;

        [SerializeField]
        private float rearRightDirection = 1.0f;

        [Header("Diagnostics")]
        [SerializeField]
        private bool logCsvDiagnostics;

        [SerializeField, Min(1.0f)]
        private float diagnosticFrequencyHz = 10.0f;

        private readonly object commandGate = new object();
        private ROSConnection ros;
        private bool subscribed;
        private bool hasReceivedCommand;
        private float requestedLinear;
        private float requestedAngular;
        private long lastCommandTimestamp;
        private int rejectedCommandCount;
        private PlanarVelocity limitedCommand;
        private WheelVelocityTargets wheelTargets;
        private bool watchdogActive = true;
        private double nextDiagnosticTime;
        private bool diagnosticHeaderWritten;

        public string CommandTopic => commandTopic;
        public bool WatchdogActive => watchdogActive;
        public bool WheelCommandSaturated => wheelTargets.Saturated;
        public float LimitedLinearMetresPerSecond => limitedCommand.LinearMetresPerSecond;
        public float LimitedAngularRadiansPerSecond => limitedCommand.AngularRadiansPerSecond;
        public float LeftWheelTargetRadiansPerSecond => wheelTargets.LeftRadiansPerSecond;
        public float RightWheelTargetRadiansPerSecond => wheelTargets.RightRadiansPerSecond;
        public int RejectedCommandCount => Volatile.Read(ref rejectedCommandCount);

        public void Configure(
            ArticulationBody configuredChassis,
            ArticulationBody configuredFrontLeftWheel,
            ArticulationBody configuredRearLeftWheel,
            ArticulationBody configuredFrontRightWheel,
            ArticulationBody configuredRearRightWheel)
        {
            chassis = configuredChassis;
            frontLeftWheel = configuredFrontLeftWheel;
            rearLeftWheel = configuredRearLeftWheel;
            frontRightWheel = configuredFrontRightWheel;
            rearRightWheel = configuredRearRightWheel;
            ValidateConfiguration();
            ConfigureWheelDrives();
        }

        private void Awake()
        {
            ValidateConfiguration();
            ConfigureWheelDrives();
            ResetRuntimeState();
        }

        private void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>(commandTopic, OnTwistReceived);
            subscribed = true;
        }

        private void FixedUpdate()
        {
            PlanarVelocity requested;
            long receivedTimestamp;
            bool received;
            lock (commandGate)
            {
                requested = new PlanarVelocity(requestedLinear, requestedAngular);
                receivedTimestamp = lastCommandTimestamp;
                received = hasReceivedCommand;
            }

            watchdogActive = SkidSteerKinematics.IsCommandStale(
                received,
                received ? ElapsedSeconds(receivedTimestamp) : double.PositiveInfinity,
                watchdogTimeoutSeconds);
            if (watchdogActive)
            {
                requested = new PlanarVelocity(0.0f, 0.0f);
            }

            var clamped = SkidSteerKinematics.ClampChassisCommand(
                requested,
                maximumForwardMetresPerSecond,
                maximumReverseMetresPerSecond,
                maximumAngularRadiansPerSecond);
            limitedCommand = SkidSteerKinematics.RateLimitChassisCommand(
                limitedCommand,
                clamped,
                linearAccelerationMetresPerSecondSquared,
                linearDecelerationMetresPerSecondSquared,
                angularAccelerationRadiansPerSecondSquared,
                angularDecelerationRadiansPerSecondSquared,
                Time.fixedDeltaTime);
            wheelTargets = SkidSteerKinematics.SaturateWheelTargets(
                SkidSteerKinematics.ComputeSkidSteerWheelTargets(
                    limitedCommand,
                    wheelRadiusMetres,
                    effectiveTrackWidthMetres),
                maximumWheelRadiansPerSecond);

            CommandWheelDrives(wheelTargets);
            LogDiagnosticsIfDue();
        }

        private void OnDisable()
        {
            if (subscribed && ros != null)
            {
                ros.Unsubscribe(commandTopic);
                subscribed = false;
            }

            if (frontLeftWheel != null)
            {
                CommandWheelDrives(new WheelVelocityTargets(0.0f, 0.0f));
            }

            ResetRuntimeState();
        }

        private void OnTwistReceived(TwistMsg message)
        {
            if (message?.linear == null || message.angular == null ||
                !double.IsFinite(message.linear.x) ||
                !double.IsFinite(message.angular.z))
            {
                Interlocked.Increment(ref rejectedCommandCount);
                return;
            }

            var linear = (float)message.linear.x;
            var angular = (float)message.angular.z;
            if (!float.IsFinite(linear) || !float.IsFinite(angular))
            {
                Interlocked.Increment(ref rejectedCommandCount);
                return;
            }

            lock (commandGate)
            {
                requestedLinear = linear;
                requestedAngular = angular;
                lastCommandTimestamp = Stopwatch.GetTimestamp();
                hasReceivedCommand = true;
            }
        }

        private void ConfigureWheelDrives()
        {
            ConfigureWheelDrive(frontLeftWheel);
            ConfigureWheelDrive(rearLeftWheel);
            ConfigureWheelDrive(frontRightWheel);
            ConfigureWheelDrive(rearRightWheel);
        }

        private void ConfigureWheelDrive(ArticulationBody wheel)
        {
            wheel.twistLock = ArticulationDofLock.FreeMotion;
            wheel.jointFriction = jointFriction;
            wheel.linearDamping = wheelBodyDamping;
            wheel.angularDamping = wheelBodyDamping;
            wheel.maxJointVelocity = hardJointVelocityRadiansPerSecond;
            wheel.maxAngularVelocity = hardJointVelocityRadiansPerSecond;

            var drive = wheel.xDrive;
            drive.driveType = ArticulationDriveType.Force;
            drive.stiffness = 0.0f;
            drive.damping = driveDampingNewtonMetreSecondsPerRadian;
            drive.forceLimit = driveTorqueLimitNewtonMetres;
            drive.target = 0.0f;
            drive.targetVelocity = 0.0f;
            wheel.xDrive = drive;
        }

        private void CommandWheelDrives(WheelVelocityTargets targets)
        {
            SetTargetVelocity(frontLeftWheel, targets.LeftRadiansPerSecond * frontLeftDirection);
            SetTargetVelocity(rearLeftWheel, targets.LeftRadiansPerSecond * rearLeftDirection);
            SetTargetVelocity(frontRightWheel, targets.RightRadiansPerSecond * frontRightDirection);
            SetTargetVelocity(rearRightWheel, targets.RightRadiansPerSecond * rearRightDirection);
        }

        private static void SetTargetVelocity(ArticulationBody wheel, float radiansPerSecond)
        {
            var drive = wheel.xDrive;
            // ArticulationBody jointVelocity is radians/second, while the angular
            // ArticulationDrive inspector/API target is expressed in degrees/second.
            // Keep the controller contract in SI and convert only at the Unity boundary.
            drive.targetVelocity =
                SkidSteerKinematics.ToArticulationDriveAngularVelocity(radiansPerSecond);
            wheel.xDrive = drive;
        }

        private void ResetRuntimeState()
        {
            lock (commandGate)
            {
                requestedLinear = 0.0f;
                requestedAngular = 0.0f;
                lastCommandTimestamp = 0L;
                hasReceivedCommand = false;
            }

            limitedCommand = new PlanarVelocity(0.0f, 0.0f);
            wheelTargets = new WheelVelocityTargets(0.0f, 0.0f);
            watchdogActive = true;
            nextDiagnosticTime = 0.0;
            diagnosticHeaderWritten = false;
        }

        private void LogDiagnosticsIfDue()
        {
            if (!logCsvDiagnostics || Time.fixedTimeAsDouble < nextDiagnosticTime)
            {
                return;
            }

            nextDiagnosticTime = Time.fixedTimeAsDouble + 1.0 / diagnosticFrequencyHz;
            if (!diagnosticHeaderWritten)
            {
                Debug.Log("SKID_STEER_CSV,time_s,limited_v_mps,limited_w_radps," +
                          "target_left_radps,target_right_radps,actual_fl_radps," +
                          "actual_rl_radps,actual_fr_radps,actual_rr_radps," +
                          "actual_v_mps,actual_w_radps,watchdog,saturated");
                diagnosticHeaderWritten = true;
            }

            var actualForward = Vector3.Dot(chassis.linearVelocity, chassis.transform.forward);
            // Unity is left-handed. Positive ROS FLU yaw maps to negative rotation
            // about the Unity articulation's local up axis.
            var actualYaw = -Vector3.Dot(chassis.angularVelocity, chassis.transform.up);
            Debug.Log(string.Format(
                CultureInfo.InvariantCulture,
                "SKID_STEER_CSV,{0:F3},{1:F5},{2:F5},{3:F5},{4:F5}," +
                "{5:F5},{6:F5},{7:F5},{8:F5},{9:F5},{10:F5},{11},{12}",
                Time.fixedTimeAsDouble,
                limitedCommand.LinearMetresPerSecond,
                limitedCommand.AngularRadiansPerSecond,
                wheelTargets.LeftRadiansPerSecond,
                wheelTargets.RightRadiansPerSecond,
                frontLeftWheel.jointVelocity[0] * frontLeftDirection,
                rearLeftWheel.jointVelocity[0] * rearLeftDirection,
                frontRightWheel.jointVelocity[0] * frontRightDirection,
                rearRightWheel.jointVelocity[0] * rearRightDirection,
                actualForward,
                actualYaw,
                watchdogActive ? 1 : 0,
                wheelTargets.Saturated ? 1 : 0));
        }

        private static double ElapsedSeconds(long startTimestamp)
        {
            return (Stopwatch.GetTimestamp() - startTimestamp) /
                   (double)Stopwatch.Frequency;
        }

        private void ValidateConfiguration()
        {
            if (string.IsNullOrWhiteSpace(commandTopic))
            {
                throw new InvalidOperationException("The Twist command topic is required.");
            }

            var wheels = new[]
            {
                frontLeftWheel,
                rearLeftWheel,
                frontRightWheel,
                rearRightWheel
            };
            if (chassis == null || Array.Exists(wheels, wheel => wheel == null))
            {
                throw new InvalidOperationException(
                    "The chassis and all four wheel ArticulationBody references are required.");
            }

            if (!chassis.isRoot)
            {
                throw new InvalidOperationException("The configured chassis must be the root ArticulationBody.");
            }

            for (var index = 0; index < wheels.Length; index++)
            {
                if (wheels[index].jointType != ArticulationJointType.RevoluteJoint ||
                    wheels[index].dofCount != 1)
                {
                    throw new InvalidOperationException(
                        $"Wheel reference {index} must be a one-DOF revolute ArticulationBody.");
                }

                for (var other = index + 1; other < wheels.Length; other++)
                {
                    if (wheels[index] == wheels[other])
                    {
                        throw new InvalidOperationException("Wheel references must be unique.");
                    }
                }
            }

            ValidateDirection(frontLeftDirection, nameof(frontLeftDirection));
            ValidateDirection(rearLeftDirection, nameof(rearLeftDirection));
            ValidateDirection(frontRightDirection, nameof(frontRightDirection));
            ValidateDirection(rearRightDirection, nameof(rearRightDirection));

            if (watchdogTimeoutSeconds <= 0.0f || wheelRadiusMetres <= 0.0f ||
                geometricTrackWidthMetres <= 0.0f || effectiveTrackWidthMetres <= 0.0f ||
                maximumForwardMetresPerSecond <= 0.0f ||
                maximumReverseMetresPerSecond <= 0.0f || maximumAngularRadiansPerSecond <= 0.0f ||
                linearAccelerationMetresPerSecondSquared <= 0.0f ||
                linearDecelerationMetresPerSecondSquared <= 0.0f ||
                angularAccelerationRadiansPerSecondSquared <= 0.0f ||
                angularDecelerationRadiansPerSecondSquared <= 0.0f ||
                maximumWheelRadiansPerSecond <= 0.0f ||
                hardJointVelocityRadiansPerSecond < maximumWheelRadiansPerSecond ||
                driveDampingNewtonMetreSecondsPerRadian <= 0.0f ||
                driveTorqueLimitNewtonMetres <= 0.0f || diagnosticFrequencyHz <= 0.0f)
            {
                throw new InvalidOperationException(
                    "Controller geometry, limits, watchdog, drive, and diagnostic values must be positive; " +
                    "the hard joint limit must not be below the operating wheel limit.");
            }
        }

        private static void ValidateDirection(float direction, string fieldName)
        {
            if (direction != 1.0f && direction != -1.0f)
            {
                throw new InvalidOperationException($"{fieldName} must be exactly +1 or -1.");
            }
        }
    }
}
