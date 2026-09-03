using System;
using UnityEngine;

namespace MotionPlanningSim.Control
{
    public readonly struct PlanarVelocity
    {
        public PlanarVelocity(float linearMetresPerSecond, float angularRadiansPerSecond)
        {
            LinearMetresPerSecond = linearMetresPerSecond;
            AngularRadiansPerSecond = angularRadiansPerSecond;
        }

        public float LinearMetresPerSecond { get; }
        public float AngularRadiansPerSecond { get; }
    }

    public readonly struct WheelVelocityTargets
    {
        public WheelVelocityTargets(
            float leftRadiansPerSecond,
            float rightRadiansPerSecond,
            bool saturated = false)
        {
            LeftRadiansPerSecond = leftRadiansPerSecond;
            RightRadiansPerSecond = rightRadiansPerSecond;
            Saturated = saturated;
        }

        public float LeftRadiansPerSecond { get; }
        public float RightRadiansPerSecond { get; }
        public bool Saturated { get; }
    }

    /// <summary>
    /// Pure skid-steer command shaping. This intentionally contains no ROS or Unity
    /// object access so its safety-critical calculations can be tested in EditMode.
    /// </summary>
    public static class SkidSteerKinematics
    {
        public static bool IsCommandStale(
            bool hasReceivedCommand,
            double commandAgeSeconds,
            float timeoutSeconds)
        {
            RequirePositive(timeoutSeconds, nameof(timeoutSeconds));
            return !hasReceivedCommand || !double.IsFinite(commandAgeSeconds) ||
                   commandAgeSeconds > timeoutSeconds;
        }

        public static PlanarVelocity ClampChassisCommand(
            PlanarVelocity requested,
            float maximumForwardMetresPerSecond,
            float maximumReverseMetresPerSecond,
            float maximumAngularRadiansPerSecond)
        {
            RequirePositive(maximumForwardMetresPerSecond, nameof(maximumForwardMetresPerSecond));
            RequirePositive(maximumReverseMetresPerSecond, nameof(maximumReverseMetresPerSecond));
            RequirePositive(maximumAngularRadiansPerSecond, nameof(maximumAngularRadiansPerSecond));

            return new PlanarVelocity(
                Mathf.Clamp(
                    requested.LinearMetresPerSecond,
                    -maximumReverseMetresPerSecond,
                    maximumForwardMetresPerSecond),
                Mathf.Clamp(
                    requested.AngularRadiansPerSecond,
                    -maximumAngularRadiansPerSecond,
                    maximumAngularRadiansPerSecond));
        }

        public static PlanarVelocity RateLimitChassisCommand(
            PlanarVelocity current,
            PlanarVelocity target,
            float linearAccelerationMetresPerSecondSquared,
            float linearDecelerationMetresPerSecondSquared,
            float angularAccelerationRadiansPerSecondSquared,
            float angularDecelerationRadiansPerSecondSquared,
            float deltaTimeSeconds)
        {
            RequirePositive(linearAccelerationMetresPerSecondSquared,
                nameof(linearAccelerationMetresPerSecondSquared));
            RequirePositive(linearDecelerationMetresPerSecondSquared,
                nameof(linearDecelerationMetresPerSecondSquared));
            RequirePositive(angularAccelerationRadiansPerSecondSquared,
                nameof(angularAccelerationRadiansPerSecondSquared));
            RequirePositive(angularDecelerationRadiansPerSecondSquared,
                nameof(angularDecelerationRadiansPerSecondSquared));

            if (!float.IsFinite(deltaTimeSeconds) || deltaTimeSeconds <= 0.0f)
            {
                return current;
            }

            return new PlanarVelocity(
                RateLimitAxis(
                    current.LinearMetresPerSecond,
                    target.LinearMetresPerSecond,
                    linearAccelerationMetresPerSecondSquared,
                    linearDecelerationMetresPerSecondSquared,
                    deltaTimeSeconds),
                RateLimitAxis(
                    current.AngularRadiansPerSecond,
                    target.AngularRadiansPerSecond,
                    angularAccelerationRadiansPerSecondSquared,
                    angularDecelerationRadiansPerSecondSquared,
                    deltaTimeSeconds));
        }

        public static float RateLimitAxis(
            float current,
            float target,
            float acceleration,
            float deceleration,
            float deltaTimeSeconds)
        {
            RequirePositive(acceleration, nameof(acceleration));
            RequirePositive(deceleration, nameof(deceleration));

            if (!float.IsFinite(current) || !float.IsFinite(target))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(target),
                    "Velocity values must be finite.");
            }

            if (!float.IsFinite(deltaTimeSeconds) || deltaTimeSeconds <= 0.0f)
            {
                return current;
            }

            // A reversal first brakes to zero. Acceleration in the opposite
            // direction starts on the next physics tick, avoiding a hidden
            // acceleration spike while crossing through zero.
            if (current * target < 0.0f)
            {
                return Mathf.MoveTowards(current, 0.0f, deceleration * deltaTimeSeconds);
            }

            var speedingUp = Mathf.Abs(target) > Mathf.Abs(current);
            var rate = speedingUp ? acceleration : deceleration;
            return Mathf.MoveTowards(current, target, rate * deltaTimeSeconds);
        }

        public static WheelVelocityTargets ComputeSkidSteerWheelTargets(
            PlanarVelocity chassisVelocity,
            float wheelRadiusMetres,
            float trackWidthMetres)
        {
            RequirePositive(wheelRadiusMetres, nameof(wheelRadiusMetres));
            RequirePositive(trackWidthMetres, nameof(trackWidthMetres));

            var halfTrack = trackWidthMetres * 0.5f;
            var leftLinear = chassisVelocity.LinearMetresPerSecond -
                             chassisVelocity.AngularRadiansPerSecond * halfTrack;
            var rightLinear = chassisVelocity.LinearMetresPerSecond +
                              chassisVelocity.AngularRadiansPerSecond * halfTrack;
            return new WheelVelocityTargets(
                leftLinear / wheelRadiusMetres,
                rightLinear / wheelRadiusMetres);
        }

        public static WheelVelocityTargets SaturateWheelTargets(
            WheelVelocityTargets requested,
            float maximumWheelRadiansPerSecond)
        {
            RequirePositive(maximumWheelRadiansPerSecond, nameof(maximumWheelRadiansPerSecond));

            var largestMagnitude = Mathf.Max(
                Mathf.Abs(requested.LeftRadiansPerSecond),
                Mathf.Abs(requested.RightRadiansPerSecond));
            if (largestMagnitude <= maximumWheelRadiansPerSecond)
            {
                return requested;
            }

            var scale = maximumWheelRadiansPerSecond / largestMagnitude;
            return new WheelVelocityTargets(
                requested.LeftRadiansPerSecond * scale,
                requested.RightRadiansPerSecond * scale,
                true);
        }

        public static float ToArticulationDriveAngularVelocity(float radiansPerSecond)
        {
            if (!float.IsFinite(radiansPerSecond))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(radiansPerSecond),
                    "Angular velocity must be finite.");
            }

            return radiansPerSecond * Mathf.Rad2Deg;
        }

        private static void RequirePositive(float value, string parameterName)
        {
            if (!float.IsFinite(value) || value <= 0.0f)
            {
                throw new ArgumentOutOfRangeException(
                    parameterName,
                    "The value must be finite and greater than zero.");
            }
        }
    }
}
