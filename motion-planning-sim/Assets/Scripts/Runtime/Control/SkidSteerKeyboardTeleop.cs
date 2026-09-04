using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MotionPlanningSim.Control
{
    /// <summary>
    /// Optional in-Editor keyboard command source for base commissioning.
    /// ROS remains the normal command owner whenever this tester is disabled.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(SkidSteerBaseController))]
    public sealed class SkidSteerKeyboardTeleop : MonoBehaviour
    {
        [Header("Optional Test Drive")]
        [SerializeField, Tooltip(
            "When enabled, keyboard commands override /cmd_vel. Disable this to return control to ROS.")]
        private bool enableKeyboardTeleop;

        [SerializeField]
        private SkidSteerBaseController controller;

        [SerializeField, Min(0.0f)]
        private float forwardSpeedMetresPerSecond = 0.35f;

        [SerializeField, Min(0.0f)]
        private float reverseSpeedMetresPerSecond = 0.25f;

        [SerializeField, Min(0.0f)]
        private float angularSpeedRadiansPerSecond = 0.6f;

        private bool overrideWasActive;

        public bool EnableKeyboardTeleop
        {
            get => enableKeyboardTeleop;
            set => enableKeyboardTeleop = value;
        }

        public SkidSteerBaseController Controller => controller;

        public void Configure(SkidSteerBaseController configuredController)
        {
            controller = configuredController != null
                ? configuredController
                : throw new ArgumentNullException(nameof(configuredController));
        }

        private void Awake()
        {
            controller ??= GetComponent<SkidSteerBaseController>();
            if (controller == null)
            {
                throw new InvalidOperationException(
                    "Keyboard teleop requires a SkidSteerBaseController on the same GameObject.");
            }
        }

        private void Update()
        {
            if (!enableKeyboardTeleop)
            {
                ReleaseOverride();
                return;
            }

            var keyboard = Keyboard.current;
            var forwardAxis = 0.0f;
            var turnAxis = 0.0f;
            if (keyboard != null)
            {
                forwardAxis = Axis(
                    keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed,
                    keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed);
                turnAxis = Axis(
                    keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed,
                    keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed);

                if (keyboard.spaceKey.isPressed)
                {
                    forwardAxis = 0.0f;
                    turnAxis = 0.0f;
                }
            }

            var command = KeyboardTeleopMapping.ComputeCommand(
                forwardAxis,
                turnAxis,
                forwardSpeedMetresPerSecond,
                reverseSpeedMetresPerSecond,
                angularSpeedRadiansPerSecond);
            controller.SetLocalCommandOverride(
                command.LinearMetresPerSecond,
                command.AngularRadiansPerSecond);
            overrideWasActive = true;
        }

        private void OnDisable()
        {
            ReleaseOverride();
        }

        private void OnValidate()
        {
            forwardSpeedMetresPerSecond = Mathf.Max(0.0f, forwardSpeedMetresPerSecond);
            reverseSpeedMetresPerSecond = Mathf.Max(0.0f, reverseSpeedMetresPerSecond);
            angularSpeedRadiansPerSecond = Mathf.Max(0.0f, angularSpeedRadiansPerSecond);
        }

        private void ReleaseOverride()
        {
            if (!overrideWasActive || controller == null)
            {
                return;
            }

            controller.ClearLocalCommandOverride();
            overrideWasActive = false;
        }

        private static float Axis(bool positive, bool negative)
        {
            return (positive ? 1.0f : 0.0f) - (negative ? 1.0f : 0.0f);
        }
    }

    public static class KeyboardTeleopMapping
    {
        public static PlanarVelocity ComputeCommand(
            float forwardAxis,
            float turnAxis,
            float forwardSpeedMetresPerSecond,
            float reverseSpeedMetresPerSecond,
            float angularSpeedRadiansPerSecond)
        {
            var clampedForward = Mathf.Clamp(forwardAxis, -1.0f, 1.0f);
            var clampedTurn = Mathf.Clamp(turnAxis, -1.0f, 1.0f);
            var linearSpeed = clampedForward >= 0.0f
                ? clampedForward * Mathf.Max(0.0f, forwardSpeedMetresPerSecond)
                : clampedForward * Mathf.Max(0.0f, reverseSpeedMetresPerSecond);
            return new PlanarVelocity(
                linearSpeed,
                clampedTurn * Mathf.Max(0.0f, angularSpeedRadiansPerSecond));
        }
    }
}
