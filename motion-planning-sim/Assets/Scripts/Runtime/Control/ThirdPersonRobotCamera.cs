using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MotionPlanningSim.Visualization
{
    public enum RobotCameraView
    {
        Orbit = 0,
        RearLeftChase = 1,
        PayloadFirstPerson = 2
    }

    /// <summary>
    /// Frame-driven robot camera with orbit, rear-left chase, and payload-mounted views.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Camera))]
    public sealed class ThirdPersonRobotCamera : MonoBehaviour
    {
        [Header("Target")]
        [SerializeField]
        private Transform target;

        [SerializeField, Tooltip(
            "Payload centre transform used by the payload first-person view.")]
        private Transform payloadTarget;

        [SerializeField]
        private Vector3 targetOffset = new Vector3(0.0f, 1.25f, 0.0f);

        [Header("View")]
        [SerializeField, Min(0.1f)]
        private float defaultDistanceMetres = 4.0f;

        [SerializeField, Min(0.1f)]
        private float minimumDistanceMetres = 1.5f;

        [SerializeField, Min(0.1f)]
        private float maximumDistanceMetres = 10.0f;

        [SerializeField, Range(5.0f, 80.0f)]
        private float defaultPitchDegrees = 24.0f;

        [Header("Rear-Left Chase View")]
        [SerializeField, Tooltip(
            "Camera position in base_link coordinates: left, up, and behind the robot.")]
        private Vector3 rearLeftLocalPosition = new Vector3(-1.1f, 0.55f, -0.35f);

        [SerializeField, Tooltip("Look point in base_link coordinates.")]
        private Vector3 rearLeftLocalLookAt = new Vector3(0.0f, 0.02f, 0.0f);

        [SerializeField, Range(30.0f, 100.0f)]
        private float rearLeftFieldOfViewDegrees = 70.0f;

        [Header("Payload First-Person View")]
        [SerializeField, Tooltip(
            "Local position relative to the payload centre. The default sits just above its surface.")]
        private Vector3 payloadLocalPosition = new Vector3(0.0f, 0.08f, 0.0f);

        [SerializeField, Tooltip("Local view rotation relative to the payload.")]
        private Vector3 payloadLocalEulerAngles = Vector3.zero;

        [SerializeField, Min(0.001f)]
        private float firstPersonNearClipMetres = 0.03f;

        [Header("Controls")]
        [SerializeField, Tooltip(
            "Press C to cycle views. In orbit view, scroll to zoom, hold right mouse to orbit, and press F to recenter.")]
        private bool enableCameraControls = true;

        [SerializeField, Min(0.0f)]
        private float zoomSensitivity = 0.01f;

        [SerializeField, Min(0.0f)]
        private float orbitSensitivity = 0.15f;

        [SerializeField, Min(0.0f)]
        private float positionSmoothTime = 0.08f;

        [SerializeField, Min(0.0f)]
        private float rotationSharpness = 16.0f;

        [Header("Obstruction Avoidance")]
        [SerializeField]
        private LayerMask collisionLayers = ~0;

        [SerializeField, Min(0.01f)]
        private float collisionRadiusMetres = 0.2f;

        [SerializeField, Min(0.0f)]
        private float collisionPaddingMetres = 0.1f;

        private readonly RaycastHit[] collisionHits = new RaycastHit[16];
        private float distanceMetres;
        private float yawOffsetDegrees;
        private float pitchDegrees;
        private Vector3 followVelocity;
        private Camera controlledCamera;
        private float normalNearClipMetres;
        private float normalFieldOfViewDegrees;
        private RobotCameraView currentView;

        public Transform Target => target;
        public Transform PayloadTarget => payloadTarget;
        public RobotCameraView CurrentView => currentView;

        public void Configure(Transform configuredTarget, Transform configuredPayloadTarget)
        {
            target = configuredTarget != null
                ? configuredTarget
                : throw new ArgumentNullException(nameof(configuredTarget));
            payloadTarget = configuredPayloadTarget != null
                ? configuredPayloadTarget
                : throw new ArgumentNullException(nameof(configuredPayloadTarget));
        }

        public void SetView(RobotCameraView view)
        {
            if (!Enum.IsDefined(typeof(RobotCameraView), view))
            {
                throw new ArgumentOutOfRangeException(nameof(view));
            }

            currentView = view;
            followVelocity = Vector3.zero;
            if (currentView == RobotCameraView.Orbit)
            {
                ResetOrbitView();
            }
        }

        private void Awake()
        {
            ValidateConfiguration();
            controlledCamera = GetComponent<Camera>();
            normalNearClipMetres = controlledCamera.nearClipPlane;
            normalFieldOfViewDegrees = controlledCamera.fieldOfView;
            SetView(RobotCameraView.Orbit);
        }

        private void Start()
        {
            SnapToTarget();
        }

        private void Update()
        {
            if (!enableCameraControls)
            {
                return;
            }

            var keyboard = Keyboard.current;
            if (keyboard?.cKey.wasPressedThisFrame == true)
            {
                SetView(RobotCameraViewCycler.Next(currentView));
            }

            if (keyboard?.fKey.wasPressedThisFrame == true)
            {
                SetView(RobotCameraView.Orbit);
            }

            if (currentView == RobotCameraView.Orbit)
            {
                var mouse = Mouse.current;
                if (mouse == null)
                {
                    return;
                }

                distanceMetres = ThirdPersonCameraMath.ClampDistance(
                    distanceMetres - mouse.scroll.ReadValue().y * zoomSensitivity,
                    minimumDistanceMetres,
                    maximumDistanceMetres);
                if (mouse.rightButton.isPressed)
                {
                    var delta = mouse.delta.ReadValue();
                    yawOffsetDegrees += delta.x * orbitSensitivity;
                    pitchDegrees = Mathf.Clamp(
                        pitchDegrees - delta.y * orbitSensitivity,
                        5.0f,
                        80.0f);
                }
            }
        }

        private void LateUpdate()
        {
            if (target == null)
            {
                return;
            }

            switch (currentView)
            {
                case RobotCameraView.Orbit:
                    UpdateOrbitView();
                    break;
                case RobotCameraView.RearLeftChase:
                    UpdateRearLeftChaseView();
                    break;
                case RobotCameraView.PayloadFirstPerson:
                    UpdatePayloadFirstPersonView();
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        private void UpdateOrbitView()
        {
            controlledCamera.nearClipPlane = normalNearClipMetres;
            controlledCamera.fieldOfView = normalFieldOfViewDegrees;
            var desiredRotation = Quaternion.Euler(
                pitchDegrees,
                target.eulerAngles.y + yawOffsetDegrees,
                0.0f);
            var focus = target.position + targetOffset;
            var boomDirection = -(desiredRotation * Vector3.forward);
            var resolvedDistance = ResolveCameraDistance(
                focus,
                boomDirection,
                distanceMetres);
            var desiredPosition = focus + boomDirection * resolvedDistance;
            MoveSmoothed(desiredPosition, desiredRotation);
        }

        private void UpdateRearLeftChaseView()
        {
            controlledCamera.nearClipPlane = normalNearClipMetres;
            controlledCamera.fieldOfView = rearLeftFieldOfViewDegrees;
            var focus = target.TransformPoint(rearLeftLocalLookAt);
            var requestedPosition = target.TransformPoint(rearLeftLocalPosition);
            var boom = requestedPosition - focus;
            var requestedDistance = boom.magnitude;
            var boomDirection = boom / requestedDistance;
            var resolvedDistance = ResolveCameraDistance(
                focus,
                boomDirection,
                requestedDistance);
            var desiredPosition = focus + boomDirection * resolvedDistance;
            var desiredRotation = Quaternion.LookRotation(
                focus - desiredPosition,
                target.up);
            MoveSmoothed(desiredPosition, desiredRotation);
        }

        private void UpdatePayloadFirstPersonView()
        {
            controlledCamera.nearClipPlane = firstPersonNearClipMetres;
            controlledCamera.fieldOfView = normalFieldOfViewDegrees;
            transform.SetPositionAndRotation(
                payloadTarget.TransformPoint(payloadLocalPosition),
                payloadTarget.rotation * Quaternion.Euler(payloadLocalEulerAngles));
            followVelocity = Vector3.zero;
        }

        private void MoveSmoothed(Vector3 desiredPosition, Quaternion desiredRotation)
        {
            transform.position = positionSmoothTime > 0.0f
                ? Vector3.SmoothDamp(
                    transform.position,
                    desiredPosition,
                    ref followVelocity,
                    positionSmoothTime)
                : desiredPosition;

            var blend = rotationSharpness > 0.0f
                ? 1.0f - Mathf.Exp(-rotationSharpness * Time.deltaTime)
                : 1.0f;
            transform.rotation = Quaternion.Slerp(transform.rotation, desiredRotation, blend);
        }

        private void OnValidate()
        {
            minimumDistanceMetres = Mathf.Max(0.1f, minimumDistanceMetres);
            maximumDistanceMetres = Mathf.Max(minimumDistanceMetres, maximumDistanceMetres);
            defaultDistanceMetres = ThirdPersonCameraMath.ClampDistance(
                defaultDistanceMetres,
                minimumDistanceMetres,
                maximumDistanceMetres);
            zoomSensitivity = Mathf.Max(0.0f, zoomSensitivity);
            orbitSensitivity = Mathf.Max(0.0f, orbitSensitivity);
            positionSmoothTime = Mathf.Max(0.0f, positionSmoothTime);
            rotationSharpness = Mathf.Max(0.0f, rotationSharpness);
            collisionRadiusMetres = Mathf.Max(0.01f, collisionRadiusMetres);
            collisionPaddingMetres = Mathf.Max(0.0f, collisionPaddingMetres);
            firstPersonNearClipMetres = Mathf.Max(0.001f, firstPersonNearClipMetres);
        }

        private void ResetOrbitView()
        {
            distanceMetres = ThirdPersonCameraMath.ClampDistance(
                defaultDistanceMetres,
                minimumDistanceMetres,
                maximumDistanceMetres);
            yawOffsetDegrees = 0.0f;
            pitchDegrees = defaultPitchDegrees;
        }

        private void SnapToTarget()
        {
            UpdateOrbitView();
        }

        private float ResolveCameraDistance(
            Vector3 focus,
            Vector3 boomDirection,
            float requestedDistance)
        {
            var hitCount = Physics.SphereCastNonAlloc(
                focus,
                collisionRadiusMetres,
                boomDirection,
                collisionHits,
                requestedDistance,
                collisionLayers,
                QueryTriggerInteraction.Ignore);
            var nearestObstruction = float.PositiveInfinity;
            var robotRoot = target.root;
            for (var index = 0; index < hitCount; index++)
            {
                var hitCollider = collisionHits[index].collider;
                if (hitCollider == null || hitCollider.transform.IsChildOf(robotRoot))
                {
                    continue;
                }

                nearestObstruction = Mathf.Min(
                    nearestObstruction,
                    collisionHits[index].distance);
            }

            return ThirdPersonCameraMath.ResolveObstructedDistance(
                requestedDistance,
                nearestObstruction,
                collisionPaddingMetres,
                collisionRadiusMetres);
        }

        private void ValidateConfiguration()
        {
            if (target == null || payloadTarget == null)
            {
                throw new InvalidOperationException(
                    "Robot camera requires explicit base_link and payload targets.");
            }

            if (minimumDistanceMetres <= 0.0f ||
                maximumDistanceMetres < minimumDistanceMetres ||
                defaultDistanceMetres < minimumDistanceMetres ||
                defaultDistanceMetres > maximumDistanceMetres)
            {
                throw new InvalidOperationException(
                    "Camera distances must satisfy 0 < minimum <= default <= maximum.");
            }

            if (rearLeftLocalPosition == rearLeftLocalLookAt ||
                firstPersonNearClipMetres <= 0.0f)
            {
                throw new InvalidOperationException(
                    "Camera view offsets and first-person near clip must define valid views.");
            }
        }
    }

    public static class ThirdPersonCameraMath
    {
        public static float ClampDistance(float distance, float minimum, float maximum)
        {
            if (!float.IsFinite(distance) || !float.IsFinite(minimum) || !float.IsFinite(maximum) ||
                minimum <= 0.0f || maximum < minimum)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(distance),
                    "Camera distance bounds must be finite and satisfy 0 < minimum <= maximum.");
            }

            return Mathf.Clamp(distance, minimum, maximum);
        }

        public static float ResolveObstructedDistance(
            float requestedDistance,
            float obstructionDistance,
            float padding,
            float minimumClearance)
        {
            if (!float.IsFinite(requestedDistance) || requestedDistance <= 0.0f ||
                float.IsNaN(obstructionDistance) || obstructionDistance < 0.0f ||
                !float.IsFinite(padding) || padding < 0.0f ||
                !float.IsFinite(minimumClearance) || minimumClearance <= 0.0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(requestedDistance),
                    "Camera obstruction distances must be non-negative and finite where required.");
            }

            if (float.IsPositiveInfinity(obstructionDistance) ||
                obstructionDistance >= requestedDistance)
            {
                return requestedDistance;
            }

            return Mathf.Clamp(
                obstructionDistance - padding,
                minimumClearance,
                requestedDistance);
        }
    }

    public static class RobotCameraViewCycler
    {
        public static RobotCameraView Next(RobotCameraView current)
        {
            return current switch
            {
                RobotCameraView.Orbit => RobotCameraView.RearLeftChase,
                RobotCameraView.RearLeftChase => RobotCameraView.PayloadFirstPerson,
                RobotCameraView.PayloadFirstPerson => RobotCameraView.Orbit,
                _ => throw new ArgumentOutOfRangeException(nameof(current))
            };
        }
    }
}
