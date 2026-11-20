using UnityEngine;
using UnityEngine.InputSystem;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// A fly camera for video shots (design 6.3): W A S D to move, Q / E down / up, Shift four times faster, scroll
    /// to change the base speed (0.5-20 m/s); look while holding the right mouse button, or Control and the left
    /// button (a trackpad). Moves in wall time, so it works while the simulation is paused.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public sealed class FreeCamera : MonoBehaviour
    {
        [SerializeField] private float speed = 3f;
        [SerializeField] private float lookDegreesPerPixel = 0.15f;
        private float yaw, pitch;

        public float Speed => speed;

        /// <summary>True when the pose changed this frame (journaled at 10 Hz while it moves).</summary>
        public bool Moved { get; private set; }

        void OnEnable()
        {
            var e = transform.eulerAngles;
            yaw = e.y;
            pitch = e.x > 180 ? e.x - 360 : e.x;
        }

        void Update()
        {
            Moved = false;
            var keyboard = Keyboard.current;
            var mouse = Mouse.current;
            if (keyboard == null) return;

            if (mouse != null)
            {
                var scroll = mouse.scroll.ReadValue().y;
                if (scroll != 0) speed = Mathf.Clamp(speed * (scroll > 0 ? 1.25f : 0.8f), 0.5f, 20f);
                var looking = mouse.rightButton.isPressed || (keyboard.ctrlKey.isPressed && mouse.leftButton.isPressed);
                if (looking)
                {
                    var d = mouse.delta.ReadValue();
                    if (d != Vector2.zero)
                    {
                        yaw += d.x * lookDegreesPerPixel;
                        pitch = Mathf.Clamp(pitch - d.y * lookDegreesPerPixel, -89f, 89f);
                        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
                        Moved = true;
                    }
                }
            }

            var move = Vector3.zero;
            if (keyboard.wKey.isPressed) move += Vector3.forward;
            if (keyboard.sKey.isPressed) move += Vector3.back;
            if (keyboard.dKey.isPressed) move += Vector3.right;
            if (keyboard.aKey.isPressed) move += Vector3.left;
            if (keyboard.eKey.isPressed) move += Vector3.up;
            if (keyboard.qKey.isPressed) move += Vector3.down;
            if (move == Vector3.zero) return;
            var rate = speed * (keyboard.shiftKey.isPressed ? 4f : 1f);
            transform.position += transform.rotation * move.normalized * (rate * Time.unscaledDeltaTime);
            Moved = true;
        }
    }
}
