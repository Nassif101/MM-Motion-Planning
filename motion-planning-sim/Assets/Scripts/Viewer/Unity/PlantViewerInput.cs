using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MotionPlanningSim.Viewer
{
    public enum ViewerAction
    {
        RunPause,
        Step,
        SpeedDown,
        SpeedUp,
        NextCamera,
        ToggleMinimap,
        CancelGoal,
        Reset,
        ScenarioPicker,
        // 4B-1 (design 7.4)
        Overlay1,
        Overlay2,
        Overlay3,
        Overlay4,
        Overlay5,
        Overlay6,
        Overlay7,  // 4B-2: B5's prediction
        NextPreset,
        Telemetry,
        Console,
        Legend,
        Hud,
        Pace,
    }

    /// <summary>
    /// The viewer's hotkeys (design 6.4). Reset needs Shift, so a stray R next to the free camera's keys cannot wipe
    /// a recording; no hotkey fires while a text field has focus.
    /// </summary>
    public static class ViewerKeymap
    {
        static readonly Dictionary<Key, ViewerAction> Map = new Dictionary<Key, ViewerAction>
        {
            [Key.Space] = ViewerAction.RunPause,
            [Key.Period] = ViewerAction.Step,
            [Key.LeftBracket] = ViewerAction.SpeedDown,
            [Key.RightBracket] = ViewerAction.SpeedUp,
            [Key.C] = ViewerAction.NextCamera,
            [Key.M] = ViewerAction.ToggleMinimap,
            [Key.X] = ViewerAction.CancelGoal,
            [Key.R] = ViewerAction.Reset,
            [Key.F1] = ViewerAction.ScenarioPicker,
            [Key.Digit1] = ViewerAction.Overlay1,
            [Key.Digit2] = ViewerAction.Overlay2,
            [Key.Digit3] = ViewerAction.Overlay3,
            [Key.Digit4] = ViewerAction.Overlay4,
            [Key.Digit5] = ViewerAction.Overlay5,
            [Key.Digit6] = ViewerAction.Overlay6,
            [Key.Digit7] = ViewerAction.Overlay7,
            [Key.P] = ViewerAction.NextPreset,
            [Key.T] = ViewerAction.Telemetry,
            [Key.Backquote] = ViewerAction.Console,
            [Key.L] = ViewerAction.Legend,
            [Key.H] = ViewerAction.Hud,
            [Key.K] = ViewerAction.Pace,
        };

        public static IReadOnlyCollection<Key> BoundKeys => Map.Keys;

        public static ViewerAction? Resolve(Key key, bool shift, bool textFieldFocused)
        {
            if (textFieldFocused || !Map.TryGetValue(key, out var action)) return null;
            if (action == ViewerAction.Reset && !shift) return null;
            return action;
        }
    }

    /// <summary>Collects this frame's hotkey actions from the Input System keyboard.</summary>
    public sealed class PlantViewerInput : MonoBehaviour
    {
        readonly List<ViewerAction> actions = new List<ViewerAction>();

        /// <summary>The actions pressed this frame; the caller consumes them.</summary>
        public List<ViewerAction> Actions => actions;

        void Update()
        {
            var keyboard = Keyboard.current;
            if (keyboard == null) return;
            var shift = keyboard.shiftKey.isPressed;
            var typing = GUIUtility.keyboardControl != 0;
            foreach (var key in ViewerKeymap.BoundKeys)
                if (keyboard[key].wasPressedThisFrame && ViewerKeymap.Resolve(key, shift, typing) is ViewerAction a)
                    actions.Add(a);
        }
    }
}
