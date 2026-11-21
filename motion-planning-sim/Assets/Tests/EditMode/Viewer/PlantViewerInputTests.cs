using System.Collections.Generic;
using System.Linq;
using MotionPlanningSim.Viewer;
using MotionPlanningSim.Viewer.Core;
using NUnit.Framework;
using UnityEngine.InputSystem;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class PlantViewerInputTests
    {
        static readonly Dictionary<ViewerAction, Key> Keys = new Dictionary<ViewerAction, Key>
        {
            [ViewerAction.RunPause] = Key.Space,
            [ViewerAction.Step] = Key.Period,
            [ViewerAction.SpeedDown] = Key.LeftBracket,
            [ViewerAction.SpeedUp] = Key.RightBracket,
            [ViewerAction.NextCamera] = Key.C,
            [ViewerAction.ToggleMinimap] = Key.M,
            [ViewerAction.CancelGoal] = Key.X,
            [ViewerAction.Reset] = Key.R,
            [ViewerAction.ScenarioPicker] = Key.F1,
            // 4B-1 (design 7.4)
            [ViewerAction.Overlay1] = Key.Digit1,
            [ViewerAction.Overlay2] = Key.Digit2,
            [ViewerAction.Overlay3] = Key.Digit3,
            [ViewerAction.Overlay4] = Key.Digit4,
            [ViewerAction.Overlay5] = Key.Digit5,
            [ViewerAction.Overlay6] = Key.Digit6,
            [ViewerAction.Overlay7] = Key.Digit7,  // 4B-2
            [ViewerAction.NextPreset] = Key.P,
            [ViewerAction.Telemetry] = Key.T,
            [ViewerAction.Console] = Key.Backquote,
            [ViewerAction.Legend] = Key.L,
            [ViewerAction.Hud] = Key.H,
            [ViewerAction.Pace] = Key.K,
        };

        // 4B-2: key 7 toggles B5's prediction, the seventh overlay
        [Test]
        public void Key7TogglesThePrediction()
        {
            Assert.AreEqual(ViewerAction.Overlay7, ViewerKeymap.Resolve(Key.Digit7, false, false));
            Assert.AreEqual(Overlay.Prediction, PlantViewerBootstrap.OverlayOf(ViewerAction.Overlay7));
            Assert.AreEqual(Overlay.Route, PlantViewerBootstrap.OverlayOf(ViewerAction.Overlay1));
            Assert.AreEqual(Overlay.Lidar, PlantViewerBootstrap.OverlayOf(ViewerAction.Overlay6));
            Assert.IsNull(PlantViewerBootstrap.OverlayOf(ViewerAction.NextPreset));
            Assert.IsNull(ViewerKeymap.Resolve(Key.Digit7, false, true), "not while typing");
        }

        // 4B-1: typing in the console's search field must not toggle overlays or panels
        [Test]
        public void NothingFiresWhileTheConsoleSearchHasFocus()
        {
            foreach (var k in new[] { Key.Digit1, Key.Digit6, Key.P, Key.T, Key.Backquote, Key.L, Key.H, Key.K })
                Assert.IsNull(ViewerKeymap.Resolve(k, false, true), k.ToString());
        }

        [Test]
        public void ResetNeedsShift()
        {
            Assert.IsNull(ViewerKeymap.Resolve(Key.R, false, false));
            Assert.AreEqual(ViewerAction.Reset, ViewerKeymap.Resolve(Key.R, true, false));
        }

        [Test]
        public void HotkeysIgnoredWhileTyping()
        {
            Assert.IsNull(ViewerKeymap.Resolve(Key.Space, false, true));
            Assert.IsNull(ViewerKeymap.Resolve(Key.R, true, true));
        }

        [Test]
        public void EveryActionHasOneKey()
        {
            foreach (var pair in Keys)
                Assert.AreEqual(pair.Key, ViewerKeymap.Resolve(pair.Value, pair.Key == ViewerAction.Reset, false), pair.Key.ToString());
            var bound = System.Enum.GetValues(typeof(Key)).Cast<Key>().Where(k => k != Key.None).Distinct()
                .Count(k => ViewerKeymap.Resolve(k, true, false) != null || ViewerKeymap.Resolve(k, false, false) != null);
            Assert.AreEqual(Keys.Count, bound, "no other key triggers an action");
            CollectionAssert.AreEquivalent(Keys.Values, ViewerKeymap.BoundKeys);
            // the free camera's keys never trigger an action
            foreach (var k in new[] { Key.W, Key.A, Key.S, Key.D, Key.Q, Key.E })
                Assert.IsNull(ViewerKeymap.Resolve(k, true, false), k.ToString());
        }
    }
}
