using System.Collections.Generic;
using System.Linq;
using MotionPlanningSim.Viewer;
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
        };

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
