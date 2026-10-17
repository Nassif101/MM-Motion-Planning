using System;
using MotionPlanningSim.Environment;
using NUnit.Framework;

namespace MotionPlanningSim.Tests
{
    public class SceneSwitchPolicyTests
    {
        private const string A = "Assets/Scenes/A.unity";
        private const string B = "Assets/Scenes/B.unity";

        [Test]
        public void CleanActiveSceneMaySwitchToAnExistingScene()
        {
            Assert.DoesNotThrow(() => SceneSwitchPolicy.Check(A, false, B, true));
        }

        [Test]
        public void DirtyActiveSceneRefusesToSwitchAndNamesTheScene()
        {
            var error = Assert.Throws<InvalidOperationException>(() => SceneSwitchPolicy.Check(A, true, B, true));
            Assert.That(error.Message, Is.EqualTo(
                "The active scene Assets/Scenes/A.unity has unsaved changes; save or discard them before switching scenes."));
        }

        [Test]
        public void DirtyActiveSceneIsNotASwitchToItself()
        {
            Assert.DoesNotThrow(() => SceneSwitchPolicy.Check(A, true, A, true));
        }

        [Test]
        public void MissingTargetIsRefusedByPath()
        {
            var error = Assert.Throws<InvalidOperationException>(() => SceneSwitchPolicy.Check(A, false, B, false));
            Assert.That(error.Message, Does.Contain(B).And.Contain("does not exist"));
        }

        [Test]
        public void MissingTargetIsReportedBeforeUnsavedChanges()
        {
            var error = Assert.Throws<InvalidOperationException>(() => SceneSwitchPolicy.Check(A, true, B, false));
            Assert.That(error.Message, Does.Contain("does not exist"));
        }

        [Test]
        public void EmptyTargetIsRefused()
        {
            Assert.Throws<InvalidOperationException>(() => SceneSwitchPolicy.Check(A, false, "", false));
        }
    }
}
