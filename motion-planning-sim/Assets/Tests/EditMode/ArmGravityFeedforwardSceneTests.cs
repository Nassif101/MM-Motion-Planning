using System.IO;
using NUnit.Framework;
namespace MotionPlanningSim.Tests
{
    // The gravity feedforward is on by default (Phase 4B-B, 2026-10-23); a scene saved from an Editor session that
    // still held the old default would switch it off for every Play that does not set it explicitly.
    public class ArmGravityFeedforwardSceneTests
    {
        [TestCase("Assets/Scenes/ConstructionSiteV1.unity")]
        [TestCase("Assets/Scenes/ConstructionSiteV2.unity")]
        public void NoSiteSceneSwitchesTheFeedforwardOff(string scene)
        {
            Assert.That(File.ReadAllText(scene), Does.Not.Contain("gravityFeedforward: 0"));
        }
    }
}
