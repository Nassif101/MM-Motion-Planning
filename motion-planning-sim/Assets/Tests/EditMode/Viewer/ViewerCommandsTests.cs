using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class ViewerCommandsTests
    {
        static void AssertJson(string expected, JObject actual) =>
            Assert.IsTrue(JToken.DeepEquals(JObject.Parse(expected), actual), actual.ToString());

        [Test]
        public void ClientRequestIncreasesPerGoalAndCancel()
        {
            var commands = new ViewerCommands();
            AssertJson("{\"cmd\":\"submit_goal\",\"target\":[1.0,2.0,0.5],\"client_request\":1}",
                commands.SubmitGoal(1.0, 2.0, 0.5));
            AssertJson("{\"cmd\":\"cancel_goal\",\"client_request\":2}", commands.CancelGoal());
            Assert.AreEqual(2, commands.LastClientRequest);
        }

        [Test]
        public void SessionCommandsMatchTheProtocol()
        {
            var commands = new ViewerCommands();
            AssertJson("{\"cmd\":\"hello\",\"schema\":3}", commands.Hello());
            AssertJson("{\"cmd\":\"list_scenarios\"}", commands.ListScenarios());
            AssertJson("{\"cmd\":\"reset\",\"scenario\":\"v2_open_yard_nav\",\"seed\":1}",
                commands.Reset("v2_open_yard_nav", 1));
            AssertJson("{\"cmd\":\"run\",\"speed\":2.0}", commands.Run(2.0));
            AssertJson("{\"cmd\":\"pause\"}", commands.Pause());
            AssertJson("{\"cmd\":\"step\",\"n\":1}", commands.Step());
            Assert.AreEqual(0, commands.LastClientRequest);
        }

        [Test]
        public void SpeedLadderClamps()
        {
            Assert.AreEqual(4.0, SpeedLadder.Up(4.0));
            Assert.AreEqual(0.25, SpeedLadder.Down(0.25));
            Assert.AreEqual(2.0, SpeedLadder.Up(1.0));
            Assert.AreEqual(0.5, SpeedLadder.Down(1.0));
            Assert.AreEqual(2.0, SpeedLadder.Up(1.5));   // an off-ladder speed moves to the next step
            Assert.AreEqual(1.0, SpeedLadder.Down(1.5));
        }
    }
}
