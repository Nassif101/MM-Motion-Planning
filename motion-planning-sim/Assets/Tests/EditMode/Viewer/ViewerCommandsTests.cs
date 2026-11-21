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
            var commands = new ViewerCommands(() => 0);
            AssertJson("{\"cmd\":\"submit_goal\",\"target\":[1.0,2.0,0.5],\"client_request\":1}",
                commands.SubmitGoal(1.0, 2.0, 0.5));
            AssertJson("{\"cmd\":\"cancel_goal\",\"client_request\":2}", commands.CancelGoal());
            Assert.AreEqual(2, commands.LastClientRequest);
        }

        /// <summary>4B-2: the demo server sends Prediction frames only to a client whose hello accepts them.</summary>
        [Test]
        public void HelloAcceptsPredictions()
        {
            var accepts = new ViewerCommands(() => 0).Hello()["accepts"] as JArray;
            Assert.IsNotNull(accepts);
            CollectionAssert.Contains(accepts.ToObject<string[]>(), "prediction");
        }

        [Test]
        public void SessionCommandsMatchTheProtocol()
        {
            var commands = new ViewerCommands(() => 0);
            AssertJson("{\"cmd\":\"hello\",\"schema\":3,\"accepts\":[\"prediction\"]}", commands.Hello());
            AssertJson("{\"cmd\":\"list_scenarios\"}", commands.ListScenarios());
            AssertJson("{\"cmd\":\"reset\",\"scenario\":\"v2_open_yard_nav\",\"seed\":1}",
                commands.Reset("v2_open_yard_nav", 1));
            AssertJson("{\"cmd\":\"run\",\"speed\":2.0}", commands.Run(2.0));
            AssertJson("{\"cmd\":\"pause\"}", commands.Pause());
            AssertJson("{\"cmd\":\"step\",\"n\":1}", commands.Step());
            Assert.AreEqual(0, commands.LastClientRequest);
        }

        // Task 2 review I1: the server keeps one client's request watermark for the whole session, so a relaunched
        // viewer must continue above the numbers an earlier viewer process used
        [Test]
        public void ClientRequestsContinueAcrossViewerProcesses()
        {
            var first = new ViewerCommands();
            long last = 0;
            for (var i = 0; i < 3; ++i) last = (long)first.SubmitGoal(0, 0, 0)["client_request"];
            Assert.Greater(last, 1_600_000_000_000L);  // Unix milliseconds
            System.Threading.Thread.Sleep(5);
            var relaunched = new ViewerCommands();
            Assert.Greater((long)relaunched.SubmitGoal(0, 0, 0)["client_request"], last);
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
