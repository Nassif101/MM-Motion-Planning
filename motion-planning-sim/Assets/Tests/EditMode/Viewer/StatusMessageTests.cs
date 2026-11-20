using MotionPlanningSim.Viewer.Core;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>Final review I1: a HUD message is in every recorded frame, so it must not outlive its cause.</summary>
    public sealed class StatusMessageTests
    {
        [Test]
        public void AMessageExpiresAfterItsHoldTime()
        {
            var m = new StatusMessage(5.0);
            Assert.AreEqual("", m.Text(0.0));
            m.Set("submit_goal: rejected occupied", 10.0);
            Assert.AreEqual("submit_goal: rejected occupied", m.Text(14.9));
            Assert.AreEqual("", m.Text(15.1));
        }

        [Test]
        public void ANewMessageRestartsTheHoldAndClearEndsIt()
        {
            var m = new StatusMessage(5.0);
            m.Set("run: not connected", 0.0);
            m.Set("cancel_goal: rejected no_goal", 4.0);
            Assert.AreEqual("cancel_goal: rejected no_goal", m.Text(8.5));
            m.Clear();
            Assert.AreEqual("", m.Text(8.6));
        }
    }
}
