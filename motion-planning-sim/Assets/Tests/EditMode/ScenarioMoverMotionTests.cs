using System;
using MotionPlanningSim.Environment;
using NUnit.Framework;

namespace MotionPlanningSim.Tests
{
    public class ScenarioMoverMotionTests
    {
        // Worker crossing the open band: (11, 2.5) -> (11, -2.5) at 0.8 m/s, trigger at 2.5 m.
        private static ScenarioMoverMotion Worker(int crossings = 1) =>
            new ScenarioMoverMotion(11.0, 2.5, 11.0, -2.5, 0.8, 2.5, crossings);

        private static void Step(ScenarioMoverMotion motion, double time, double robotX, double robotY)
        {
            var (x, y) = motion.Next(time, robotX, robotY);
            motion.Commit(time, x, y);
        }

        [Test]
        public void WaitsAtTheStartUntilTheRobotIsWithinTheTriggerDistance()
        {
            var motion = Worker();
            for (var t = 0.0; t < 5.0; t += 0.02)
            {
                Step(motion, t, 16.0 - 0.3 * t, 0.0);  // robot approaching from x = 16
            }

            // The robot reaches x = 13.5 (2.5 m from the midpoint (11, 0)) at t = 8.33 s.
            Assert.That(motion.TriggerTime, Is.Null);
            Assert.That(motion.X, Is.EqualTo(11.0));
            Assert.That(motion.Y, Is.EqualTo(2.5));
        }

        [Test]
        public void CrossesOnceAtConstantSpeedAndStaysAtTheEnd()
        {
            var motion = Worker();
            Step(motion, 10.0, 13.4, 0.0);  // triggers
            Assert.That(motion.TriggerTime, Is.EqualTo(10.0));
            Step(motion, 12.5, 13.0, 0.0);  // 2.0 m walked
            Assert.That(motion.Y, Is.EqualTo(0.5).Within(1e-9));
            Step(motion, 20.0, 12.0, 0.0);  // 8.0 m would exceed the 5.0 m crossing
            Assert.That(motion.Finished, Is.True);
            Assert.That(motion.Y, Is.EqualTo(-2.5).Within(1e-9));
            Assert.That(motion.Walked, Is.EqualTo(5.0).Within(1e-9));
        }

        [Test]
        public void TwoCrossingsReturnToTheStart()
        {
            var motion = Worker(crossings: 2);
            Assert.That(motion.PositionAt(7.5).y, Is.EqualTo(0.0).Within(1e-9));  // halfway back
            Assert.That(motion.PositionAt(10.0).y, Is.EqualTo(2.5).Within(1e-9));
            Assert.That(motion.PositionAt(99.0).y, Is.EqualTo(2.5).Within(1e-9));
        }

        [Test]
        public void HeldTimeCountsAsBlockedAndNotAsWalking()
        {
            var motion = Worker();
            Step(motion, 10.0, 13.0, 0.0);
            motion.Next(11.0, 13.0, 0.0);
            motion.Hold(11.0);  // robot in the way for 1 s
            Assert.That(motion.BlockedSeconds, Is.EqualTo(1.0).Within(1e-9));
            Assert.That(motion.Walked, Is.EqualTo(0.0));
            Step(motion, 11.5, 13.0, 0.0);
            Assert.That(motion.Walked, Is.EqualTo(0.4).Within(1e-9));
        }

        [Test]
        public void RejectsInvalidMotion()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new ScenarioMoverMotion(0, 0, 0, 0.05, 0.8, 2.5, 1));
            Assert.Throws<ArgumentOutOfRangeException>(() => new ScenarioMoverMotion(0, 0, 0, 2, 0.0, 2.5, 1));
            Assert.Throws<ArgumentOutOfRangeException>(() => new ScenarioMoverMotion(0, 0, 0, 2, 0.8, 2.5, 0));
            Assert.Throws<ArgumentOutOfRangeException>(() => new ScenarioMoverMotion(double.NaN, 0, 0, 2, 0.8, 2.5, 1));
        }
    }
}
