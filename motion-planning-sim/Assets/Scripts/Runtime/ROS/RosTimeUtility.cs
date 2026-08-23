using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;

namespace MotionPlanningSim.ROS
{
    public static class RosTimeUtility
    {
        private const double NanosecondsPerSecond = 1_000_000_000.0;

        public static TimeMsg FromSeconds(double seconds)
        {
            if (!double.IsFinite(seconds) || seconds < 0.0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(seconds),
                    "ROS simulation time must be finite and non-negative.");
            }

            var wholeSeconds = Math.Floor(seconds);
            var nanoseconds = (long)Math.Round(
                (seconds - wholeSeconds) * NanosecondsPerSecond);

            if (nanoseconds >= (long)NanosecondsPerSecond)
            {
                wholeSeconds += 1.0;
                nanoseconds = 0;
            }

#if ROS2
            return new TimeMsg(checked((int)wholeSeconds), (uint)nanoseconds);
#else
            return new TimeMsg(checked((uint)wholeSeconds), (uint)nanoseconds);
#endif
        }

        public static HeaderMsg Header(double seconds, string frameId = "")
        {
#if ROS2
            return new HeaderMsg(FromSeconds(seconds), frameId);
#else
            return new HeaderMsg(0, FromSeconds(seconds), frameId);
#endif
        }
    }

    public static class PublicationSchedule
    {
        public static bool IsDue(
            double now,
            double frequencyHz,
            ref double nextPublishTime,
            ref double previousTime)
        {
            if (!double.IsFinite(now) ||
                !double.IsFinite(frequencyHz) ||
                frequencyHz <= 0.0)
            {
                return false;
            }

            if (now < previousTime)
            {
                nextPublishTime = now;
            }

            previousTime = now;
            var period = 1.0 / frequencyHz;
            var tolerance = Math.Max(1e-6, period * 1e-4);
            if (now + tolerance < nextPublishTime)
            {
                return false;
            }

            var elapsedPeriods = Math.Max(
                1.0,
                Math.Floor((now - nextPublishTime) / period) + 1.0);
            nextPublishTime += elapsedPeriods * period;
            return true;
        }
    }
}
