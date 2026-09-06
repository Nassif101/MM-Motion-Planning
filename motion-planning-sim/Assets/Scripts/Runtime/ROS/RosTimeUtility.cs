using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;

namespace MotionPlanningSim.ROS
{
    public static class RosTimeUtility
    {
        private const double NanosecondsPerSecond = 1_000_000_000.0;
        public static double PhysicsTimeSeconds { get; internal set; }

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

    /// <summary>Exact ROS periods, independent of Unity's floating-point fixed-time accumulation.</summary>
    public sealed class PhysicsStepClock
    {
        public long Nanoseconds { get; private set; }
        private bool initialized;
        public double Advance(double initialTime, float fixedDeltaTime)
        {
            // Unity may report a configured 0.02 s step as 0.0199999921 s.
            // Define the ROS physics period to microsecond precision, then accumulate
            // integer nanoseconds; otherwise exact ROS 20 ms timers can skip a tick.
            long step=(long)Math.Round(fixedDeltaTime*1_000_000.0)*1000;
            if(step<=0 || !double.IsFinite(initialTime) || initialTime<0)
                throw new ArgumentOutOfRangeException(nameof(fixedDeltaTime));
            if(!initialized)
            {
                Nanoseconds=(long)Math.Round(initialTime/fixedDeltaTime)*step;
                initialized=true;
            }
            else Nanoseconds=checked(Nanoseconds+step);
            return Nanoseconds*1e-9;
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
