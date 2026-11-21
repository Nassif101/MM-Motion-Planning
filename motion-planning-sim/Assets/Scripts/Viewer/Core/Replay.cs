using System;
using System.Collections.Generic;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// The replay's simulation time (spec 4C): it advances with wall time times the speed while playing, stops at the
    /// log's end, and restarts at its start. Starts paused at the start.
    /// </summary>
    public sealed class ReplayClock
    {
        readonly double startT, endT;

        public ReplayClock(double startT, double endT)
        {
            this.startT = startT;
            this.endT = Math.Max(startT, endT);
            T = startT;
        }

        public double T { get; private set; }
        public bool Playing { get; set; }
        public double Speed { get; set; } = 1.0;
        public bool AtEnd => T >= endT;

        public void Advance(double wallDt)
        {
            if (!Playing || wallDt <= 0.0) return;
            T = Math.Min(endT, T + wallDt * Speed);
            if (AtEnd) Playing = false;
        }

        public void Restart()
        {
            T = startT;
            Playing = false;
        }

        /// <summary>Sets the time directly (the render steps it per frame), within the log.</summary>
        public void Set(double t) => T = Math.Max(startT, Math.Min(endT, t));
    }

    /// <summary>A run log's frames, applied in order up to a time, each once (spec 4C).</summary>
    public sealed class ReplayFeed
    {
        readonly RunLog log;
        int next;

        public ReplayFeed(RunLog log) => this.log = log;

        public RunLog Log => log;

        /// <summary>Applies every frame not yet applied whose time is at or before <paramref name="t"/>; a frame without a
        /// time (the Hello, events without one) goes with the frames around it.</summary>
        public void FeedUntil(double t, Action<Frame> apply)
        {
            while (next < log.Frames.Count)
            {
                var f = log.Frames[next];
                var ft = (double?)f.Header["t"];
                if (ft.HasValue && ft.Value > t + 1e-9) break;
                apply(f);
                ++next;
            }
        }

        public void Reset() => next = 0;
    }

    /// <summary>
    /// Holds F2's plans and B5's predictions until the displayed time reaches theirs (spec 4C, the 4B deferred item:
    /// overlays drawn about 0.1 s ahead of the robot). Every other frame passes at once.
    /// </summary>
    public sealed class FrameGate
    {
        readonly List<Frame> held = new List<Frame>();

        public void Offer(Frame frame, Action<Frame> apply)
        {
            if (frame.Type == FrameType.Plans || frame.Type == FrameType.Prediction) held.Add(frame);
            else apply(frame);
        }

        public void Release(double displayT, Action<Frame> apply)
        {
            var kept = 0;
            for (var i = 0; i < held.Count; ++i)
            {
                var t = (double?)held[i].Header["t"] ?? double.NegativeInfinity;
                if (t <= displayT + 1e-9) apply(held[i]);
                else held[kept++] = held[i];
            }
            held.RemoveRange(kept, held.Count - kept);
        }

        public void Clear() => held.Clear();
    }
}
