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

        /// <summary>How far the feed runs ahead of the displayed time: at least one State (the plant's 0.02 s), so the
        /// robot is sampled between two States at the clock's time (Task 4 live check). Plans and predictions still wait
        /// for the displayed time in the <see cref="FrameGate"/>.</summary>
        public const double LeadS = 0.1;

        /// <summary>The frames needed to draw <paramref name="displayT"/>: up to <see cref="LeadS"/> beyond it.</summary>
        public void FeedFor(double displayT, Action<Frame> apply) => FeedUntil(displayT + LeadS, apply);
    }

    /// <summary>
    /// Holds F2's plans and B5's predictions until the displayed time reaches theirs (spec 4C, the 4B deferred item:
    /// overlays drawn about 0.1 s ahead of the robot). The stream's order is kept (Task 2 review I1): States, Scans and
    /// Hellos pass at once (a Hello drops what is held: the old session's frames never land in the new one); every
    /// other frame waits in one queue behind a held plan, at its own time or, without one, at the last State's time
    /// before it. So a goal's end that came after its plan is applied after it.
    /// </summary>
    public sealed class FrameGate
    {
        readonly Queue<(double t, Frame frame)> held = new Queue<(double, Frame)>();
        double lastStateT = double.NegativeInfinity;

        public void Offer(Frame frame, Action<Frame> apply)
        {
            switch (frame.Type)
            {
                case FrameType.Hello:
                    Clear();
                    apply(frame);
                    return;
                case FrameType.State:
                    lastStateT = (double?)frame.Header["t"] ?? lastStateT;
                    apply(frame);
                    return;
                case FrameType.Scan:
                    apply(frame);
                    return;
            }
            var t = frame.Type == FrameType.Plans || frame.Type == FrameType.Prediction
                ? (double?)frame.Header["t"] ?? lastStateT
                : lastStateT;
            if (held.Count == 0 && t == double.NegativeInfinity) apply(frame);  // before any State: nothing to wait for
            else held.Enqueue((t, frame));
        }

        public void Release(double displayT, Action<Frame> apply)
        {
            while (held.Count > 0 && held.Peek().t <= displayT + 1e-9) apply(held.Dequeue().frame);
        }

        public void Clear()
        {
            held.Clear();
            lastStateT = double.NegativeInfinity;
        }
    }
}
