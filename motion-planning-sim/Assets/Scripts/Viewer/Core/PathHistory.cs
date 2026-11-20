using System.Collections.Generic;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>The travelled path (4B-1 overlay 2): points at least <c>minSpacingM</c> apart, the newest <c>cap</c> kept.</summary>
    public sealed class PathHistory
    {
        readonly int cap;
        readonly double minSpacingM;
        readonly List<(double x, double y)> points = new List<(double, double)>();

        public PathHistory(int cap = 5000, double minSpacingM = 0.02)
        {
            this.cap = cap;
            this.minSpacingM = minSpacingM;
        }

        public IReadOnlyList<(double x, double y)> Points => points;

        /// <summary>Incremented on every change, so a presenter redraws only when the path changed.</summary>
        public int Version { get; private set; }

        public void Add(double x, double y)
        {
            if (points.Count > 0)
            {
                var (lx, ly) = points[points.Count - 1];
                if ((x - lx) * (x - lx) + (y - ly) * (y - ly) < minSpacingM * minSpacingM) return;
            }
            points.Add((x, y));
            if (points.Count > cap) points.RemoveRange(0, points.Count - cap);
            ++Version;
        }

        public void Clear()
        {
            points.Clear();
            ++Version;
        }
    }
}
