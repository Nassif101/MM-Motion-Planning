using System;

namespace MotionPlanningSim.Environment
{
    /// <summary>
    /// Motion of a scenario mover (a worker or cart crossing the robot's route) in the ROS map
    /// frame. The mover waits at its start until the robot comes within the trigger distance
    /// of the segment's midpoint, then walks the segment at constant speed for the given
    /// number of crossings (alternating direction) and stays at the last end. Time is physics
    /// time, so runs with the same robot motion repeat exactly. A blocked mover holds its
    /// position; the held time does not count as walking.
    /// </summary>
    public sealed class ScenarioMoverMotion
    {
        public double StartX { get; }
        public double StartY { get; }
        public double EndX { get; }
        public double EndY { get; }
        public double Speed { get; }
        public double TriggerDistance { get; }
        public int Crossings { get; }

        public double TriggerX => (StartX + EndX) / 2;
        public double TriggerY => (StartY + EndY) / 2;
        public double Length => Math.Sqrt((EndX - StartX) * (EndX - StartX) + (EndY - StartY) * (EndY - StartY));

        public double? TriggerTime { get; private set; }
        public double Walked { get; private set; }
        public double BlockedSeconds { get; private set; }
        public bool Finished => Walked >= Crossings * Length;
        public double X { get; private set; }
        public double Y { get; private set; }

        private double? previousTime;

        public ScenarioMoverMotion(double startX, double startY, double endX, double endY,
            double speed, double triggerDistance, int crossings)
        {
            if (!(double.IsFinite(startX) && double.IsFinite(startY) && double.IsFinite(endX) && double.IsFinite(endY)))
            {
                throw new ArgumentOutOfRangeException(nameof(startX), "Mover end points must be finite.");
            }

            if (!(speed > 0 && speed <= 2.0))
            {
                throw new ArgumentOutOfRangeException(nameof(speed), "Mover speed must be in (0, 2] m/s.");
            }

            if (!(triggerDistance > 0 && triggerDistance <= 20.0))
            {
                throw new ArgumentOutOfRangeException(nameof(triggerDistance), "Trigger distance must be in (0, 20] m.");
            }

            if (crossings < 1 || crossings > 10)
            {
                throw new ArgumentOutOfRangeException(nameof(crossings), "Crossings must be 1 to 10.");
            }

            (StartX, StartY, EndX, EndY) = (startX, startY, endX, endY);
            (Speed, TriggerDistance, Crossings) = (speed, triggerDistance, crossings);
            if (!(Length >= 0.1))
            {
                throw new ArgumentOutOfRangeException(nameof(endX), "Mover segment must be at least 0.1 m long.");
            }

            (X, Y) = (startX, startY);
        }

        /// <summary>
        /// Position the mover would take at <paramref name="time"/>, given the robot position.
        /// Call <see cref="Commit"/> to move there, or <see cref="Hold"/> when it is blocked.
        /// </summary>
        public (double x, double y) Next(double time, double robotX, double robotY)
        {
            if (TriggerTime == null)
            {
                var dx = robotX - TriggerX;
                var dy = robotY - TriggerY;
                if (Math.Sqrt(dx * dx + dy * dy) > TriggerDistance)
                {
                    previousTime = time;
                    return (X, Y);
                }

                TriggerTime = time;
                previousTime = time;
            }

            var step = previousTime.HasValue ? Math.Max(0.0, time - previousTime.Value) : 0.0;
            return PositionAt(Math.Min(Walked + Speed * step, Crossings * Length));
        }

        public void Commit(double time, double x, double y)
        {
            if (TriggerTime != null && previousTime.HasValue)
            {
                Walked = Math.Min(Walked + Speed * Math.Max(0.0, time - previousTime.Value), Crossings * Length);
            }

            previousTime = time;
            (X, Y) = (x, y);
        }

        public void Hold(double time)
        {
            if (TriggerTime != null && previousTime.HasValue && !Finished)
            {
                BlockedSeconds += Math.Max(0.0, time - previousTime.Value);
            }

            previousTime = time;
        }

        /// <summary>Position after walking <paramref name="distance"/> along the alternating crossings.</summary>
        public (double x, double y) PositionAt(double distance)
        {
            var length = Length;
            var clamped = Math.Max(0.0, Math.Min(distance, Crossings * length));
            var leg = Math.Min((int)Math.Floor(clamped / length), Crossings - 1);
            var along = (clamped - leg * length) / length;
            if (leg % 2 == 1)
            {
                along = 1.0 - along;  // return leg: end -> start
            }

            return (StartX + along * (EndX - StartX), StartY + along * (EndY - StartY));
        }
    }
}
