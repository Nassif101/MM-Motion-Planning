using System;
using UnityEngine;

namespace MotionPlanningSim.Visualization
{
    /// <summary>
    /// Navigation state published by the ROS nav_telemetry node on /mm/telemetry (JSON in
    /// std_msgs/String). Field names match the JSON; unavailable numbers arrive as -1.
    /// </summary>
    [Serializable]
    public sealed class NavigationTelemetry
    {
        [Serializable]
        public sealed class Goal
        {
            public string status = "idle";
            public int recoveries;
            public float distance_remaining_m = -1f;
            public float eta_s = -1f;
            public float elapsed_s;
            public float[] pose = Array.Empty<float>();
        }

        [Serializable]
        public sealed class PathState
        {
            public float length_m;
            public int points;
            public float age_s = -1f;
            public float cross_track_m = -1f;
            public float cross_track_max_m;
        }

        [Serializable]
        public sealed class Monitor
        {
            public string action = "none";
            public string polygon = "";
        }

        [Serializable]
        public sealed class Lidar
        {
            public float rate_hz;
            public int gaps_over_0p5s;
            public float max_gap_s;
        }

        [Serializable]
        public sealed class Event
        {
            public float t;
            public string text = "";
        }

        public float t;
        public string scenario = "";
        public string footprint_profile = "";
        public string behavior_tree = "";
        public string planner = "";
        public string controller = "";
        public int goal_seq;
        public Goal goal = new Goal();
        public PathState path = new PathState();
        public Monitor monitor = new Monitor();
        public Lidar lidar = new Lidar();
        public Event[] events = Array.Empty<Event>();

        /// <summary>Parses one telemetry message; returns null for malformed JSON.</summary>
        public static NavigationTelemetry Parse(string json)
        {
            if (string.IsNullOrWhiteSpace(json))
            {
                return null;
            }

            try
            {
                return JsonUtility.FromJson<NavigationTelemetry>(json);
            }
            catch (ArgumentException)
            {
                return null;
            }
        }
    }

    /// <summary>Latest telemetry with a wall-clock receipt time for staleness.</summary>
    public sealed class TelemetryFeed
    {
        public const double StaleAfterSeconds = 1.5;

        public NavigationTelemetry Latest { get; private set; }
        public double ReceivedAt { get; private set; } = double.NegativeInfinity;
        public long Received { get; private set; }
        public long Rejected { get; private set; }

        public void Accept(string json, double wallTime)
        {
            var parsed = NavigationTelemetry.Parse(json);
            if (parsed == null)
            {
                Rejected++;
                return;
            }

            Latest = parsed;
            ReceivedAt = wallTime;
            Received++;
        }

        public bool IsStale(double wallTime) => Latest == null || wallTime - ReceivedAt > StaleAfterSeconds;

        public double Age(double wallTime) => Latest == null ? double.PositiveInfinity : wallTime - ReceivedAt;
    }
}
