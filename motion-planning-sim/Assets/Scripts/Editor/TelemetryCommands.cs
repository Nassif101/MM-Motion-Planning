using System;
using MotionPlanningSim.Visualization;
using Unity.Pipeline.Commands;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    /// <summary>Scriptable control of the telemetry window (e.g. for performance measurements).</summary>
    public static class TelemetryCommands
    {
        [CliCommand("telemetry_window",
            "Show or hide the telemetry window and path overlay in Play; reports feed statistics",
            MainThreadRequired = true)]
        public static object SetWindow(bool visible)
        {
            if (!Application.isPlaying)
            {
                throw new InvalidOperationException("Enter Play first.");
            }

            var window = UnityEngine.Object.FindFirstObjectByType<TelemetryWindow>()
                ?? throw new InvalidOperationException("No TelemetryWindow in the scene (run configure_mobile_manipulator).");
            window.SetVisible(visible);
            return new
            {
                visible = window.Visible,
                received = window.Feed.Received,
                rejected = window.Feed.Rejected,
                stale = window.Feed.IsStale(Time.realtimeSinceStartupAsDouble),
                scenario = window.Feed.Latest?.scenario
            };
        }
    }
}
