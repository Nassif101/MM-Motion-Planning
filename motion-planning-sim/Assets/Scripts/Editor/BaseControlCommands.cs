using System;
using MotionPlanningSim.Control;
using Unity.Pipeline.Commands;
using UnityEngine;

namespace MotionPlanningSim.Editor
{
    // Switches of the skid-steer base controller for experiment runs (tools/run_nav_scenario.py).
    public static class BaseControlCommands
    {
        // Play only: a Play epoch starts from the edit scene's setting (on by default); runs set it explicitly.
        [CliCommand("base_friction_feedforward", "Switch the base's breakaway friction feedforward on or off in Play",
            MainThreadRequired=true)]
        public static object FrictionFeedforward(bool enabled)
        {
            if (!Application.isPlaying) throw new InvalidOperationException("Enter Play first.");
            var b = UnityEngine.Object.FindFirstObjectByType<SkidSteerBaseController>()
                ?? throw new InvalidOperationException("Base controller missing.");
            b.FrictionFeedforward = enabled;
            return new {enabled};
        }
    }
}
