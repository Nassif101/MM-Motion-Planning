using System;

namespace MotionPlanningSim.Environment
{
    /// <summary>
    /// When the Editor may switch the active scene. Opening a scene in Single mode discards the
    /// unsaved changes of the scene it replaces, so a switch is refused while the active scene
    /// is dirty, and refused for a target that is not on disk.
    /// </summary>
    public static class SceneSwitchPolicy
    {
        /// <summary>Throws InvalidOperationException when switching from the active scene to the target is not allowed.</summary>
        public static void Check(string activePath, bool activeIsDirty, string targetPath, bool targetExists)
        {
            if (string.IsNullOrEmpty(targetPath))
                throw new InvalidOperationException("No scene path was given.");
            if (activePath == targetPath)
                return;  // not a switch
            if (!targetExists)
                throw new InvalidOperationException($"The scene {targetPath} does not exist.");
            if (activeIsDirty)
                throw new InvalidOperationException(
                    $"The active scene {activePath} has unsaved changes; save or discard them before switching scenes.");
        }
    }
}
