using System;
using MotionPlanningSim.Viewer.Core;
using MotionPlanningSim.Visualization;
using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// Spec 4C: replay and directing. With <c>-replay FILE</c> the viewer plays a run log instead of connecting to a
    /// server: Space plays and pauses, <c>[</c> and <c>]</c> change the speed, Shift+R restarts the run and starts a new
    /// take (journal file). Goals, cancel and scenario resets are off. In both modes F2's plans and B5's predictions
    /// wait for the displayed time (<see cref="FrameGate"/>), and the orbit camera's parameters are journaled while they
    /// change.
    /// </summary>
    public sealed partial class PlantViewerBootstrap
    {
        public const int DirectingWidth = 1920, DirectingHeight = 1080;

        private readonly FrameGate gate = new FrameGate();
        private RunLog replayLog;
        private ReplayClock replayClock;
        private ReplayFeed replayFeed;
        private string replayError;
        private (float yaw, float pitch, float distance) lastOrbit, frameOrbit;
        private double lastOrbitJournal, orbitChangedAt, freeMovedAt;
        private bool orbitMoving, freeMoving;
        const double StillAfterS = 0.15;  // unchanged this long (wall time): the move ended (Task 3 review I1)

        /// <summary>The viewer plays a run log (spec 4C).</summary>
        public bool Replaying => replayLog != null;

        public RunLog ReplayLog => replayLog;
        public ReplayClock ReplayClock => replayClock;

        /// <summary>Plays a run log (<c>-replay</c>; public for automation and the Editor's live checks).</summary>
        public void OpenReplay(string path)
        {
            try
            {
                replayLog = RunLogReader.Load(path);
                replayClock = new ReplayClock(replayLog.StartT, replayLog.EndT);
                replayFeed = new ReplayFeed(replayLog);
            }
            catch (Exception e)
            {
                replayError = $"replay {path}: {e.Message}";
                replayLog = null;
            }
            Screen.SetResolution(DirectingWidth, DirectingHeight, FullScreenMode.Windowed);  // the video's size (design 5.3)
        }

        /// <summary>The replay's frames up to its clock, through the gate; the robot at the clock's time exactly.</summary>
        bool UpdateReplay(double wall, out RobotSample sample)
        {
            replayClock.Advance(Time.unscaledDeltaTime);
            replayFeed.FeedFor(replayClock.T, f => Route(f, wall));
            gate.Release(replayClock.T, f => state.Apply(f, wall));
            return state.TrySampleAt(replayClock.T, out sample);
        }

        /// <summary>One frame from the stream or the log: scans to the lidar overlay, the rest through the gate.</summary>
        void Route(Frame frame, double wall)
        {
            if (frame.Type == FrameType.Scan) lidar.Queue(frame, overlays[Overlay.Lidar]);
            else gate.Offer(frame, f => state.Apply(f, wall));
        }

        /// <summary>Shift+R in replay: the run from its start, as a new take.</summary>
        void RestartReplay()
        {
            replayClock.Restart();
            replayFeed.Reset();
            gate.Clear();
            travelledPath.Clear();
            lidar.Clear();
            lastSampleT = double.NaN;
            displayedT = double.NaN;
            appliedHello = null;  // the log's Hello applies again: OnSession opens a new journal file, a new take
        }

        /// <summary>The journal header's extra fields for a directing pass: the log it was made from.</summary>
        JObject ReplayIdentity() =>
            Replaying ? new JObject { ["log"] = replayLog.Name, ["log_sha256"] = replayLog.Sha256 } : null;

        /// <summary>
        /// The orbit camera's parameters, journaled at 10 Hz of wall time while they change (spec 4C), and a recenter
        /// line when F recenters it. Mouse orbit and zoom are blocked while the pointer is over the minimap or a panel.
        /// </summary>
        void UpdateCameraJournal(double wall, Rect[] panels)
        {
            var mouse = Mouse.current;
            robotCamera.InputBlocked = mouse != null && PanelLayout.PointerOver(mouse.position.ReadValue(), Screen.height, panels);
            if (view != ViewerCamera.Orbit) return;
            var orbit = robotCamera.Orbit;
            if (!orbit.Equals(frameOrbit))
            {
                frameOrbit = orbit;
                orbitChangedAt = wall;
                orbitMoving = true;
            }
            if (orbitMoving && wall - orbitChangedAt >= StillAfterS)  // the move ended: its resting pose, marked still
            {
                orbitMoving = false;
                lastOrbit = orbit;
                lastOrbitJournal = wall;
                JournalOrbit(still: true);
            }
            else if (orbitMoving && !orbit.Equals(lastOrbit) && wall - lastOrbitJournal >= 1.0 / JournalPoseHz)
            {
                lastOrbit = orbit;
                lastOrbitJournal = wall;
                JournalOrbit();
            }
        }

        void JournalOrbit(bool still = false)
        {
            var o = robotCamera.Orbit;
            var line = new JObject { ["yaw"] = o.yaw, ["pitch"] = o.pitch, ["distance"] = o.distance };
            if (still) line["still"] = true;
            Journal("orbit", line);
        }

        /// <summary>The free camera's pose at 10 Hz while it moves, and once more, marked still, where it stops.</summary>
        void UpdateFreeCameraJournal(double wall)
        {
            if (view != ViewerCamera.Free) return;
            if (freeCamera.Moved)
            {
                freeMoving = true;
                freeMovedAt = wall;
                if (wall - lastPoseJournal >= 1.0 / JournalPoseHz)
                {
                    lastPoseJournal = wall;
                    JournalFree(still: false);
                }
            }
            else if (freeMoving && wall - freeMovedAt >= StillAfterS)
            {
                freeMoving = false;
                lastPoseJournal = wall;
                JournalFree(still: true);
            }
        }

        void JournalFree(bool still)
        {
            var t = freeCamera.transform;
            var line = new JObject
            {
                ["position"] = new JArray(t.position.x, t.position.y, t.position.z),
                ["rotation"] = new JArray(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
            };
            if (still) line["still"] = true;
            Journal("free_camera", line, thinnable: !still);  // the resting pose is never thinned
        }

        void OnRecentered()
        {
            // F switches the scene camera to orbit from any of its views (Task 4 review I1): the viewer follows, so the
            // journal and the render show the view the director saw
            if (view != ViewerCamera.Orbit) SetCamera(ViewerCamera.Orbit);
            Journal("recenter");
            lastOrbit = frameOrbit = robotCamera.Orbit;
            orbitMoving = false;
            JournalOrbit(still: true);  // a recenter is a cut to a resting pose
        }

        /// <summary>A new journal file starts with the camera as it is (spec 4C: a take's render starts from it).</summary>
        void JournalCamera()
        {
            Journal("camera", new JObject { ["view"] = view.ToString() });
            if (view == ViewerCamera.Orbit) JournalOrbit(still: true);
            else if (view == ViewerCamera.Free) JournalFree(still: true);
        }

        /// <summary>The replay's keys; true when the action was a replay action.</summary>
        bool PerformReplay(ViewerAction action)
        {
            if (!Replaying) return false;
            switch (action)
            {
                case ViewerAction.RunPause:
                    if (replayClock.AtEnd) Say("replay: at the end (Shift+R restarts)");
                    else replayClock.Playing = !replayClock.Playing;
                    return true;
                case ViewerAction.SpeedDown:
                case ViewerAction.SpeedUp:
                    replayClock.Speed = action == ViewerAction.SpeedUp ? SpeedLadder.Up(replayClock.Speed) : SpeedLadder.Down(replayClock.Speed);
                    return true;
                case ViewerAction.Reset:
                    RestartReplay();
                    return true;
                case ViewerAction.Step:
                case ViewerAction.CancelGoal:
                case ViewerAction.ScenarioPicker:
                    Say($"{action}: off in replay");
                    return true;
            }
            return false;
        }

        string ReplayStatus()
        {
            var hello = state.SessionHello;
            var session = hello == null ? "no session" : $"{(string)hello["scenario"]} seed {(int?)hello["seed"]}";
            return $"Replay {replayLog.Name}{(replayLog.EndsEarly ? " (ends early)" : "")} | {session} | " +
                $"t {(double.IsNaN(displayedT) ? "-" : displayedT.ToString("F2"))} / {replayLog.EndT:F2} s | " +
                $"speed {replayClock.Speed:0.##}x | {fps:0} FPS | {view}" + (replayClock.Playing ? "" : replayClock.AtEnd ? " | end" : " | paused");
        }
    }
}
