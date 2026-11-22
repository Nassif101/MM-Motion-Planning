using System;
using System.Collections;
using System.IO;
using MotionPlanningSim.Viewer.Core;
using MotionPlanningSim.Visualization;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// Spec 4C: render mode (<c>-render LOG -journal FILE -out DIR</c>). The log replays at a fixed 60 frames per video
    /// second (<c>Time.captureFramerate</c>), frame n at <c>from + n speed / 60</c> of simulation time. The journal's
    /// direction (camera, orbit, free-camera pose, presets, overlay and panel toggles) is applied at each frame's time,
    /// and the screen, panels included, is saved as <c>frame_NNNNNN.jpg</c> (or <c>.png</c>). A mismatched journal is
    /// refused before the first frame. The player writes <c>render-summary.json</c> and quits.
    /// </summary>
    public sealed partial class PlantViewerBootstrap
    {
        const int WarmUpFrames = 30;  // HDRP's exposure and temporal history settle on the first frame's picture

        private JournalTrack renderTrack;
        private int renderCount, renderFrame;
        private Func<int, double> renderTimeOf;
        private string renderWarning = "";
        private (int w, int h) renderSize;

        /// <summary>The viewer renders a directed log to frames (spec 4C).</summary>
        public bool Rendering => options != null && options.Render != null;

        void OpenRender()
        {
            try
            {
                Directory.CreateDirectory(options.RenderOut);
            }
            catch (Exception e)  // nowhere to write: say so in the log and quit, never wait
            {
                Debug.LogError($"render: -out {options.RenderOut}: {e.Message}");
                Application.Quit(2);
                return;
            }
            OpenReplay(options.Render);
            if (replayLog == null)
            {
                FinishRender(false, error: replayError);
                return;
            }
            try
            {
                renderTrack = JournalTrack.Load(options.RenderJournal);
            }
            catch (Exception e)
            {
                FinishRender(false, error: $"journal {options.RenderJournal}: {e.Message}");
                return;
            }
            var (ok, warning, refusal) = RenderPlan.Check(replayLog, renderTrack);
            if (!ok)
            {
                FinishRender(false, refusal: refusal);
                return;
            }
            renderWarning = warning;
            (renderCount, renderTimeOf) = RenderPlan.Frames(replayLog, options.From, options.To, options.Speed);
            if (renderCount == 0)
            {
                FinishRender(false, error: $"the range is outside the log ({replayLog.StartT:F2}-{replayLog.EndT:F2} s)");
                return;
            }
            if (options.Quality == null)  // High Fidelity: the frame rate does not matter offline (design 5.5)
            {
                var level = Array.IndexOf(QualitySettings.names, "High Fidelity");
                if (level >= 0) QualitySettings.SetQualityLevel(level, true);
            }
            QualitySettings.vSyncCount = 0;
            Application.targetFrameRate = -1;
            Time.captureFramerate = RenderPlan.FramesPerSecond;
            robotCamera.InputBlocked = true;
        }

        /// <summary>The render's frame: the log at the frame's time and the directed view at it.</summary>
        bool UpdateRender(double wall, out RobotSample sample)
        {
            sample = default;
            if (renderTimeOf == null) return false;
            var t = renderTimeOf(Math.Max(0, renderFrame - WarmUpFrames));
            replayClock.Set(t);
            replayFeed.FeedFor(t, f => Route(f, wall));
            gate.Release(t, f => state.Apply(f, wall));
            ApplyDirection(renderTrack.ViewAt(t));
            return state.TrySampleAt(t, out sample);
        }

        /// <summary>Sets the cameras and toggles to the journal's view (without journaling: the render writes none).</summary>
        void ApplyDirection(JournalView v)
        {
            if (v.Camera != null && Enum.TryParse(v.Camera, out ViewerCamera camera) && camera != view) SetCamera(camera);
            if (view == ViewerCamera.Orbit && v.Orbit is { } o) robotCamera.Orbit = ((float)o.yaw, (float)o.pitch, (float)o.distance);
            if (view == ViewerCamera.Free && v.FreePosition != null && v.FreeRotation != null)
                freeCamera.transform.SetPositionAndRotation(
                    new Vector3((float)v.FreePosition[0], (float)v.FreePosition[1], (float)v.FreePosition[2]),
                    new Quaternion((float)v.FreeRotation[0], (float)v.FreeRotation[1], (float)v.FreeRotation[2], (float)v.FreeRotation[3]));
            foreach (var kv in v.Toggles)
            {
                if (Enum.TryParse(kv.Key, out Overlay overlay))
                {
                    if (overlays[overlay] != kv.Value) overlays.Toggle(overlay);
                }
                else if (Enum.TryParse(kv.Key, out Panel panel) && overlays[panel] != kv.Value) overlays.Set(panel, kv.Value);
            }
        }

        /// <summary>Saves each rendered frame at the end of its frame, after the warm-up; quits after the last.</summary>
        IEnumerator CaptureFrames()
        {
            while (renderFrame < WarmUpFrames + renderCount)
            {
                yield return new WaitForEndOfFrame();
                if (renderFrame >= WarmUpFrames)
                {
                    var n = renderFrame - WarmUpFrames;
                    var shot = ScreenCapture.CaptureScreenshotAsTexture();
                    try
                    {
                        renderSize = (shot.width, shot.height);
                        var bytes = options.Png ? shot.EncodeToPNG() : shot.EncodeToJPG(95);
                        File.WriteAllBytes(Path.Combine(options.RenderOut, $"frame_{n:D6}.{(options.Png ? "png" : "jpg")}"), bytes);
                    }
                    finally
                    {
                        Destroy(shot);
                    }
                }
                ++renderFrame;
            }
            FinishRender(true);
        }

        void FinishRender(bool ok, string error = null, string refusal = null)
        {
            var summary = new JObject
            {
                ["ok"] = ok,
                ["log"] = options.Render, ["journal"] = options.RenderJournal,
                ["frames"] = ok ? renderCount : 0, ["fps"] = RenderPlan.FramesPerSecond, ["speed"] = options.Speed,
                ["from"] = ok ? renderTimeOf(0) : double.NaN, ["to"] = ok ? renderTimeOf(renderCount - 1) : double.NaN,
                ["size"] = new JArray(renderSize.w, renderSize.h),
                ["warning"] = renderWarning + (ok && (renderSize.w != DirectingWidth || renderSize.h != DirectingHeight)
                    ? $"{(renderWarning.Length > 0 ? "; " : "")}frames are {renderSize.w} x {renderSize.h}, not {DirectingWidth} x {DirectingHeight}" : ""),
                ["skipped"] = renderTrack == null ? new JObject() : JObject.FromObject(renderTrack.SkippedKinds),
            };
            if (error != null) summary["error"] = error;
            if (refusal != null) summary["refusal"] = refusal;
            try
            {
                Directory.CreateDirectory(options.RenderOut);
                File.WriteAllText(Path.Combine(options.RenderOut, "render-summary.json"), summary.ToString());
            }
            catch (Exception e)
            {
                Debug.LogError("render summary not written: " + e.Message);
            }
            Time.captureFramerate = 0;
            Application.Quit(ok ? 0 : 2);
        }

        string RenderStatus()
        {
            var hello = state.SessionHello;
            var session = hello == null ? "" : $"{(string)hello["scenario"]} seed {(int?)hello["seed"]}";
            return $"Replay | {session} | t {(double.IsNaN(displayedT) ? "-" : displayedT.ToString("F2"))} s";
        }
    }
}
