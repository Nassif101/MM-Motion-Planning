using System;
using System.IO;
using MotionPlanningSim.ROS;
using Unity.Pipeline.Commands;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace MotionPlanningSim.Editor
{
    // Renders the open scene from a temporary camera at a ROS map pose and writes a PNG, for
    // documentation screenshots. The camera is never saved with the scene.
    public static class SceneCaptureCommands
    {
        private const int WarmUpFrames = 12;

        [CliCommand("scene_capture",
            "Render the open scene from a camera at a ROS map pose (x, y, z metres; yaw, pitch degrees, pitch down " +
            "positive) to a PNG at output (relative to the repository root)",
            MainThreadRequired = true)]
        public static object Capture(string output, double x, double y, double z, double yaw = 0.0, double pitch = 0.0,
            float fov = 60.0f, int width = 1600, int height = 900)
        {
            if (string.IsNullOrWhiteSpace(output) || !output.EndsWith(".png", StringComparison.OrdinalIgnoreCase))
                throw new ArgumentException("output must be a .png path.", nameof(output));
            var repositoryRoot = Path.GetFullPath(Path.Combine(Directory.GetCurrentDirectory(), ".."));
            var path = Path.GetFullPath(Path.Combine(repositoryRoot, output));
            if (!path.StartsWith(repositoryRoot + Path.DirectorySeparatorChar, StringComparison.Ordinal))
                throw new ArgumentException("output must lie inside the repository.", nameof(output));

            var cameraObject = new GameObject("SceneCaptureCamera") { hideFlags = HideFlags.HideAndDontSave };
            var target = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB)
            {
                antiAliasing = 1
            };
            var image = new Texture2D(width, height, TextureFormat.RGB24, false);
            var previous = RenderTexture.active;
            try
            {
                cameraObject.transform.SetPositionAndRotation(
                    RosMapPose.ToUnityPosition(x, y, (float)z),
                    RosMapPose.ToUnityRotation(yaw * Math.PI / 180.0) * Quaternion.Euler((float)pitch, 0.0f, 0.0f));
                var camera = cameraObject.AddComponent<Camera>();
                camera.fieldOfView = fov;
                camera.nearClipPlane = 0.05f;
                camera.farClipPlane = 400.0f;
                var hdCamera = cameraObject.AddComponent<HDAdditionalCameraData>();
                hdCamera.antialiasing = HDAdditionalCameraData.AntialiasingMode.SubpixelMorphologicalAntiAliasing;
                camera.targetTexture = target;
                // HDRP builds history (exposure, shadows, volumetrics) over a few frames.
                for (var i = 0; i < WarmUpFrames; i++)
                    camera.Render();

                RenderTexture.active = target;
                image.ReadPixels(new Rect(0, 0, width, height), 0, 0);
                image.Apply();
                Directory.CreateDirectory(Path.GetDirectoryName(path));
                File.WriteAllBytes(path, image.EncodeToPNG());
                camera.targetTexture = null;
            }
            finally
            {
                RenderTexture.active = previous;
                UnityEngine.Object.DestroyImmediate(cameraObject);
                UnityEngine.Object.DestroyImmediate(image);
                target.Release();
                UnityEngine.Object.DestroyImmediate(target);
            }
            return new { output = path, bytes = new FileInfo(path).Length };
        }
    }
}
