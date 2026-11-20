using System.IO;
using UnityEngine;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>The committed codec fixtures shared with the C++ and Python tests.</summary>
    internal static class ViewerFixtures
    {
        public static byte[] Read(string name) =>
            File.ReadAllBytes(Path.Combine(Application.dataPath, "..", "..", "tools", "plant", "fixtures", name));
    }
}
