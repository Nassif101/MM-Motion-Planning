using System.IO;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests.Viewer
{
    public sealed class MapFrameTests
    {
        static string MapYaml => Path.Combine(Application.dataPath, "..", "..", "ros2_ws", "src",
            "mobile_manipulator_navigation", "maps", "construction_site_v2.yaml");

        // The server's digests of the committed map files (plain SHA-256 of each file, routing_inputs.cpp)
        static JObject Routing() => new JObject
        {
            ["map_yaml_sha256"] = "9018b4107fe852fe4d7dbebdd2f166ce5ce031cb8ce9f307e5577ee080ff14c0",
            ["map_image_sha256"] = "73c0600c97e0eea41b6a160d966e870b2d5ff97c8ed5b38450e5709b6102fd37",
            ["map_metadata_sha256"] = "6b2e2b695962cf6e8eb51f11c413894f0c67f4d23b4bf0a67db5ef1438292937",
        };

        [Test]
        public void PixelWorldRoundTripAtCorners()
        {
            var m = MapFrame.Load(MapYaml);
            Assert.AreEqual((1000, 800), (m.Width, m.Height));
            Assert.AreEqual(0.05, m.Resolution, 1e-12);
            Assert.AreEqual((-25.0, -20.0), (m.OriginX, m.OriginY));
            Assert.AreEqual(m.Width * m.Height, m.Pixels.Length);
            foreach (var (px, py) in new[] { (0.0, 0.0), ((double)m.Width, (double)m.Height) })
            {
                var w = m.PixelToWorld(px, py);
                var p = m.WorldToPixel(w.x, w.y);
                Assert.AreEqual(px, p.px, 1e-9);
                Assert.AreEqual(py, p.py, 1e-9);
            }
            var topLeft = m.PixelToWorld(0, 0);  // row 0 is the map's y max
            Assert.AreEqual(-25.0, topLeft.x, 1e-9);
            Assert.AreEqual(20.0, topLeft.y, 1e-9);
            var cell = m.PixelToWorld(0.5, m.Height - 0.5);
            Assert.AreEqual(-24.975, cell.x, 1e-9);
            Assert.AreEqual(-19.975, cell.y, 1e-9);
        }

        [Test]
        public void FingerprintMatchesTheServersDigests()
        {
            Assert.IsTrue(MapFrame.Load(MapYaml).MatchesFingerprint(Routing(), out var reason), reason);
        }

        [Test]
        public void FingerprintMismatchDisablesGoals()
        {
            var m = MapFrame.Load(MapYaml);
            var routing = Routing();
            routing["map_image_sha256"] = new string('0', 64);
            Assert.IsFalse(m.MatchesFingerprint(routing, out var reason));
            StringAssert.Contains("construction_site_v2.pgm", reason);
            Assert.IsFalse(m.MatchesFingerprint(new JObject(), out reason), "a server without routing inputs");
            Assert.IsFalse(m.MatchesFingerprint(null, out reason));
        }
    }
}
