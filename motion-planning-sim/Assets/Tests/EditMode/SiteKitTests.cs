using System;
using System.IO;
using MotionPlanningSim.Environment;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests
{
    public class SiteKitTests
    {
        private static KitSpec LoadSpec()
        {
            var path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "tools", "site_kit", "kit_v2.json"));
            return KitSpec.Parse(File.ReadAllText(path));
        }

        [Test]
        public void ParsesPiecesAndOverhead()
        {
            var spec = LoadSpec();
            Assert.That(spec.Pieces.Length, Is.EqualTo(34));
            Assert.That(spec.Overhead.h3_bottom_m, Is.EqualTo(2.4f).Within(1e-6f));
            Assert.That(spec.Overhead.tops_m.Length, Is.EqualTo(3));
            var wall = spec.Get("wall_blockwork");
            Assert.That(wall.parametric_x, Is.True);
            Assert.That(wall.proxies.Length, Is.EqualTo(1));
            Assert.That(wall.material, Is.EqualTo("blockwork"));
            Assert.Throws<ArgumentException>(() => spec.Get("nope"));
        }

        [Test]
        public void ProxyBoxesAreTranslatedAndYawed()
        {
            var column = LoadSpec().Get("column_rc");
            var boxes = KitGeometry.ProxyBoxes(column, 2.0, 3.0, 0.0, Math.PI / 2.0);
            Assert.That(boxes.Length, Is.EqualTo(1));
            Assert.That(boxes[0].Center.x, Is.EqualTo(2.0f).Within(1e-5f));
            Assert.That(boxes[0].Center.y, Is.EqualTo(3.0f).Within(1e-5f));
            Assert.That(boxes[0].Center.z, Is.EqualTo(1.5f).Within(1e-5f));
            Assert.That(boxes[0].Yaw, Is.EqualTo((float)(Math.PI / 2.0)).Within(1e-6f));

            // An off-centre proxy rotates with the piece: local +x ends up along ROS +y.
            var ledger = LoadSpec().Get("scaffold_ledger_2p5");
            var ends = KitGeometry.ProxyBoxes(ledger, 2.0, 3.0, 0.0, Math.PI / 2.0);
            Assert.That(ends[2].Center.x, Is.EqualTo(2.0f).Within(1e-5f));
            Assert.That(ends[2].Center.y, Is.EqualTo(3.0f + 1.158f).Within(1e-5f));
        }

        [Test]
        public void ParametricWallScalesItsProxy()
        {
            var wall = LoadSpec().Get("wall_blockwork");
            var boxes = KitGeometry.ProxyBoxes(wall, 0.0, 0.0, 0.0, 0.0, 4.5f);
            Assert.That(boxes[0].Size.x, Is.EqualTo(4.5f).Within(1e-6f));
            Assert.That(boxes[0].Size.y, Is.EqualTo(0.2f).Within(1e-6f));
            Assert.Throws<ArgumentException>(() =>
                KitGeometry.ProxyBoxes(LoadSpec().Get("column_rc"), 0, 0, 0, 0, 2.0f));
        }

        [Test]
        public void MeshBoundsCheckAcceptsWithinTolerance()
        {
            var cabin = LoadSpec().Get("cabin_office");
            var size = cabin.Size + new Vector3(0.015f, -0.015f, 0.015f);
            Assert.That(KitGeometry.BoundsMismatch(cabin, size), Is.Null);
        }

        [Test]
        public void MeshBoundsCheckRejectsSwappedAxes()
        {
            var cabin = LoadSpec().Get("cabin_office");
            var s = cabin.Size;
            var message = KitGeometry.BoundsMismatch(cabin, new Vector3(s.x, s.z, s.y));
            Assert.That(message, Does.Contain("cabin_office"));
        }

        [Test]
        public void ImportedSizeSwapsUnityYAndZ()
        {
            Assert.That(KitGeometry.ImportedRosSize(new Vector3(6f, 2.5f, 2.4f)), Is.EqualTo(new Vector3(6f, 2.4f, 2.5f)));
        }
    }
}
