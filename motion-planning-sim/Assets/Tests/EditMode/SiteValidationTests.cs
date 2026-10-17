using System.Collections.Generic;
using System.IO;
using System.Linq;
using MotionPlanningSim.Environment;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests
{
    public class SiteValidationTests
    {
        private static readonly Nav2GridSpec Grid = new Nav2GridSpec(-10.0f, -10.0f, 10.0f, 10.0f, 0.05f);

        // A ROS map box from its bottom height and full size.
        private static NamedBox Box(string name, float x, float y, float bottom, float sx, float sy, float sz,
            float yaw = 0.0f)
        {
            return new NamedBox
            {
                Name = name,
                Box = new RosBox(new Vector3(x, y, bottom + sz * 0.5f), new Vector3(sx, sy, sz), yaw)
            };
        }

        private static List<string> Problems(SiteFeatures features, params NamedBox[] boxes)
        {
            return SiteValidation.Problems(boxes, features ?? new SiteFeatures(), Grid);
        }

        // A wall along x at y = 0 (0.2 m thick, 3 m high) with a door frame whose jambs leave
        // clearWidth between their inner faces, centred at x = 0.
        private static NamedBox[] WallWithDoor(float clearWidth)
        {
            var half = clearWidth * 0.5f;
            return new[]
            {
                Box("Wall/West", -3.0f - half - 0.06f, 0.0f, 0.0f, 6.0f, 0.2f, 3.0f),
                Box("Door/jamb_w", -half - 0.03f, 0.0f, 0.0f, 0.06f, 0.22f, 2.1f),
                Box("Door/jamb_e", half + 0.03f, 0.0f, 0.0f, 0.06f, 0.22f, 2.1f),
                Box("Door/head", 0.0f, 0.0f, 2.1f, clearWidth + 0.12f, 0.22f, 0.9f),
                Box("Wall/East", 3.0f + half + 0.06f, 0.0f, 0.0f, 6.0f, 0.2f, 3.0f),
            };
        }

        private static SiteFeatures DoorFeatures(float clearWidth)
        {
            var features = new SiteFeatures();
            features.Doors.Add(new DoorFeature { Name = "door", CentreX = 0.0f, CentreY = 0.0f, WallYaw = 0.0f, ClearWidth = clearWidth });
            return features;
        }

        [Test]
        public void FloatingBoxIsReported()
        {
            var problems = Problems(null, Box("Crate", 0.0f, 0.0f, 0.5f, 1.0f, 1.0f, 1.0f));
            Assert.That(problems, Has.Count.EqualTo(1));
            Assert.That(problems[0], Does.Contain("Crate").And.Contain("unsupported"));
        }

        [Test]
        public void BoxRestingOnAnotherIsSupported()
        {
            var problems = Problems(null,
                Box("Pallet", 0.0f, 0.0f, 0.0f, 1.2f, 0.8f, 0.144f),
                Box("Stack", 0.1f, 0.0f, 0.15f, 1.0f, 0.8f, 0.6f));
            Assert.That(problems, Is.Empty);
        }

        // Side-mounted members (scaffold ledgers, transoms) are supported when they touch the side
        // of a supported box within 0.01 m; a chain must reach the ground.
        [Test]
        public void SideMountedMemberIsSupported()
        {
            var problems = Problems(null,
                Box("Standard_a", 0.0f, 0.0f, 0.0f, 0.124f, 0.124f, 2.0f),
                Box("Standard_b", 2.5f, 0.0f, 0.0f, 0.124f, 0.124f, 2.0f),
                Box("Ledger", 1.25f, 0.0f, 1.42f, 2.376f, 0.05f, 0.08f));
            Assert.That(problems, Is.Empty);
        }

        [Test]
        public void BoxTouchingOnlyAFloatingBoxIsReported()
        {
            var problems = Problems(null,
                Box("Floating_a", 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f),
                Box("Floating_b", 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f));
            Assert.That(problems, Has.Count.EqualTo(2));
            Assert.That(problems.All(p => p.Contains("unsupported")), Is.True);
        }

        [Test]
        public void IntersectingBoxesAreReportedUnlessJoined()
        {
            var a = Box("Fence/panel_1", 0.0f, 0.0f, 0.0f, 2.0f, 0.2f, 0.2f);
            var b = Box("Fence/panel_2", 1.9f, 0.0f, 0.0f, 2.0f, 0.2f, 0.2f);
            var problems = Problems(null, a, b);
            Assert.That(problems, Has.Count.EqualTo(1));
            Assert.That(problems[0], Does.Contain("intersect").And.Contain("Fence/panel_1").And.Contain("Fence/panel_2"));

            var joined = new SiteFeatures();
            joined.Joints.Add(new JointPair { A = "Fence/panel_2", B = "Fence/panel_1" });
            Assert.That(Problems(joined, a, b), Is.Empty);
        }

        [Test]
        public void TouchingBoxesAreFine()
        {
            var problems = Problems(null,
                Box("A", 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f),
                Box("B", 0.996f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f),
                Box("C", 0.0f, 0.0f, 0.996f, 1.0f, 1.0f, 0.5f));
            Assert.That(problems, Is.Empty);
        }

        [Test]
        public void YawedBoxesIntersectByTheirOrientedFootprint()
        {
            // Two 45-degree boxes whose axis-aligned bounds overlap but whose footprints do not.
            var yaw = Mathf.PI / 4.0f;
            var clear = Problems(null,
                Box("A", 0.0f, 0.0f, 0.0f, 2.0f, 0.2f, 1.0f, yaw),
                Box("B", 0.6f, -0.6f, 0.0f, 2.0f, 0.2f, 1.0f, yaw));
            Assert.That(clear, Is.Empty);
            var crossing = Problems(null,
                Box("A", 0.0f, 0.0f, 0.0f, 2.0f, 0.2f, 1.0f, yaw),
                Box("B", 0.0f, 0.0f, 0.0f, 2.0f, 0.2f, 1.0f, -yaw));
            Assert.That(crossing, Has.Count.EqualTo(1));
        }

        [Test]
        public void DoorAtDeclaredWidthPasses()
        {
            Assert.That(Problems(DoorFeatures(1.05f), WallWithDoor(1.05f)), Is.Empty);
        }

        [Test]
        public void DoorNarrowerThanDeclaredIsReported()
        {
            var problems = Problems(DoorFeatures(1.05f), WallWithDoor(1.03f));
            Assert.That(problems, Has.Count.EqualTo(1));
            Assert.That(problems[0], Does.Contain("door").And.Contain("1.030"));
        }

        [Test]
        public void BoxInApproachIsReported()
        {
            var boxes = WallWithDoor(1.05f).ToList();
            // Outside is to the right of the wall direction: -y for a wall along +x.
            boxes.Add(Box("Pallet_out", 0.0f, -2.0f, 0.0f, 1.2f, 0.8f, 0.144f));
            boxes.Add(Box("Column_in", 0.3f, 4.5f, 0.0f, 0.4f, 0.4f, 3.0f));
            var problems = Problems(DoorFeatures(1.05f), boxes.ToArray());
            Assert.That(problems, Has.Count.EqualTo(2));
            Assert.That(problems.Any(p => p.Contains("Pallet_out") && p.Contains("outside")), Is.True);
            Assert.That(problems.Any(p => p.Contains("Column_in") && p.Contains("inside")), Is.True);
        }

        [Test]
        public void BoxInClearRectIsReportedButOverheadMembersAreNot()
        {
            var features = new SiteFeatures();
            features.ClearRects.Add(new ClearRect { Name = "lane", CentreX = 0.0f, CentreY = 0.0f, SizeX = 6.0f, SizeY = 3.0f });
            features.Overheads.Add(new OverheadFeature { Name = "H1", BottomZ = 1.42f });
            var problems = Problems(features,
                Box("Standard_a", 0.0f, -1.562f, 0.0f, 0.124f, 0.124f, 2.0f),
                Box("Standard_b", 0.0f, 1.562f, 0.0f, 0.124f, 0.124f, 2.0f),
                Box("H1", 0.0f, 0.0f, 1.42f, 0.05f, 3.0f, 0.08f),
                Box("High", 2.0f, 0.0f, 2.5f, 0.5f, 0.5f, 0.5f),
                Box("Stack", -2.0f, 0.0f, 0.0f, 1.2f, 0.8f, 1.0f));
            // "High" floats (reported as unsupported) but sits at 2.5 m, above the clear-rect band.
            var rectProblems = problems.Where(p => !p.Contains("unsupported")).ToList();
            Assert.That(rectProblems, Has.Count.EqualTo(1));
            Assert.That(rectProblems[0], Does.Contain("Stack").And.Contain("lane"));
        }

        [Test]
        public void OverheadAtWrongHeightIsReported()
        {
            var features = new SiteFeatures();
            features.Overheads.Add(new OverheadFeature { Name = "Tunnel/H1", BottomZ = 1.42f });
            NamedBox[] Bay(float bottom) => new[]
            {
                Box("Tunnel/Standard_a", 0.0f, 0.0f, 0.0f, 0.124f, 0.124f, 2.0f),
                Box("Tunnel/Standard_b", 2.5f, 0.0f, 0.0f, 0.124f, 0.124f, 2.0f),
                Box("Tunnel/H1/proxy_0", 1.25f, 0.0f, bottom, 2.376f, 0.05f, 0.08f),
            };
            Assert.That(Problems(features, Bay(1.423f)), Is.Empty);
            var problems = Problems(features, Bay(1.45f));
            Assert.That(problems, Has.Count.EqualTo(1));
            Assert.That(problems[0], Does.Contain("Tunnel/H1").And.Contain("1.450"));

            var missing = new SiteFeatures();
            missing.Overheads.Add(new OverheadFeature { Name = "Tunnel/H3", BottomZ = 2.4f });
            Assert.That(Problems(missing, Bay(1.42f)).Single(), Does.Contain("Tunnel/H3").And.Contain("missing"));
        }

        [Test]
        public void BoxOutsideGridIsReported()
        {
            var problems = Problems(null,
                Box("Inside", 9.5f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f),
                Box("Outside", 9.6f, 3.0f, 0.0f, 1.0f, 1.0f, 1.0f));
            Assert.That(problems, Has.Count.EqualTo(1));
            Assert.That(problems[0], Does.Contain("Outside").And.Contain("grid"));
        }

        // The V2 layout itself (kit proxies from the spec plus primitive boxes) passes the
        // validator, independent of the built scene.
        [Test]
        public void V2LayoutPassesTheValidator()
        {
            var path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "tools", "site_kit", "kit_v2.json"));
            var spec = KitSpec.Parse(File.ReadAllText(path));
            var boxes = SiteV2Layout.ColliderBoxes(spec);
            Assert.That(boxes.Count, Is.GreaterThan(50));
            var problems = SiteValidation.Problems(boxes, SiteV2Layout.Features(spec),
                SiteRegistry.Get(SiteV2Layout.SiteName).Grid);
            Assert.That(problems, Is.Empty, string.Join("\n", problems));
        }
    }
}
