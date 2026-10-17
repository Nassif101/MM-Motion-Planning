using System;
using System.Collections.Generic;

namespace MotionPlanningSim.Environment
{
    // A collider box of the site in the ROS map frame, named by its path under
    // Environment/NavigationObstacles (for example "Building/WallSouth_1/proxy_0").
    public sealed class NamedBox
    {
        public string Name;
        public RosBox Box;
    }

    public sealed class SiteFeatures
    {
        public readonly List<DoorFeature> Doors = new List<DoorFeature>();
        public readonly List<ClearRect> ClearRects = new List<ClearRect>();
        public readonly List<OverheadFeature> Overheads = new List<OverheadFeature>();
        public readonly List<JointPair> Joints = new List<JointPair>();
    }

    // A door in a wall along WallYaw (radians); outside is to the right of the wall direction.
    // ClearWidth is the width between the reveals (frames included); the straight approaches run
    // ApproachOut outside and ApproachIn inside, ClearWidth wide.
    public sealed class DoorFeature
    {
        public string Name;
        public float CentreX;
        public float CentreY;
        public float WallYaw;
        public float ClearWidth;
        public float ApproachOut = 3.0f;
        public float ApproachIn = 5.0f;
        // The frame's depth across the wall; the approaches start at its faces.
        public float FrameDepth = 0.22f;
    }

    // A rectangle (lane, open area) that must stay free of boxes below ClearBandTop. SizeX lies
    // along Yaw.
    public sealed class ClearRect
    {
        public string Name;
        public float CentreX;
        public float CentreY;
        public float SizeX;
        public float SizeY;
        public float Yaw;
    }

    // An overhead member: the boxes named Name (or under it) must have their bottom at BottomZ.
    // Overhead members are exempt from the clear-rectangle and approach checks.
    // Boxes of the member whose bottom is below MinBottomZ (a bridge's posts) are not part of it.
    public sealed class OverheadFeature
    {
        public string Name;
        public float BottomZ;
        public float MinBottomZ;
    }

    // Two pieces (names or name prefixes) that are allowed to intersect and support each other.
    public sealed class JointPair
    {
        public string A;
        public string B;
    }

    // Static checks of a site's collider boxes (ROS map frame): every box inside the grid, no two
    // boxes intersecting unless declared as a joint, every box supported, and the declared
    // features (door clear widths and approaches, clear rectangles, overhead heights).
    public static class SiteValidation
    {
        public const double GridTolerance = 1e-3;
        // Boxes may overlap by this much and still count as touching.
        public const double IntersectionTolerance = 0.005;
        // A box whose bottom is at most this high stands on the ground.
        public const double GroundedBottom = 0.01;
        // Resting (top to bottom) and side contact gap that still counts as support.
        public const double SupportTolerance = 0.01;
        public const double DoorWidthTolerance = 0.005;
        public const double OverheadTolerance = 0.005;
        // Approaches and clear rectangles must be free of boxes whose bottom is below this.
        public const double ClearBandTop = 2.5;
        // Door reveals: boxes on the wall line reaching below this height (the frame head is above).
        public const double RevealBandTop = 2.0;
        private const double Epsilon = 1e-4;

        public static List<string> Problems(IReadOnlyList<NamedBox> boxes, SiteFeatures features, Nav2GridSpec grid)
        {
            if (boxes == null)
                throw new ArgumentNullException(nameof(boxes));
            features = features ?? new SiteFeatures();
            var problems = new List<string>();
            var shapes = new Shape[boxes.Count];
            for (var i = 0; i < boxes.Count; i++)
                shapes[i] = Shape.Of(boxes[i].Box);

            for (var i = 0; i < shapes.Length; i++)
            {
                var s = shapes[i];
                if (s.MinX < grid.MinRosX - GridTolerance || s.MaxX > grid.MaxRosX + GridTolerance ||
                    s.MinY < grid.MinRosY - GridTolerance || s.MaxY > grid.MaxRosY + GridTolerance)
                    problems.Add($"outside the grid: {boxes[i].Name}");
            }

            var neighbours = Neighbours(shapes);
            for (var i = 0; i < shapes.Length; i++)
            {
                foreach (var j in neighbours[i])
                {
                    if (j <= i)
                        continue;
                    var depth = Math.Min(Overlap2D(shapes[i], shapes[j]), ZOverlap(shapes[i], shapes[j]));
                    if (depth > IntersectionTolerance && !Joined(features, boxes[i].Name, boxes[j].Name))
                        problems.Add(Invariant($"intersect: {boxes[i].Name} x {boxes[j].Name} by {depth:F3} m"));
                }
            }

            var supported = Supported(boxes, shapes, neighbours, features);
            for (var i = 0; i < shapes.Length; i++)
            {
                if (!supported[i])
                    problems.Add(Invariant($"unsupported: {boxes[i].Name} (bottom {shapes[i].MinZ:F3} m)"));
            }

            var exempt = new bool[boxes.Count];
            for (var i = 0; i < boxes.Count; i++)
            {
                var shape = shapes[i];
                exempt[i] = features.Overheads.Exists(o => Matches(boxes[i].Name, o.Name) && shape.MinZ >= o.MinBottomZ);
            }

            foreach (var door in features.Doors)
                CheckDoor(door, boxes, shapes, exempt, problems);

            foreach (var rect in features.ClearRects)
            {
                var area = Shape.Rect(rect.CentreX, rect.CentreY, rect.SizeX, rect.SizeY, rect.Yaw);
                CheckClear(area, $"clear rect {rect.Name}", boxes, shapes, exempt, problems);
            }

            foreach (var overhead in features.Overheads)
            {
                var bottom = double.PositiveInfinity;
                for (var i = 0; i < boxes.Count; i++)
                {
                    if (Matches(boxes[i].Name, overhead.Name) && shapes[i].MinZ >= overhead.MinBottomZ)
                        bottom = Math.Min(bottom, shapes[i].MinZ);
                }
                if (double.IsPositiveInfinity(bottom))
                    problems.Add($"overhead {overhead.Name}: missing");
                else if (Math.Abs(bottom - overhead.BottomZ) > OverheadTolerance)
                    problems.Add(Invariant($"overhead {overhead.Name}: bottom {bottom:F3} m, expected {overhead.BottomZ:F3} m"));
            }
            return problems;
        }

        // True when name is the key itself or lies under it ("Zone/Piece" matches "Zone/Piece/proxy_0").
        public static bool Matches(string name, string key)
        {
            return name == key || (name.Length > key.Length && name.StartsWith(key, StringComparison.Ordinal) &&
                                   name[key.Length] == '/');
        }

        private static bool Joined(SiteFeatures features, string a, string b)
        {
            foreach (var joint in features.Joints)
            {
                if ((Matches(a, joint.A) && Matches(b, joint.B)) || (Matches(a, joint.B) && Matches(b, joint.A)))
                    return true;
            }
            return false;
        }

        // Supported boxes, grown from the ground: a box is supported when it stands on the ground,
        // rests on a supported box (bottom within SupportTolerance of its top, footprints
        // overlapping), touches the side of a supported box (gap within SupportTolerance, heights
        // overlapping) or is joined to one.
        private static bool[] Supported(IReadOnlyList<NamedBox> boxes, Shape[] shapes, List<int>[] neighbours,
            SiteFeatures features)
        {
            var supported = new bool[shapes.Length];
            for (var i = 0; i < shapes.Length; i++)
                supported[i] = shapes[i].MinZ <= GroundedBottom;

            var changed = true;
            while (changed)
            {
                changed = false;
                for (var i = 0; i < shapes.Length; i++)
                {
                    if (supported[i])
                        continue;
                    foreach (var j in neighbours[i])
                    {
                        if (!supported[j])
                            continue;
                        var footprint = Overlap2D(shapes[i], shapes[j]);
                        var rests = Math.Abs(shapes[i].MinZ - shapes[j].MaxZ) <= SupportTolerance && footprint > Epsilon;
                        var side = footprint >= -SupportTolerance && ZOverlap(shapes[i], shapes[j]) > Epsilon;
                        if (rests || side || Joined(features, boxes[i].Name, boxes[j].Name))
                        {
                            supported[i] = true;
                            changed = true;
                            break;
                        }
                    }
                }
            }
            return supported;
        }

        private static void CheckDoor(DoorFeature door, IReadOnlyList<NamedBox> boxes, Shape[] shapes, bool[] exempt,
            List<string> problems)
        {
            var cos = Math.Cos(door.WallYaw);
            var sin = Math.Sin(door.WallYaw);
            // Outside is to the right of the wall direction.
            var outX = sin;
            var outY = -cos;
            var left = double.NegativeInfinity;
            var right = double.PositiveInfinity;
            for (var i = 0; i < shapes.Length; i++)
            {
                var s = shapes[i];
                if (s.MinZ >= RevealBandTop)
                    continue;
                s.Project(door.CentreX, door.CentreY, cos, sin, out var uMin, out var uMax);
                s.Project(door.CentreX, door.CentreY, outX, outY, out var vMin, out var vMax);
                if (!(vMin < 0.0 && vMax > 0.0))
                    continue;
                if (uMin < 0.0 && uMax > 0.0)
                    problems.Add($"door {door.Name}: {boxes[i].Name} blocks the opening");
                else if (uMax <= 0.0)
                    left = Math.Max(left, uMax);
                else
                    right = Math.Min(right, uMin);
            }

            if (double.IsInfinity(left) || double.IsInfinity(right))
            {
                problems.Add($"door {door.Name}: no reveal on one side");
            }
            else if (Math.Abs(right - left - door.ClearWidth) > DoorWidthTolerance)
            {
                problems.Add(Invariant($"door {door.Name}: clear width {right - left:F3} m, declared {door.ClearWidth:F3} m"));
            }

            var start = door.FrameDepth * 0.5;
            var outside = Shape.Rect(door.CentreX + outX * (start + door.ApproachOut * 0.5),
                door.CentreY + outY * (start + door.ApproachOut * 0.5), door.ClearWidth, door.ApproachOut, door.WallYaw);
            var inside = Shape.Rect(door.CentreX - outX * (start + door.ApproachIn * 0.5),
                door.CentreY - outY * (start + door.ApproachIn * 0.5), door.ClearWidth, door.ApproachIn, door.WallYaw);
            CheckClear(outside, $"door {door.Name} approach outside", boxes, shapes, exempt, problems);
            CheckClear(inside, $"door {door.Name} approach inside", boxes, shapes, exempt, problems);
        }

        private static void CheckClear(Shape area, string label, IReadOnlyList<NamedBox> boxes, Shape[] shapes,
            bool[] exempt, List<string> problems)
        {
            for (var i = 0; i < shapes.Length; i++)
            {
                if (exempt[i] || shapes[i].MinZ >= ClearBandTop || !area.BoundsOverlap(shapes[i], 0.0))
                    continue;
                var depth = Overlap2D(area, shapes[i]);
                if (depth > IntersectionTolerance)
                    problems.Add(Invariant($"{label}: {boxes[i].Name} intrudes by {depth:F3} m"));
            }
        }

        private static List<int>[] Neighbours(Shape[] shapes)
        {
            var result = new List<int>[shapes.Length];
            for (var i = 0; i < shapes.Length; i++)
                result[i] = new List<int>();
            for (var i = 0; i < shapes.Length; i++)
            {
                for (var j = i + 1; j < shapes.Length; j++)
                {
                    if (shapes[i].BoundsOverlap(shapes[j], 2.0 * SupportTolerance) &&
                        shapes[i].MinZ <= shapes[j].MaxZ + 2.0 * SupportTolerance &&
                        shapes[j].MinZ <= shapes[i].MaxZ + 2.0 * SupportTolerance)
                    {
                        result[i].Add(j);
                        result[j].Add(i);
                    }
                }
            }
            return result;
        }

        // Smallest projected overlap of two footprints over their four axes (separating axis
        // theorem): positive is the penetration depth, negative a gap.
        private static double Overlap2D(Shape a, Shape b)
        {
            var overlap = double.PositiveInfinity;
            foreach (var shape in new[] { a, b })
            {
                overlap = Math.Min(overlap, AxisOverlap(a, b, shape.Cos, shape.Sin));
                overlap = Math.Min(overlap, AxisOverlap(a, b, -shape.Sin, shape.Cos));
            }
            return overlap;
        }

        private static double AxisOverlap(Shape a, Shape b, double ax, double ay)
        {
            a.Project(0.0, 0.0, ax, ay, out var aMin, out var aMax);
            b.Project(0.0, 0.0, ax, ay, out var bMin, out var bMax);
            return Math.Min(aMax, bMax) - Math.Max(aMin, bMin);
        }

        private static double ZOverlap(Shape a, Shape b) => Math.Min(a.MaxZ, b.MaxZ) - Math.Max(a.MinZ, b.MinZ);

        private static string Invariant(FormattableString text) => FormattableString.Invariant(text);

        // An oriented box: footprint centre, half extents along its own axes, yaw, height range,
        // and its axis-aligned footprint bounds.
        private readonly struct Shape
        {
            private Shape(double cx, double cy, double hx, double hy, double yaw, double minZ, double maxZ)
            {
                Cx = cx;
                Cy = cy;
                Hx = hx;
                Hy = hy;
                Cos = Math.Cos(yaw);
                Sin = Math.Sin(yaw);
                MinZ = minZ;
                MaxZ = maxZ;
                var ex = Math.Abs(Cos) * hx + Math.Abs(Sin) * hy;
                var ey = Math.Abs(Sin) * hx + Math.Abs(Cos) * hy;
                MinX = cx - ex;
                MaxX = cx + ex;
                MinY = cy - ey;
                MaxY = cy + ey;
            }

            public readonly double Cx, Cy, Hx, Hy, Cos, Sin, MinZ, MaxZ, MinX, MaxX, MinY, MaxY;

            public static Shape Of(RosBox box)
            {
                return new Shape(box.Center.x, box.Center.y, box.Size.x * 0.5, box.Size.y * 0.5, box.Yaw,
                    box.Center.z - box.Size.z * 0.5, box.Center.z + box.Size.z * 0.5);
            }

            public static Shape Rect(double cx, double cy, double sizeX, double sizeY, double yaw)
            {
                return new Shape(cx, cy, sizeX * 0.5, sizeY * 0.5, yaw, double.NegativeInfinity, double.PositiveInfinity);
            }

            // Interval of the footprint projected on the unit axis (ax, ay), relative to (ox, oy).
            public void Project(double ox, double oy, double ax, double ay, out double min, out double max)
            {
                var centre = (Cx - ox) * ax + (Cy - oy) * ay;
                var extent = Hx * Math.Abs(Cos * ax + Sin * ay) + Hy * Math.Abs(-Sin * ax + Cos * ay);
                min = centre - extent;
                max = centre + extent;
            }

            public bool BoundsOverlap(Shape other, double margin)
            {
                return MinX <= other.MaxX + margin && other.MinX <= MaxX + margin &&
                       MinY <= other.MaxY + margin && other.MinY <= MaxY + margin;
            }
        }
    }
}
