using System;
using System.Collections.Generic;
using UnityEngine;

namespace MotionPlanningSim.Environment
{
    // One kit piece placed in the ROS map frame (bottom centre, yaw about +z). LengthX > 0
    // stretches a parametric_x piece.
    public sealed class KitPlacement
    {
        public string Zone;
        public string Name;
        public string PieceId;
        public double X;
        public double Y;
        public double Z;
        public double Yaw;
        public float LengthX;
    }

    // A primitive box with a kit material: a collider proxy plus visual when Collider is true
    // (posts), a flush ground decal without a collider otherwise (asphalt, markings).
    public sealed class PrimitiveBox
    {
        public string Zone;
        public string Name;
        public string Material;
        public RosBox Box;
        public bool Collider;
    }

    public sealed class ZoneLayout
    {
        public string Zone;
        public readonly List<KitPlacement> Pieces = new List<KitPlacement>();
        public readonly List<PrimitiveBox> Boxes = new List<PrimitiveBox>();
        public readonly SiteFeatures Features = new SiteFeatures();
    }

    // The ConstructionSiteV2 layout (spec Section 4), ROS map frame, metres. The builder places
    // exactly these pieces; the validator checks them against the declared features.
    public static class SiteV2Layout
    {
        public const string SiteName = "construction_site_v2";

        // Default robot spawn: on the spine near the gate.
        public const double SpawnX = -21.0;
        public const double SpawnY = 0.0;
        public const double SpawnYaw = 0.0;

        // Perimeter: hoarding on the street (west) side, mesh fence elsewhere, vehicle gate.
        public const float SiteMinX = -25.0f;
        public const float SiteMaxX = 25.0f;
        public const float SiteMinY = -20.0f;
        public const float SiteMaxY = 20.0f;
        public const float GateMinY = -2.5f;
        public const float GateMaxY = 2.5f;
        public const float CornerPostSize = 0.3f;
        public const float CornerPostHeight = 2.4f;
        public const float GatePierSize = 0.4f; // column_rc
        // Perimeter lines (piece centrelines) sit on the corner post centres.
        public const float WestLineX = SiteMinX + CornerPostSize * 0.5f;
        public const float EastLineX = SiteMaxX - CornerPostSize * 0.5f;
        public const float NorthLineY = SiteMaxY - CornerPostSize * 0.5f;
        public const float SouthLineY = SiteMinY + CornerPostSize * 0.5f;
        public const float FenceStagger = 0.03f;

        // Spine road: asphalt haul road with centre and edge markings.
        public const float SpineMinX = -25.0f;
        public const float SpineMaxX = 21.0f;
        public const float SpineMinY = -2.5f;
        public const float SpineMaxY = 2.5f;
        public const float MarkingWidth = 0.1f;
        public const float EdgeLineInset = 0.15f;
        public const float MarkingStartX = -24.0f;
        public const float MarkingEndX = 20.8f;
        public const float CentreDash = 3.0f;
        public const float CentreGap = 3.0f;
        // Edge-line breaks where lanes and yards join the spine.
        private static readonly (float from, float to)[] NorthEdgeBreaks = { (-8.0f, -5.0f) }; // scaffold tunnel
        private static readonly (float from, float to)[] SouthEdgeBreaks = { (-7.0f, 7.0f), (15.0f, 17.6f) }; // delivery yard, services
        // Flush decals: the asphalt top is the floor (z 0); markings stand 2 mm proud.
        public const float AsphaltThickness = 0.003f;
        public const float MarkingTop = 0.002f;

        // Building shell.
        public const float BuildingMinX = -3.0f;
        public const float BuildingMaxX = 19.0f;
        public const float BuildingMinY = 5.5f;
        public const float BuildingMaxY = 17.0f;
        public const float WallThickness = 0.2f;
        public const float ColumnSize = 0.4f;
        public const float StoreyHeight = 3.0f;
        public const float BeamWidth = 0.3f;
        public static readonly float[] ColumnGridX = { 2.5f, 8.0f, 13.5f };
        public const float ColumnRowY = 11.25f;
        public const float CoreMinX = 15.0f;
        public const float CoreMaxX = 19.0f;
        public const float CoreMinY = 13.5f;
        public const float CoreMaxY = 17.0f;
        public const float CoreWallThickness = 0.25f;
        public const float Door130CentreX = 5.25f;
        public const float Door105CentreX = 16.25f;
        public const float Door130FrameWidth = 1.42f;
        public const float Door105FrameWidth = 1.17f;
        public const float ForecourtMinY = 2.5f;
        public const float ForecourtMaxY = 5.5f;
        // The slab finish is visual only; its top sits just above the ground decal.
        public const float SlabTop = 0.0005f;

        private const float HalfPi = Mathf.PI * 0.5f;

        public static IReadOnlyList<ZoneLayout> Zones()
        {
            return new[] { Street(), Perimeter(), Spine(), Building() };
        }

        // Every collider box of the layout, named as the builder names its proxies
        // (<Zone>/<Name>/proxy_<i>).
        public static List<NamedBox> ColliderBoxes(KitSpec spec)
        {
            var boxes = new List<NamedBox>();
            foreach (var zone in Zones())
            {
                foreach (var p in zone.Pieces)
                {
                    var proxies = KitGeometry.ProxyBoxes(spec.Get(p.PieceId), p.X, p.Y, p.Z, p.Yaw, p.LengthX);
                    for (var i = 0; i < proxies.Length; i++)
                        boxes.Add(new NamedBox { Name = $"{zone.Zone}/{p.Name}/proxy_{i}", Box = proxies[i] });
                }
                foreach (var b in zone.Boxes)
                {
                    if (b.Collider)
                        boxes.Add(new NamedBox { Name = $"{zone.Zone}/{b.Name}/proxy_0", Box = b.Box });
                }
            }
            return boxes;
        }

        public static SiteFeatures Features(KitSpec spec)
        {
            var features = new SiteFeatures();
            foreach (var zone in Zones())
            {
                features.Doors.AddRange(zone.Features.Doors);
                features.ClearRects.AddRange(zone.Features.ClearRects);
                features.Overheads.AddRange(zone.Features.Overheads);
                features.Joints.AddRange(zone.Features.Joints);
            }
            return features;
        }

        public static ZoneLayout Perimeter()
        {
            var zone = new ZoneLayout { Zone = "Perimeter" };
            var postTopY = NorthLineY - CornerPostSize * 0.5f;
            var postBottomY = SouthLineY + CornerPostSize * 0.5f;
            var postWestX = WestLineX + CornerPostSize * 0.5f;
            var postEastX = EastLineX - CornerPostSize * 0.5f;

            Post(zone, "CornerPostNW", WestLineX, NorthLineY);
            Post(zone, "CornerPostNE", EastLineX, NorthLineY);
            Post(zone, "CornerPostSE", EastLineX, SouthLineY);
            Post(zone, "CornerPostSW", WestLineX, SouthLineY);

            // Vehicle gate: two concrete piers flanking the 5 m opening, the two Heras gate panels
            // parked open inside along the hoarding.
            var pierX = SiteMinX + GatePierSize * 0.5f;
            Piece(zone, "GatePierNorth", "column_rc", pierX, GateMaxY + GatePierSize * 0.5f);
            Piece(zone, "GatePierSouth", "column_rc", pierX, GateMinY - GatePierSize * 0.5f);
            var hoardingInnerX = WestLineX + 0.085f;
            var parkedX = hoardingInnerX + 0.005f + 0.105f;
            Piece(zone, "GateLeafNorth", "fence_panel_3p5", parkedX, GateMaxY + GatePierSize + 0.1f + 1.75f, 0.0, HalfPi);
            Piece(zone, "GateLeafSouth", "fence_panel_3p5", parkedX, GateMinY - GatePierSize - 0.1f - 1.75f, 0.0, HalfPi);

            // Street side: plywood hoarding between the gate piers and the corner posts. Both runs
            // head south so the panels' framing faces the site and the plain face the street.
            Run(zone, "HoardingNorth", "hoarding_panel_2p4", 2.4f, new Vector2(WestLineX, postTopY),
                new Vector2(WestLineX, GateMaxY + GatePierSize));
            Run(zone, "HoardingSouth", "hoarding_panel_2p4", 2.4f, new Vector2(WestLineX, GateMinY - GatePierSize),
                new Vector2(WestLineX, postBottomY));

            // Mesh fence on the other three sides.
            Run(zone, "FenceNorth", "fence_panel_3p5", 3.5f, new Vector2(postWestX, NorthLineY),
                new Vector2(postEastX, NorthLineY));
            Run(zone, "FenceEast", "fence_panel_3p5", 3.5f, new Vector2(EastLineX, postTopY),
                new Vector2(EastLineX, postBottomY));
            Run(zone, "FenceSouth", "fence_panel_3p5", 3.5f, new Vector2(postEastX, SouthLineY),
                new Vector2(postWestX, SouthLineY));
            return zone;
        }

        // The public road outside the gate (visual only, beyond the grid): carriageway, footway,
        // and the vehicle crossover into the spine.
        public static ZoneLayout Street()
        {
            var zone = new ZoneLayout { Zone = "Street" };
            const float footwayInner = SiteMinX - 2.0f, kerb = footwayInner, far = 100.0f;
            Decal(zone, "Carriageway", "asphalt", kerb - 7.0f, kerb, -far, far, -AsphaltThickness, 0.0f);
            Decal(zone, "FootwayNorth", "concrete", footwayInner, SiteMinX, GateMaxY, far, -AsphaltThickness, 0.0f);
            Decal(zone, "FootwaySouth", "concrete", footwayInner, SiteMinX, -far, GateMinY, -AsphaltThickness, 0.0f);
            Decal(zone, "Crossover", "asphalt", footwayInner, SiteMinX, GateMinY, GateMaxY, -AsphaltThickness, 0.0f);
            var index = 1;
            for (var y = -far; y + CentreDash <= far; y += CentreDash + CentreGap)
            {
                Decal(zone, $"CentreDash_{index++:00}", "lane_marking", kerb - 3.5f - MarkingWidth * 0.5f,
                    kerb - 3.5f + MarkingWidth * 0.5f, y, y + CentreDash, -AsphaltThickness, MarkingTop);
            }
            return zone;
        }

        public static ZoneLayout Spine()
        {
            var zone = new ZoneLayout { Zone = "Spine" };
            Decal(zone, "Asphalt", "asphalt", SpineMinX, SpineMaxX, SpineMinY, SpineMaxY, -AsphaltThickness, 0.0f);

            var edgeY = SpineMaxY - EdgeLineInset - MarkingWidth * 0.5f;
            Line(zone, "EdgeNorth", edgeY, NorthEdgeBreaks);
            Line(zone, "EdgeSouth", -edgeY, SouthEdgeBreaks);

            var index = 1;
            for (var x = MarkingStartX; x + CentreDash <= MarkingEndX + 1e-4f; x += CentreDash + CentreGap)
            {
                Decal(zone, $"CentreDash_{index++:00}", "lane_marking", x, x + CentreDash,
                    -MarkingWidth * 0.5f, MarkingWidth * 0.5f, -AsphaltThickness, MarkingTop);
            }

            zone.Features.ClearRects.Add(Rect("Spine", SpineMinX, SpineMaxX, SpineMinY, SpineMaxY));
            return zone;
        }

        public static ZoneLayout Building()
        {
            var zone = new ZoneLayout { Zone = "Building" };
            var southY = BuildingMinY + ColumnSize * 0.5f;            // column centres, flush outside
            var northY = BuildingMaxY - ColumnSize * 0.5f;
            var westX = BuildingMinX + ColumnSize * 0.5f;
            var eastX = BuildingMaxX - ColumnSize * 0.5f;
            var southWallY = BuildingMinY + WallThickness * 0.5f;
            var northWallY = BuildingMaxY - WallThickness * 0.5f;
            var westWallX = BuildingMinX + WallThickness * 0.5f;
            var eastWallX = BuildingMaxX - WallThickness * 0.5f;
            var c = ColumnSize * 0.5f;

            // Reinforced-concrete frame: perimeter columns flush with the outside face, one
            // interior row. The north-east corner is the core.
            Piece(zone, "ColumnSW", "column_rc", westX, southY);
            Piece(zone, "ColumnSE", "column_rc", eastX, southY);
            Piece(zone, "ColumnNW", "column_rc", westX, northY);
            Piece(zone, "ColumnW", "column_rc", westX, ColumnRowY);
            Piece(zone, "ColumnE", "column_rc", eastX, ColumnRowY);
            for (var i = 0; i < ColumnGridX.Length; i++)
            {
                Piece(zone, $"ColumnS{i + 1}", "column_rc", ColumnGridX[i], southY);
                Piece(zone, $"ColumnN{i + 1}", "column_rc", ColumnGridX[i], northY);
                Piece(zone, $"ColumnInterior{i + 1}", "column_rc", ColumnGridX[i], ColumnRowY);
            }

            // Blockwork infill between the columns, with the two door frames in the south wall.
            var door130 = (Door130CentreX - Door130FrameWidth * 0.5f, Door130CentreX + Door130FrameWidth * 0.5f);
            var door105 = (Door105CentreX - Door105FrameWidth * 0.5f, Door105CentreX + Door105FrameWidth * 0.5f);
            WallX(zone, "WallSouth", southWallY, new[]
            {
                (BuildingMinX + ColumnSize, ColumnGridX[0] - c), (ColumnGridX[0] + c, door130.Item1),
                (door130.Item2, ColumnGridX[1] - c), (ColumnGridX[1] + c, ColumnGridX[2] - c),
                (ColumnGridX[2] + c, door105.Item1), (door105.Item2, BuildingMaxX - ColumnSize),
            });
            WallX(zone, "WallNorth", northWallY, new[]
            {
                (BuildingMinX + ColumnSize, ColumnGridX[0] - c), (ColumnGridX[0] + c, ColumnGridX[1] - c),
                (ColumnGridX[1] + c, ColumnGridX[2] - c), (ColumnGridX[2] + c, BuildingMaxX),
            });
            WallY(zone, "WallWest", westWallX, new[]
            {
                (BuildingMinY + ColumnSize, ColumnRowY - c), (ColumnRowY + c, BuildingMaxY - ColumnSize),
            });
            WallY(zone, "WallEast", eastWallX, new[]
            {
                (BuildingMinY + ColumnSize, ColumnRowY - c), (ColumnRowY + c, BuildingMaxY - WallThickness),
            });
            Piece(zone, "DoorFrame130", "door_frame_1p30", Door130CentreX, southWallY);
            Piece(zone, "DoorFrame105", "door_frame_1p05", Door105CentreX, southWallY);

            // Ring beams on the walls, flush with the outside face, stopping at the core.
            var beamZ = StoreyHeight;
            var b = BeamWidth * 0.5f;
            Piece(zone, "BeamSouth", "beam_rc", (BuildingMinX + BuildingMaxX) * 0.5f, BuildingMinY + b, beamZ, 0.0f,
                BuildingMaxX - BuildingMinX);
            Piece(zone, "BeamNorth", "beam_rc", (BuildingMinX + CoreMinX) * 0.5f, BuildingMaxY - b, beamZ, 0.0f,
                CoreMinX - BuildingMinX);
            var sideFrom = BuildingMinY + BeamWidth;
            Piece(zone, "BeamWest", "beam_rc", BuildingMinX + b, (sideFrom + BuildingMaxY - BeamWidth) * 0.5f, beamZ,
                HalfPi, BuildingMaxY - BeamWidth - sideFrom);
            Piece(zone, "BeamEast", "beam_rc", BuildingMaxX - b, (sideFrom + CoreMinY) * 0.5f, beamZ, HalfPi,
                CoreMinY - sideFrom);

            // Stair and lift core: concrete walls in the north-east corner, one lift ahead of the
            // frame, stair and lift openings facing the slab.
            var t = CoreWallThickness;
            var innerNorth = BuildingMaxY - WallThickness;
            var innerEast = BuildingMaxX - WallThickness;
            CoreWall(zone, "CoreWest", CoreMinX + t * 0.5f, CoreMinY, innerNorth, 0.0f, alongY: true);
            CoreWall(zone, "CoreSouth1", CoreMinY + t * 0.5f, CoreMinX + t, 15.5f, 0.0f, alongY: false);
            CoreWall(zone, "CoreSouth2", CoreMinY + t * 0.5f, 16.4f, 17.4f, 0.0f, alongY: false);
            CoreWall(zone, "CoreSouth3", CoreMinY + t * 0.5f, 18.3f, innerEast, 0.0f, alongY: false);
            CoreWall(zone, "CoreLiftWall", 16.775f, CoreMinY + t, innerNorth, 0.0f, alongY: true);
            CoreWall(zone, "CoreUpperSouth", CoreMinY + t * 0.5f, CoreMinX, CoreMaxX, StoreyHeight, alongY: false);
            CoreWall(zone, "CoreUpperNorth", CoreMaxY - t * 0.5f, CoreMinX, CoreMaxX, StoreyHeight, alongY: false);
            CoreWall(zone, "CoreUpperWest", CoreMinX + t * 0.5f, CoreMinY + t, CoreMaxY - t, StoreyHeight, alongY: true);
            CoreWall(zone, "CoreUpperEast", CoreMaxX - t * 0.5f, CoreMinY + t, CoreMaxY - t, StoreyHeight, alongY: true);

            // Power-floated slab finish over the footprint (visual only, flush with the floor).
            var slab = Piece(zone, "SlabFinish", "slab_finish", (BuildingMinX + BuildingMaxX) * 0.5f,
                (BuildingMinY + BuildingMaxY) * 0.5f, SlabTop - 0.01f);
            slab.LengthX = BuildingMaxX - BuildingMinX;

            var frameFace = southWallY - 0.11f;
            zone.Features.Doors.Add(new DoorFeature
            {
                Name = "Door130", CentreX = Door130CentreX, CentreY = southWallY, WallYaw = 0.0f, ClearWidth = 1.30f
            });
            zone.Features.Doors.Add(new DoorFeature
            {
                Name = "Door105", CentreX = Door105CentreX, CentreY = southWallY, WallYaw = 0.0f, ClearWidth = 1.05f
            });
            zone.Features.ClearRects.Add(Rect("Forecourt", BuildingMinX, BuildingMaxX, ForecourtMinY, frameFace));
            return zone;
        }

        private static KitPlacement Piece(ZoneLayout zone, string name, string pieceId, double x, double y,
            double z = 0.0, double yaw = 0.0, float lengthX = 0.0f)
        {
            var placement = new KitPlacement
            {
                Zone = zone.Zone, Name = name, PieceId = pieceId, X = x, Y = y, Z = z, Yaw = yaw, LengthX = lengthX
            };
            zone.Pieces.Add(placement);
            return placement;
        }

        private static void Post(ZoneLayout zone, string name, float x, float y)
        {
            zone.Boxes.Add(new PrimitiveBox
            {
                Zone = zone.Zone, Name = name, Material = "galvanized_steel", Collider = true,
                Box = new RosBox(new Vector3(x, y, CornerPostHeight * 0.5f),
                    new Vector3(CornerPostSize, CornerPostSize, CornerPostHeight))
            });
        }

        // Panels of one piece along a straight run from start to end. The first panel starts at
        // start and the last ends at end; when the run is not a whole number of panels, the
        // panels overlap evenly, alternate panels step FenceStagger to either side so their
        // meshes clear, and each pair of neighbours (sharing feet) is declared as a joint.
        private static void Run(ZoneLayout zone, string name, string pieceId, float pieceLength, Vector2 start,
            Vector2 end)
        {
            var delta = end - start;
            var length = delta.magnitude;
            var direction = delta / length;
            var normal = new Vector2(-direction.y, direction.x);
            var count = Mathf.Max(1, Mathf.CeilToInt(length / pieceLength - 1e-4f));
            var step = count == 1 ? 0.0f : (length - pieceLength) / (count - 1);
            var staggered = step < pieceLength - 1e-4f && count > 1;
            var yaw = Mathf.Atan2(direction.y, direction.x);
            for (var i = 0; i < count; i++)
            {
                var offset = staggered ? (i % 2 == 0 ? -FenceStagger : FenceStagger) : 0.0f;
                var centre = start + direction * (pieceLength * 0.5f + step * i) + normal * offset;
                Piece(zone, $"{name}_{i + 1:00}", pieceId, centre.x, centre.y, 0.0, yaw);
                if (staggered && i > 0)
                {
                    zone.Features.Joints.Add(new JointPair
                    {
                        A = $"{zone.Zone}/{name}_{i:00}", B = $"{zone.Zone}/{name}_{i + 1:00}"
                    });
                }
            }
        }

        private static void WallX(ZoneLayout zone, string name, float y, (float from, float to)[] segments)
        {
            for (var i = 0; i < segments.Length; i++)
            {
                var (from, to) = segments[i];
                Piece(zone, $"{name}_{i + 1}", "wall_blockwork", (from + to) * 0.5f, y, 0.0, 0.0, to - from);
            }
        }

        private static void WallY(ZoneLayout zone, string name, float x, (float from, float to)[] segments)
        {
            for (var i = 0; i < segments.Length; i++)
            {
                var (from, to) = segments[i];
                Piece(zone, $"{name}_{i + 1}", "wall_blockwork", x, (from + to) * 0.5f, 0.0, HalfPi, to - from);
            }
        }

        private static void CoreWall(ZoneLayout zone, string name, float line, float from, float to, float z, bool alongY)
        {
            var mid = (from + to) * 0.5f;
            if (alongY)
                Piece(zone, name, "core_wall", line, mid, z, HalfPi, to - from);
            else
                Piece(zone, name, "core_wall", mid, line, z, 0.0, to - from);
        }

        private static void Decal(ZoneLayout zone, string name, string material, float minX, float maxX, float minY,
            float maxY, float bottom, float top)
        {
            zone.Boxes.Add(new PrimitiveBox
            {
                Zone = zone.Zone, Name = name, Material = material, Collider = false,
                Box = new RosBox(new Vector3((minX + maxX) * 0.5f, (minY + maxY) * 0.5f, (bottom + top) * 0.5f),
                    new Vector3(maxX - minX, maxY - minY, top - bottom))
            });
        }

        // A solid lane line along x at y, from MarkingStartX to MarkingEndX, broken where lanes join.
        private static void Line(ZoneLayout zone, string name, float y, (float from, float to)[] breaks)
        {
            var pieces = new List<(float from, float to)>();
            var x = MarkingStartX;
            foreach (var (from, to) in breaks)
            {
                pieces.Add((x, from));
                x = to;
            }
            pieces.Add((x, MarkingEndX));
            for (var i = 0; i < pieces.Count; i++)
            {
                Decal(zone, $"{name}_{i + 1}", "lane_marking", pieces[i].from, pieces[i].to,
                    y - MarkingWidth * 0.5f, y + MarkingWidth * 0.5f, -AsphaltThickness, MarkingTop);
            }
        }

        private static ClearRect Rect(string name, float minX, float maxX, float minY, float maxY)
        {
            return new ClearRect
            {
                Name = name, CentreX = (minX + maxX) * 0.5f, CentreY = (minY + maxY) * 0.5f,
                SizeX = maxX - minX, SizeY = maxY - minY, Yaw = 0.0f
            };
        }
    }
}
