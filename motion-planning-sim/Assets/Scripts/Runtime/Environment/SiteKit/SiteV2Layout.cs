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
        // Physics material asset name for a collider that is not the kit default (friction patches).
        public string PhysicsMaterial;
    }

    // An optional Megascans scan (git-ignored, so present only on machines that downloaded it)
    // placed as visual-only dressing at a ROS map position, yaw in degrees. Dressing stands only
    // where the robot cannot reach (outside the site), since it has no collider.
    public sealed class ScanDressing
    {
        public string Name;
        public string AssetPath;
        public float X;
        public float Y;
        public float YawDegrees;
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
    public static partial class SiteV2Layout
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
        // Walkway crossings (zebras over the spine): W1 laydown to welfare, W2 building forecourt to
        // delivery yard, in line with door A (clear x 4.625..5.925).
        public const float W1MinX = -13.0f;
        public const float W1MaxX = -11.5f;
        public const float W2MinX = Door130CentreX - 0.75f;
        public const float W2MaxX = Door130CentreX + 0.75f;
        public const float W2WalkwayMinY = -6.0f;
        private const float ZebraBar = 0.5f;
        // Centre dashes stop this far short of a crossing.
        private const float CrossingMargin = 0.5f;
        // Edge-line breaks where crossings, lanes and yards join the spine.
        private static readonly (float from, float to)[] NorthEdgeBreaks =
        {
            (W1MinX, W1MaxX), (-8.0f, -5.0f), (W2MinX, W2MaxX) // W1, scaffold tunnel, W2
        };
        private static readonly (float from, float to)[] SouthEdgeBreaks =
        {
            (W1MinX, W1MaxX), (-7.0f, 7.0f), (15.0f, 17.6f) // W1, delivery yard (W2 inside it), services
        };
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
        // Both door axes sit on a 5 cm map cell centre (the V2 map origin is x -25), so a planner route down
        // the axis lies on one cell column instead of splitting two.
        public const float Door130CentreX = 5.275f;
        public const float Door105CentreX = 16.275f;
        public const float Door130FrameWidth = 1.42f;
        public const float Door105FrameWidth = 1.17f;
        public const float ForecourtMinY = 2.5f;
        public const float ForecourtMaxY = 5.5f;
        // Door approach lanes: two runs of precast block barriers (barrier_block_0p8 turned, 0.55 m
        // wide, 2.5 m long) in front of each south door, from the wall face south, symmetric about
        // the door axis. The lane between them is ApproachLaneWidth clear, so a route can no longer
        // cut to a door diagonally across the forecourt: it runs straight and centred for the last
        // 2.5 m (the base-only route of B5 reached door B 4 deg off-axis and 7 cm off-centre
        // without them, and the collision monitor's stop zone touched a jamb).
        public const float ApproachLaneWidth = 1.6f;
        private const float LaneBarrierWidth = 0.55f;
        private const float LaneBarrierLength = 2.5f;
        public const float LaneBarrierMinY = ForecourtMaxY - LaneBarrierLength;
        // The slab finish is visual only; its top sits just above the ground decal.
        public const float SlabTop = 0.0005f;

        // Laydown yard: a chicane aisle through four staggered rows of tall block stacks, bounded by
        // low stacks (under the panel) and timber and rebar on bearers that close the bypasses.
        public const float LaydownMinX = -23.0f;
        public const float LaydownMaxX = -9.5f;
        public const float ScaffoldGravelMaxX = -8.0f; // gravel runs on under the west scaffold bays
        public const float LaydownMinY = 2.5f;
        public const float LaydownMaxY = 18.0f;
        public const float ChicaneWestX = -21.5f;      // east face of the west low stacks
        public const float ChicaneGap = 2.4f;
        public const float ChicaneFirstRowY = 3.7f;
        public const int ChicaneRows = 4;
        public const int StacksPerRow = 6;
        private const float StackGap = 0.05f;          // between neighbouring pallets
        private const float StoreGap = 0.1f;           // between rows of stored material
        private const float TallStackWidth = 0.8f;     // stack_block_tall turned: 0.8 along x, 1.2 along y
        private const float TallStackDepth = 1.2f;
        private const float TallRowLength = StacksPerRow * TallStackWidth + (StacksPerRow - 1) * StackGap;
        public const float ChicaneWestRowMinX = ChicaneWestX + StoreGap;
        public const float ChicaneEastRowMinX = ChicaneWestX + ChicaneGap;
        public const float ChicaneEastX = ChicaneWestRowMinX + TallRowLength + ChicaneGap;

        // Scaffold tunnel: a 3.0 m lane between two runs of 2.5 x 1.2 m scaffold bays; the only
        // member across the lane below 2.4 m is the H1 ledger. Turnaround pocket at the north end.
        public const float TunnelMinX = -8.0f;
        public const float TunnelMaxX = -5.0f;
        public const float TunnelMinY = 2.5f;
        public const float PocketMinX = -9.5f;
        public const float PocketMaxX = -3.5f;
        public const float PocketMinY = 16.6f;
        public const float PocketMaxY = 19.2f;
        public const float ScaffoldFirstY = 4.5f;      // standard lines every bay length
        public const int ScaffoldBays = 4;
        public const float BayLength = 2.5f;
        public const float BayWidth = 1.2f;
        public const float H1Y = 7.0f;                 // on the second standard line
        // Ledger tube bottoms on the rosettes at 0.444 + 0.5 k (kit scaffold family): base lift,
        // mid lift (H1) and the deck transoms; boards rest on the transom wedge heads.
        public const float H1BottomZ = 1.42f;          // kit overhead.h1_bottom_m
        private const float BaseLiftZ = 0.42f;
        private const float DeckTransomZ = 1.92f;
        private const float BoardZ = DeckTransomZ + 0.08f;
        private const float RosetteDiameter = 0.124f;
        private const float StandardPlate = 0.15f;
        public const string H1Member = "Scaffold/H1Ledger";

        // Office and welfare: 20 ft cabins facing the spine (their fronts are the -y side, so they
        // are turned half round); the drying room sits behind the office.
        public const float CabinFrontY = -5.08f;
        private const float CabinWidth = 2.438f;
        public const float OfficeX = -21.3f;
        public const float WelfareX = -14.25f;         // its door (piece x -2.0) on walkway W1
        public const float DryingRoomY = -9.9f;

        // Delivery yard: asphalt hardstanding south of the spine; an open area in the north with the
        // skip on its east edge, the serpentine store (SiteV2Layout.Interiors.cs) in the south.
        public const float YardMinX = -7.0f;
        public const float YardMaxX = 7.0f;
        public const float YardMinY = -17.0f;
        public const float YardMaxY = SpineMinY;
        public const float SkipX = 6.05f;              // skip turned: x 5.4..6.7, y -10.0..-7.8
        public const float SkipY = -8.9f;
        public const float SkipMinX = 5.4f;
        public const float SkipMaxY = -7.8f;
        public const float YardOpenMinY = -10.5f;      // the open area: x -7..7, y -10.5..-2.5

        // Services corridor: a 2.6 m dead-end lane with a turn bay, the pipe rack along its east
        // side, the H3 pipe bridge across it and block barriers along its west side.
        public const float ServicesMinX = 15.0f;
        public const float ServicesMaxX = 17.6f;
        public const float ServicesMinY = -18.0f;
        public const float TurnBayMinX = 10.0f;
        public const float TurnBayMinY = -12.0f;
        public const float TurnBayMaxY = -7.0f;
        public const float PipeRackX = 18.2f;
        public const float PipeRackZ = 1.2f;           // pipe_rack_support arm top
        public const float BridgeY = -14.0f;
        // The bridge's posts stand 3.8 m apart: centred at x 16.0 the east post clears the lane
        // (x 17.8..18.0) and the west post stands just behind the barrier line (x 13.9..14.1).
        public const float BridgeX = 16.0f;
        private const float BridgePostHalf = 0.1f;
        public const float BarrierX = 14.675f;         // barrier_block_0p8 turned: x 14.4..14.95
        public const float BarrierMinY = -17.6f;
        public const float DeadEndWallY = -18.1f;
        public const string H3Member = "Services/PipeBridge";          // the bridge deck and rails (not its posts)
        private const float H3MinBottomZ = 1.0f;                       // posts and feet reach the ground

        // Tier 1 terrain (spec Section 5): a spoil heap with stepped proxies and an open trench
        // inside proxied barriers. The trench is a visual pit only: the ground visual has a hole
        // over it, while the floor collider stays one flat box.
        public const float SpoilHeapX = 22.0f;          // spoil_heap turned: x 19.6..24.4, y 8..16
        public const float SpoilHeapY = 12.0f;
        // Trench edges on even metres from the site's south-west corner, so the ground visual's
        // texture (a 2 m tile) runs on without a seam around the hole.
        public const float TrenchMinX = 20.0f;
        public const float TrenchMaxX = 21.0f;
        public const float TrenchMinY = -18.0f;
        public const float TrenchMaxY = -4.0f;
        public const float TrenchDepth = 0.9f;
        private const float TrenchBarrierLength = 2.0f;
        private const float TrenchBarrierFeet = 0.5f;
        private const float TrenchBarrierStandOff = 0.6f;   // barrier line centre to trench edge

        private const float HalfPi = Mathf.PI * 0.5f;

        public static IReadOnlyList<ZoneLayout> Zones()
        {
            return new[]
            {
                Street(), Perimeter(), Spine(), Building(), Forecourt(), FitOut(), Laydown(), Scaffold(), Cabins(),
                Walkways(), DeliveryYard(), YardStore(), Services(), Terrain(), Gravel()
            };
        }

        // The Tier 2 disturbance groups (spec Section 5), built under Environment/Disturbances, which is
        // inactive in the saved scene and never exported. Not part of Zones(): these colliders are not
        // navigation obstacles.
        public static IReadOnlyList<ZoneLayout> Disturbances()
        {
            return new[] { RumbleStrips(), RoadPlates(), CableProtector(), GravelPatch(), MudPatch() };
        }

        // Friction patches are thin collider slabs standing PatchThickness above the floor: the base
        // rides on the patch alone (the floor is below its wheels), so only the patch material acts.
        public const float PatchThickness = 0.003f;

        // Four strips across the spine just inside the gate; each is two 2.5 m pieces end to end.
        public static ZoneLayout RumbleStrips()
        {
            var zone = new ZoneLayout { Zone = "RumbleStrips" };
            var xs = new[] { -19.8f, -19.4f, -19.0f, -18.6f };
            for (var i = 0; i < xs.Length; i++)
            {
                Piece(zone, $"Strip{i + 1}_South", "rumble_strip_0p02", xs[i], -1.25, 0.0, HalfPi);
                Piece(zone, $"Strip{i + 1}_North", "rumble_strip_0p02", xs[i], 1.25, 0.0, HalfPi);
            }
            return zone;
        }

        // Steel plates across the whole spine, x 9..11 (2 x 1.25 m, four side by side).
        public static ZoneLayout RoadPlates()
        {
            var zone = new ZoneLayout { Zone = "RoadPlates" };
            for (var i = 0; i < 4; i++)
                Piece(zone, $"Plate_{i + 1}", "road_plate_0p025", 10.0, SpineMinY + 0.625 + i * 1.25);
            return zone;
        }

        // A cable protector across the tunnel lane at its entrance (y 4.0..4.4), three 0.9 m pieces.
        public static ZoneLayout CableProtector()
        {
            var zone = new ZoneLayout { Zone = "CableProtector" };
            for (var i = 0; i < 3; i++)
                Piece(zone, $"Protector_{i + 1}", "cable_protector_0p035", -7.4 + i * 0.9, 4.2);
            return zone;
        }

        public static ZoneLayout GravelPatch()
        {
            var zone = new ZoneLayout { Zone = "GravelPatch" };
            Patch(zone, "Patch", "gravel", "gravel_patch", PocketMinX + 0.25f, PocketMaxX - 0.25f, PocketMinY + 0.25f,
                PocketMaxY - 0.25f);
            return zone;
        }

        public static ZoneLayout MudPatch()
        {
            var zone = new ZoneLayout { Zone = "MudPatch" };
            // In store aisle 2 (YardStore), clear of the hoarding rows.
            Patch(zone, "Patch", "mud", "mud_patch", -5.5f, -1.5f, -15.6f, -13.6f);
            return zone;
        }

        private static void Patch(ZoneLayout zone, string name, string material, string physicsMaterial, float minX,
            float maxX, float minY, float maxY)
        {
            zone.Boxes.Add(new PrimitiveBox
            {
                Zone = zone.Zone, Name = name, Material = material, Collider = true, PhysicsMaterial = physicsMaterial,
                Box = new RosBox(new Vector3((minX + maxX) * 0.5f, (minY + maxY) * 0.5f, PatchThickness * 0.5f),
                    new Vector3(maxX - minX, maxY - minY, PatchThickness))
            });
        }

        public static List<NamedBox> DisturbanceColliderBoxes(KitSpec spec) => ColliderBoxes(spec, Disturbances());

        // Every collider box of the layout, named as the builder names its proxies
        // (<Zone>/<Name>/proxy_<i>).
        public static List<NamedBox> ColliderBoxes(KitSpec spec) => ColliderBoxes(spec, Zones());

        private static List<NamedBox> ColliderBoxes(KitSpec spec, IReadOnlyList<ZoneLayout> zones)
        {
            var boxes = new List<NamedBox>();
            foreach (var zone in zones)
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
            // Overhead members at the kit's heights (the layout hangs them on its own constants,
            // so a kit change that the layout does not follow shows up here).
            features.Overheads.Add(new OverheadFeature { Name = H1Member, BottomZ = spec.Overhead.h1_bottom_m });
            features.Overheads.Add(new OverheadFeature { Name = H3Member, BottomZ = spec.Overhead.h3_bottom_m, MinBottomZ = H3MinBottomZ });
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

            // Centre dashes, cut back where they would run into a crossing (pieces under 1 m dropped).
            var index = 1;
            var crossings = new[] { (W1MinX - CrossingMargin, W1MaxX + CrossingMargin), (W2MinX - CrossingMargin, W2MaxX + CrossingMargin) };
            for (var x = MarkingStartX; x + CentreDash <= MarkingEndX + 1e-4f; x += CentreDash + CentreGap)
            {
                var pieces = new List<(float from, float to)> { (x, x + CentreDash) };
                foreach (var (from, to) in crossings)
                {
                    var cut = new List<(float from, float to)>();
                    foreach (var piece in pieces)
                    {
                        if (piece.to <= from || piece.from >= to)
                        {
                            cut.Add(piece);
                            continue;
                        }
                        if (from - piece.from >= 1.0f)
                            cut.Add((piece.from, from));
                        if (piece.to - to >= 1.0f)
                            cut.Add((to, piece.to));
                    }
                    pieces = cut;
                }
                foreach (var piece in pieces)
                {
                    Decal(zone, $"CentreDash_{index++:00}", "lane_marking", piece.from, piece.to,
                        -MarkingWidth * 0.5f, MarkingWidth * 0.5f, -AsphaltThickness, MarkingTop);
                }
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

            zone.Features.Doors.Add(new DoorFeature
            {
                Name = "Door130", CentreX = Door130CentreX, CentreY = southWallY, WallYaw = 0.0f, ClearWidth = 1.30f
            });
            zone.Features.Doors.Add(new DoorFeature
            {
                Name = "Door105", CentreX = Door105CentreX, CentreY = southWallY, WallYaw = 0.0f, ClearWidth = 1.05f
            });
            return zone;
        }

        // The forecourt between the spine and the building, with an approach lane in front of each
        // door. Walkway W2 (x 4.525..6.025) runs inside door A's lane. Clear: each lane (forecourt edge to
        // the door frame), the strip south of the barriers, and the forecourt either side of the lanes.
        public static ZoneLayout Forecourt()
        {
            var zone = new ZoneLayout { Zone = "Forecourt" };
            var frameFace = BuildingMinY + WallThickness * 0.5f - 0.11f; // the frames stand 0.01 m proud of the wall
            var barrierY = LaneBarrierMinY + LaneBarrierLength * 0.5f;
            var offset = (ApproachLaneWidth + LaneBarrierWidth) * 0.5f;
            var lanes = new[] { ("Door130", Door130CentreX), ("Door105", Door105CentreX) };
            var outer = new List<float> { BuildingMinX };
            foreach (var (door, centreX) in lanes)
            {
                Piece(zone, $"Lane{door}_West", "barrier_block_0p8", centreX - offset, barrierY, 0.0, HalfPi);
                Piece(zone, $"Lane{door}_East", "barrier_block_0p8", centreX + offset, barrierY, 0.0, HalfPi);
                zone.Features.ClearRects.Add(Rect(ApproachLaneName(door), centreX - ApproachLaneWidth * 0.5f,
                    centreX + ApproachLaneWidth * 0.5f, ForecourtMinY, frameFace));
                outer.Add(centreX - offset - LaneBarrierWidth * 0.5f);
                outer.Add(centreX + offset + LaneBarrierWidth * 0.5f);
            }
            outer.Add(BuildingMaxX);
            string[] sides = { "ForecourtWest", "ForecourtMid", "ForecourtEast" };
            for (var i = 0; i < sides.Length; i++)
                zone.Features.ClearRects.Add(Rect(sides[i], outer[2 * i], outer[2 * i + 1], ForecourtMinY, frameFace));
            zone.Features.ClearRects.Add(Rect("ForecourtFront", BuildingMinX, BuildingMaxX, ForecourtMinY, LaneBarrierMinY));
            return zone;
        }

        // The clear rectangle of a south door's approach lane (door feature name).
        public static string ApproachLaneName(string door) => $"ApproachLane{door}";

        // Megascans dressing outside the fence: traffic cones on the street footway either side of
        // the gate crossover.
        public static IReadOnlyList<ScanDressing> Dressing()
        {
            const string cone = "Assets/Models/roadside_construction_mid/vhvpabi/vhvpabi.fbx";
            var items = new List<ScanDressing>();
            var index = 1;
            foreach (var y in new[] { GateMaxY + 0.4f, GateMinY - 0.4f })
            {
                foreach (var x in new[] { SiteMinX - 0.5f, SiteMinX - 1.5f })
                {
                    items.Add(new ScanDressing
                    {
                        Name = $"TrafficCone_{index:00}", AssetPath = cone, X = x, Y = y, YawDegrees = 20.0f * index
                    });
                    index++;
                }
            }
            return items;
        }

        public static ZoneLayout Laydown()
        {
            var zone = new ZoneLayout { Zone = "Laydown" };
            var pitch = TallStackDepth + ChicaneGap;
            var westRowMaxX = ChicaneWestRowMinX + TallRowLength;
            var lastRowMaxY = ChicaneFirstRowY + (ChicaneRows - 1) * pitch + TallStackDepth;

            // Four rows of tall block stacks, alternately against the west and east sides, so the
            // aisle runs up the east side, across, up the west side and so on, every gap 2.4 m.
            for (var row = 0; row < ChicaneRows; row++)
            {
                var minX = row % 2 == 0 ? ChicaneWestRowMinX : ChicaneEastRowMinX;
                var y = ChicaneFirstRowY + row * pitch + TallStackDepth * 0.5f;
                for (var i = 0; i < StacksPerRow; i++)
                {
                    Piece(zone, $"Chicane{row + 1}_{i + 1}", "stack_block_tall",
                        minX + TallStackWidth * 0.5f + i * (TallStackWidth + StackGap), y, 0.0, HalfPi);
                }
            }

            // The aisle's sides: low pallet stacks (0.8 m) on the west and panel stacks (0.6 m) on
            // the east, low enough for the carried panel to overhang them.
            var lowPitch = 1.2f + StoreGap;
            for (var i = 0; ChicaneFirstRowY + i * lowPitch + 1.2f <= lastRowMaxY + 1e-4f; i++)
            {
                Piece(zone, $"LowPalletsWest_{i + 1}", "stack_pallet_low_0p8", ChicaneWestX - 0.4f,
                    ChicaneFirstRowY + 0.6f + i * lowPitch, 0.0, HalfPi);
            }
            var panelPitch = 2.4f + StackGap;
            for (var i = 0; i < 5; i++)
            {
                Piece(zone, $"PanelsEast_{i + 1}", "stack_panel", ChicaneEastX + 0.6f,
                    ChicaneFirstRowY + 1.2f + i * panelPitch, 0.0, HalfPi);
            }

            // Timber and rebar on bearers behind the sides. Every gap they leave is narrower than
            // the base, so the chicane is the only way north through the yard.
            var timberWestX = ChicaneWestX - 0.8f - StoreGap - 0.6f;
            for (var i = 0; i < 4; i++)
                Piece(zone, $"TimberWest_{i + 1}", "bearers_timber", timberWestX, 3.5f + 1.8f + i * (3.6f + StackGap), 0.0, HalfPi);
            var rebarX = ChicaneEastX + 1.2f + StoreGap + 0.6f;
            for (var i = 0; i < 2; i++)
                Piece(zone, $"Rebar_{i + 1}", "bearers_rebar", rebarX, ChicaneFirstRowY + 3.0f + i * (6.0f + StoreGap), 0.0, HalfPi);
            var timberEastX = rebarX + 0.6f + 0.45f + 0.6f;
            for (var i = 0; i < 3; i++)
                Piece(zone, $"TimberEast_{i + 1}", "bearers_timber", timberEastX, ChicaneFirstRowY + 1.8f + i * (3.6f + StackGap), 0.0, HalfPi);

            // The aisle stays clear: the entry from the spine, the gaps between the rows, the side
            // passages and the open strip at the north end.
            zone.Features.ClearRects.Add(Rect("ChicaneEntry", westRowMaxX, ChicaneEastX, LaydownMinY,
                ChicaneFirstRowY + TallStackDepth));
            for (var row = 0; row < ChicaneRows; row++)
            {
                var rowMinY = ChicaneFirstRowY + row * pitch;
                if (row > 0)
                {
                    if (row % 2 == 0)
                        zone.Features.ClearRects.Add(Rect($"ChicanePass{row + 1}", westRowMaxX, ChicaneEastX, rowMinY, rowMinY + TallStackDepth));
                    else
                        zone.Features.ClearRects.Add(Rect($"ChicanePass{row + 1}", ChicaneWestX, ChicaneEastRowMinX, rowMinY, rowMinY + TallStackDepth));
                }
                if (row < ChicaneRows - 1)
                {
                    zone.Features.ClearRects.Add(Rect($"ChicaneGap{row + 1}", ChicaneWestX, ChicaneEastX,
                        rowMinY + TallStackDepth, rowMinY + pitch));
                }
            }
            zone.Features.ClearRects.Add(Rect("LaydownNorth", ChicaneWestX, LaydownMaxX, 16.0f, LaydownMaxY));
            return zone;
        }

        public static ZoneLayout Scaffold()
        {
            var zone = new ZoneLayout { Zone = "Scaffold" };
            // Inner standards stand with their base plates on the lane edges.
            var westInner = TunnelMinX - StandardPlate * 0.5f;
            var eastInner = TunnelMaxX + StandardPlate * 0.5f;
            ScaffoldRun(zone, "West", westInner, westInner - BayWidth);
            ScaffoldRun(zone, "East", eastInner, eastInner + BayWidth);

            // H1: one ledger across the lane between the inner standards on the y 7 line, hung on
            // the 1.444 m rosettes, stretched so its heads meet the standard tubes.
            Piece(zone, "H1Ledger", "scaffold_ledger_2p5", (westInner + eastInner) * 0.5f, H1Y, H1BottomZ, 0.0,
                eastInner - westInner - RosetteDiameter);

            // An asphalt lane off the spine with edge lines, and the pocket at its end.
            Flush(zone, "Lane", "asphalt", TunnelMinX, TunnelMaxX, TunnelMinY, PocketMinY);
            Flush(zone, "Pocket", "asphalt", PocketMinX, PocketMaxX, PocketMinY, PocketMaxY);
            LineY(zone, "EdgeWest", TunnelMinX + EdgeLineInset + MarkingWidth * 0.5f, TunnelMinY, PocketMinY);
            LineY(zone, "EdgeEast", TunnelMaxX - EdgeLineInset - MarkingWidth * 0.5f, TunnelMinY, PocketMinY);

            zone.Features.ClearRects.Add(Rect("TunnelLane", TunnelMinX, TunnelMaxX, TunnelMinY, PocketMinY));
            zone.Features.ClearRects.Add(Rect("TurnaroundPocket", PocketMinX, PocketMaxX, PocketMinY, PocketMaxY));
            return zone;
        }

        // Zebra crossings over the spine and yellow walkway edges on to the welfare cabin (W1), the
        // building forecourt and the delivery yard (W2). All flush paint, no colliders.
        public static ZoneLayout Walkways()
        {
            var zone = new ZoneLayout { Zone = "Walkways" };
            Zebra(zone, "W1", W1MinX, W1MaxX);
            Zebra(zone, "W2", W2MinX, W2MaxX);
            WalkwayPath(zone, "W1South", W1MinX, W1MaxX, CabinFrontY, SpineMinY);
            WalkwayPath(zone, "W2North", W2MinX, W2MaxX, SpineMaxY, BuildingMinY);
            WalkwayPath(zone, "W2South", W2MinX, W2MaxX, W2WalkwayMinY, SpineMinY);
            return zone;
        }

        // White bars across the spine, ZebraBar wide with equal gaps, centred on the road.
        private static void Zebra(ZoneLayout zone, string name, float minX, float maxX)
        {
            var bars = Mathf.FloorToInt((SpineMaxY - SpineMinY - 2.0f * ZebraBar) / (2.0f * ZebraBar)) + 1;
            var first = -(bars - 1) * ZebraBar;
            for (var i = 0; i < bars; i++)
            {
                var y = first + i * 2.0f * ZebraBar;
                Decal(zone, $"{name}Zebra_{i + 1}", "lane_marking", minX, maxX, y - ZebraBar * 0.5f, y + ZebraBar * 0.5f,
                    -AsphaltThickness, MarkingTop);
            }
        }

        // A pedestrian route off the spine: yellow lines along both edges.
        private static void WalkwayPath(ZoneLayout zone, string name, float minX, float maxX, float minY, float maxY)
        {
            LineY(zone, $"{name}West", minX + MarkingWidth * 0.5f, minY, maxY, "walkway_marking");
            LineY(zone, $"{name}East", maxX - MarkingWidth * 0.5f, minY, maxY, "walkway_marking");
        }

        public static ZoneLayout DeliveryYard()
        {
            var zone = new ZoneLayout { Zone = "DeliveryYard" };
            // Asphalt hardstanding with white edge lines on its three open sides.
            Flush(zone, "Hardstanding", "asphalt", YardMinX, YardMaxX, YardMinY, YardMaxY);
            var inset = EdgeLineInset + MarkingWidth * 0.5f;
            var southLineY = YardMinY + inset;
            Decal(zone, "EdgeSouth", "lane_marking", YardMinX + inset - MarkingWidth * 0.5f, YardMaxX - inset + MarkingWidth * 0.5f,
                southLineY - MarkingWidth * 0.5f, southLineY + MarkingWidth * 0.5f, -AsphaltThickness, MarkingTop);
            LineY(zone, "EdgeWest", YardMinX + inset, southLineY + MarkingWidth * 0.5f, YardMaxY);
            LineY(zone, "EdgeEast", YardMaxX - inset, southLineY + MarkingWidth * 0.5f, YardMaxY);
            Piece(zone, "Skip", "skip", SkipX, SkipY, 0.0, HalfPi);
            // The open area x -7..7, y -10.5..-2.5 (14 x 8 m) is clear apart from the skip on its east
            // edge, so it is declared as the part west of the skip plus the strip north of it.
            zone.Features.ClearRects.Add(Rect("DeliveryYardOpen", YardMinX, SkipMinX, YardOpenMinY, YardMaxY));
            zone.Features.ClearRects.Add(Rect("DeliveryYardOpenEast", SkipMinX, YardMaxX, SkipMaxY, YardMaxY));
            return zone;
        }

        public static ZoneLayout Services()
        {
            var zone = new ZoneLayout { Zone = "Services" };
            Flush(zone, "Lane", "asphalt", ServicesMinX, ServicesMaxX, ServicesMinY, SpineMinY);
            Flush(zone, "TurnBay", "asphalt", TurnBayMinX, ServicesMinX, TurnBayMinY, TurnBayMaxY);
            var westLine = ServicesMinX + EdgeLineInset + MarkingWidth * 0.5f;
            LineY(zone, "EdgeWest_1", westLine, ServicesMinY, TurnBayMinY);
            LineY(zone, "EdgeWest_2", westLine, TurnBayMaxY, SpineMinY);
            LineY(zone, "EdgeEast", ServicesMaxX - EdgeLineInset - MarkingWidth * 0.5f, ServicesMinY, SpineMinY);

            // H3: the pipe bridge across the lane, deck bottom at 2.40 m.
            Piece(zone, "PipeBridge", "pipe_bridge", BridgeX, BridgeY);

            // Pipe rack on T-supports along the east side. Its pipes run into the bridge's east
            // post from both sides (they rise there and cross on the deck).
            var postSouth = BridgeY - BridgePostHalf;
            var postNorth = BridgeY + BridgePostHalf;
            const float rackSouth = -17.6f, rackNorth = -3.0f;
            var supports = new[] { -17.0f, -15.0f, -12.5f, -9.5f, -6.5f, -3.5f };
            for (var i = 0; i < supports.Length; i++)
                Piece(zone, $"RackSupport_{i + 1}", "pipe_rack_support", PipeRackX, supports[i]);
            Piece(zone, "PipesSouth", "pipe_run", PipeRackX, (rackSouth + postSouth) * 0.5f, PipeRackZ, HalfPi,
                postSouth - rackSouth);
            Piece(zone, "PipesNorth", "pipe_run", PipeRackX, (postNorth + rackNorth) * 0.5f, PipeRackZ, HalfPi,
                rackNorth - postNorth);

            // Precast block barriers along the west side, in front of the bridge's west post.
            for (var i = 0; i < 2; i++)
                Piece(zone, $"Barrier_{i + 1}", "barrier_block_0p8", BarrierX, BarrierMinY + 1.25f + i * 2.5f, 0.0, HalfPi);

            // Dead-end wall across the lane's end, from the barrier line to the lane's east edge.
            var wallFrom = BarrierX - 0.275f;
            Piece(zone, "DeadEndWall", "wall_blockwork", (wallFrom + ServicesMaxX) * 0.5f, DeadEndWallY, 0.0, 0.0,
                ServicesMaxX - wallFrom);

            zone.Features.ClearRects.Add(Rect("ServicesLane", ServicesMinX, ServicesMaxX, ServicesMinY, SpineMinY));
            zone.Features.ClearRects.Add(Rect("TurnBay", TurnBayMinX, ServicesMinX, TurnBayMinY, TurnBayMaxY));
            return zone;
        }

        // Gravel ground (flush, visual only) under the laydown yard (hoarding to the scaffold run) and
        // around the office and welfare cabins (hoarding to the delivery yard), so the yards read
        // apart from the bare ground.
        public static ZoneLayout Gravel()
        {
            var zone = new ZoneLayout { Zone = "Gravel" };
            var hoardingInnerX = WestLineX + 0.085f;
            var fenceInnerY = NorthLineY - 0.105f;
            Flush(zone, "Laydown", "gravel", hoardingInnerX, LaydownMaxX, LaydownMinY, fenceInnerY);
            // Runs on under the west scaffold bays (x -9.5 .. -8), stopping at the turnaround pocket.
            Flush(zone, "LaydownScaffold", "gravel", LaydownMaxX, ScaffoldGravelMaxX, LaydownMinY, PocketMinY);
            Flush(zone, "Cabins", "gravel", hoardingInnerX, YardMinX, -12.0f, SpineMinY);
            return zone;
        }

        public static ZoneLayout Terrain()
        {
            var zone = new ZoneLayout { Zone = "Terrain" };
            Piece(zone, "SpoilHeap", "spoil_heap", SpoilHeapX, SpoilHeapY, 0.0, HalfPi);

            // Trench barriers end to end along both sides (feet touching), closed at both ends.
            var trenchMidX = (TrenchMinX + TrenchMaxX) * 0.5f;
            var westX = TrenchMinX - TrenchBarrierStandOff;
            var eastX = TrenchMaxX + TrenchBarrierStandOff;
            var runMinY = TrenchMinY;
            var count = Mathf.RoundToInt((TrenchMaxY - TrenchMinY) / TrenchBarrierLength);
            for (var i = 0; i < count; i++)
            {
                var y = runMinY + (i + 0.5f) * TrenchBarrierLength;
                Piece(zone, $"TrenchBarrierWest_{i + 1}", "trench_barrier", westX, y, 0.0, HalfPi);
                Piece(zone, $"TrenchBarrierEast_{i + 1}", "trench_barrier", eastX, y, 0.0, HalfPi);
            }
            var runMaxY = runMinY + count * TrenchBarrierLength;
            Piece(zone, "TrenchBarrierNorth", "trench_barrier", trenchMidX, runMaxY + TrenchBarrierFeet * 0.5f);
            Piece(zone, "TrenchBarrierSouth", "trench_barrier", trenchMidX, runMinY - TrenchBarrierFeet * 0.5f);

            // The pit under the ground visual's hole: four walls and a floor (visual only).
            const float wall = 0.05f;
            var top = -AsphaltThickness;
            var bottom = -TrenchDepth;
            Decal(zone, "TrenchFloor", "compacted_ground", TrenchMinX, TrenchMaxX, TrenchMinY, TrenchMaxY, bottom - wall, bottom);
            Decal(zone, "TrenchWallWest", "compacted_ground", TrenchMinX, TrenchMinX + wall, TrenchMinY, TrenchMaxY, bottom, top);
            Decal(zone, "TrenchWallEast", "compacted_ground", TrenchMaxX - wall, TrenchMaxX, TrenchMinY, TrenchMaxY, bottom, top);
            Decal(zone, "TrenchWallSouth", "compacted_ground", TrenchMinX + wall, TrenchMaxX - wall, TrenchMinY, TrenchMinY + wall, bottom, top);
            Decal(zone, "TrenchWallNorth", "compacted_ground", TrenchMinX + wall, TrenchMaxX - wall, TrenchMaxY - wall, TrenchMaxY, bottom, top);
            return zone;
        }

        public static ZoneLayout Cabins()
        {
            var zone = new ZoneLayout { Zone = "Cabins" };
            var rowY = CabinFrontY - CabinWidth * 0.5f;
            Piece(zone, "Office", "cabin_office", OfficeX, rowY, 0.0, Mathf.PI);
            Piece(zone, "Welfare", "cabin_welfare", WelfareX, rowY, 0.0, Mathf.PI);
            Piece(zone, "DryingRoom", "cabin_drying", OfficeX, DryingRoomY, 0.0, Mathf.PI);
            return zone;
        }

        // One run of bays along y between an inner and an outer line of standards: ledgers along
        // the run at the base and mid lifts, transoms across at the base lift and under the deck,
        // three boards per bay on the deck transoms and toe boards on the outer boards' edges.
        private static void ScaffoldRun(ZoneLayout zone, string side, float innerX, float outerX)
        {
            var name = side;
            var centreX = (innerX + outerX) * 0.5f;
            const float board = 0.32f, boardGap = 0.008f, toeBoard = 0.03f;
            var outward = Mathf.Sign(outerX - innerX);
            for (var k = 0; k <= ScaffoldBays; k++)
            {
                var y = ScaffoldFirstY + k * BayLength;
                Piece(zone, $"{name}StandardInner_{k + 1}", "scaffold_standard", innerX, y);
                Piece(zone, $"{name}StandardOuter_{k + 1}", "scaffold_standard", outerX, y);
                Piece(zone, $"{name}TransomBase_{k + 1}", "scaffold_transom_1p2", centreX, y, BaseLiftZ);
                Piece(zone, $"{name}TransomDeck_{k + 1}", "scaffold_transom_1p2", centreX, y, DeckTransomZ);
            }
            for (var k = 0; k < ScaffoldBays; k++)
            {
                var y = ScaffoldFirstY + (k + 0.5f) * BayLength;
                foreach (var (line, x) in new[] { ("Inner", innerX), ("Outer", outerX) })
                {
                    Piece(zone, $"{name}Ledger{line}Base_{k + 1}", "scaffold_ledger_2p5", x, y, BaseLiftZ, HalfPi);
                    Piece(zone, $"{name}Ledger{line}Mid_{k + 1}", "scaffold_ledger_2p5", x, y, H1BottomZ, HalfPi);
                }
                for (var b = -1; b <= 1; b++)
                {
                    Piece(zone, $"{name}Board{b + 2}_{k + 1}", "scaffold_board_2p5", centreX + b * (board + boardGap), y,
                        BoardZ, HalfPi);
                }
                // Toe boards on the outer edges of the outer boards, inside the standards.
                var edge = board * 1.5f + boardGap - toeBoard * 0.5f;
                Piece(zone, $"{name}ToeBoardInner_{k + 1}", "scaffold_toe_board_2p5", centreX - outward * edge, y,
                    BoardZ + 0.05f, HalfPi);
                Piece(zone, $"{name}ToeBoardOuter_{k + 1}", "scaffold_toe_board_2p5", centreX + outward * edge, y,
                    BoardZ + 0.05f, HalfPi);
            }
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

        // A flush ground surface (asphalt, gravel, concrete hardstanding): top at the floor, no collider.
        private static void Flush(ZoneLayout zone, string name, string material, float minX, float maxX, float minY,
            float maxY)
        {
            Decal(zone, name, material, minX, maxX, minY, maxY, -AsphaltThickness, 0.0f);
        }

        // A painted line along y at x (lane marking white unless given), standing MarkingTop proud.
        private static void LineY(ZoneLayout zone, string name, float x, float minY, float maxY,
            string material = "lane_marking")
        {
            Decal(zone, name, material, x - MarkingWidth * 0.5f, x + MarkingWidth * 0.5f, minY, maxY,
                -AsphaltThickness, MarkingTop);
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
