using UnityEngine;

namespace MotionPlanningSim.Environment
{
    // Interior layouts of ConstructionSiteV2 (spec Section 4, "Interior layouts", added 2026-10-17):
    // the building fit-out and the delivery-yard store. ROS map frame, metres.
    //
    // The kit has one blockwork wall, 0.2 m thick, so the partitions are 0.2 m (the spec drew
    // 0.15 m). Their lines are set so the clear widths stay as the spec gives them: corridor C1 and
    // hallway H 2.0 m, store S 2.85 m; the door clear widths come from the kit frames. The yard
    // store's hoarding panels are 0.17 m deep (the spec drew 0.4 m rows); the rows are spaced for
    // 2.2 m clear aisles (aisle 1 y -11.085..-13.285, aisle 2 y -13.455..-15.655, which holds the
    // spec's mud patch), and the end walls stand in the spec's x 7.0..7.4 bands.
    public static partial class SiteV2Layout
    {
        // Building fit-out: corridor C1 from door A north to hallway H; rooms R1, R2 and store S south
        // of the hallway, R3, R4 and R5 north of it; east bay E between store S and the east wall,
        // reached from door B and from the hallway's east end.
        public const float InnerWestX = BuildingMinX + WallThickness;   // -2.8
        public const float InnerSouthY = BuildingMinY + WallThickness;  // 5.7
        public const float InnerNorthY = BuildingMaxY - WallThickness;  // 16.8
        public const float C1WestX = 4.15f;            // C1 clear x 4.25..6.25 (2.0 m), centred on door A
        public const float C1EastX = 6.35f;
        // South rooms' north partition (y 11.45..11.65) on the interior columns' north faces; north
        // partition y 13.65..13.85: hallway clear y 11.65..13.65 (2.0 m).
        public const float SouthPartitionY = 11.55f;
        public const float NorthPartitionY = 13.75f;
        public const float StoreWestX = 12.05f;        // store S clear x 12.15..15.0 (2.85 m)
        public const float StoreEastX = 15.1f;         // east bay E clear from x 15.2
        public const float FitOutEastX = CoreMinX;     // hallway partitions end at the core's west face
        private static readonly float[] NorthSeparatorsX = { 5.1f, 10.6f };
        // Door centres along their partitions.
        public const float DoorR1Y = 8.65f;            // clear y 8.0..9.3, in C1's west partition
        public const float DoorR2X = 10.05f;           // clear x 9.4..10.7
        public const float DoorSY = 9.025f;            // clear y 8.5..9.55, in store S's east partition
        public const float DoorR3X = 1.65f;            // clear x 1.0..2.3
        public const float DoorR4X = 7.825f;           // clear x 7.3..8.35
        public const float DoorR5X = 12.65f;           // clear x 12.0..13.3
        // Door frames stand 0.01 m proud of the 0.2 m partitions; clear areas start at the frame faces.
        private const float FrameHalfDepth = 0.11f;

        // Delivery-yard store: three rows of hoarding panels and two end walls make an S-shaped
        // route from the open area (entry gap at the west end of row 1) along aisle 1, through the
        // U-turn gap at the east end of row 2, into aisle 2, which ends at the west end wall.
        private const float StorePanel = 2.4f;          // hoarding_panel_2p4 length
        private const float StorePanelDepth = 0.17f;
        public const float StoreAisle = 2.2f;
        public const float StoreRow1Y = -11.0f;
        public const float StoreRow2Y = StoreRow1Y - StoreAisle - StorePanelDepth;   // -13.37
        public const float StoreRow3Y = StoreRow2Y - StoreAisle - StorePanelDepth;   // -15.74
        // End walls' inner faces; every run is a whole number of panels, so none overlap.
        public const float StoreInnerX = 7.2f;
        // Rows 1 and 2 are five panels (12.0 m), leaving the 2.4 m entry and U-turn gaps; row 3 is six.
        private const int ShortRowPanels = 5;

        // Partitions, door frames and their features (the fit-out of the building shell).
        public static ZoneLayout FitOut()
        {
            var zone = new ZoneLayout { Zone = "FitOut" };
            var h = WallThickness * 0.5f;
            var r1 = FrameSpan(DoorR1Y, Door130FrameWidth);
            var r2 = FrameSpan(DoorR2X, Door130FrameWidth);
            var s = FrameSpan(DoorSY, Door105FrameWidth);
            var r3 = FrameSpan(DoorR3X, Door130FrameWidth);
            var r4 = FrameSpan(DoorR4X, Door105FrameWidth);
            var r5 = FrameSpan(DoorR5X, Door130FrameWidth);
            var hallSouth = SouthPartitionY + h;
            var hallNorth = NorthPartitionY - h;

            // Corridor C1, from the south wall to the hallway; R1's door in its west partition.
            WallY(zone, "C1West", C1WestX, new[] { (InnerSouthY, r1.from), (r1.to, hallSouth) });
            WallY(zone, "C1East", C1EastX, new[] { (InnerSouthY, hallSouth) });
            // The south rooms' north partition, open for C1; R2's door; store S between R2 and the
            // east bay, its door in its east partition.
            WallX(zone, "SouthPartition", SouthPartitionY, new[]
            {
                (InnerWestX, C1WestX - h), (C1EastX + h, r2.from), (r2.to, FitOutEastX),
            });
            WallY(zone, "StoreWest", StoreWestX, new[] { (InnerSouthY, SouthPartitionY - h) });
            WallY(zone, "StoreEast", StoreEastX, new[] { (InnerSouthY, s.from), (s.to, hallSouth) });
            // The north partition with the doors of R3, R4 and R5, and the separators between them.
            WallX(zone, "NorthPartition", NorthPartitionY, new[]
            {
                (InnerWestX, r3.from), (r3.to, r4.from), (r4.to, r5.from), (r5.to, FitOutEastX),
            });
            for (var i = 0; i < NorthSeparatorsX.Length; i++)
                WallY(zone, $"NorthSeparator{i + 1}", NorthSeparatorsX[i], new[] { (NorthPartitionY + h, InnerNorthY) });

            // Doors, with straight approaches as deep as the room or hallway they open into (floored
            // to 0.1 m; R4 stops short of the north column inside it). WallYaw puts outside on the
            // corridor, hallway or bay side.
            Door(zone, "DoorR1", "door_frame_1p30", 1.30f, C1WestX, DoorR1Y, alongY: true, HalfPi, 1.9f, 6.8f);
            Door(zone, "DoorR2", "door_frame_1p30", 1.30f, DoorR2X, SouthPartitionY, alongY: false, Mathf.PI, 1.9f, 5.7f);
            Door(zone, "DoorS", "door_frame_1p05", 1.05f, StoreEastX, DoorSY, alongY: true, HalfPi, 3.5f, 2.8f);
            Door(zone, "DoorR3", "door_frame_1p30", 1.30f, DoorR3X, NorthPartitionY, alongY: false, 0.0f, 1.9f, 2.9f);
            Door(zone, "DoorR4", "door_frame_1p05", 1.05f, DoorR4X, NorthPartitionY, alongY: false, 0.0f, 1.9f, 2.7f);
            Door(zone, "DoorR5", "door_frame_1p30", 1.30f, DoorR5X, NorthPartitionY, alongY: false, 0.0f, 1.9f, 2.9f);

            var eastBayEast = BuildingMaxX - ColumnSize;    // the east columns stand 0.4 m in
            zone.Features.ClearRects.Add(Rect("CorridorC1", C1WestX + FrameHalfDepth, C1EastX - h,
                InnerSouthY + 0.01f, hallSouth));
            zone.Features.ClearRects.Add(Rect("HallwayH", InnerWestX, FitOutEastX, hallSouth + 0.01f,
                hallNorth - 0.01f));
            zone.Features.ClearRects.Add(Rect("EastBay", StoreEastX + FrameHalfDepth, eastBayEast,
                InnerSouthY + 0.01f, CoreMinY));
            return zone;
        }

        // The serpentine store in the south of the delivery yard (hoarding panels, 2.4 m high).
        public static ZoneLayout YardStore()
        {
            var zone = new ZoneLayout { Zone = "YardStore" };
            var gapEdge = ShortRowPanels * StorePanel - StoreInnerX;          // 4.8
            var halfDepth = StorePanelDepth * 0.5f;
            var endWallX = StoreInnerX + halfDepth;
            var top = StoreRow1Y + halfDepth;                                  // row 1's north face
            var endWallLength = 2.0f * StorePanel;                             // to inside row 3
            Run(zone, "Row1", "hoarding_panel_2p4", StorePanel, new Vector2(-gapEdge, StoreRow1Y),
                new Vector2(StoreInnerX, StoreRow1Y));
            Run(zone, "Row2", "hoarding_panel_2p4", StorePanel, new Vector2(-StoreInnerX, StoreRow2Y),
                new Vector2(gapEdge, StoreRow2Y));
            Run(zone, "Row3", "hoarding_panel_2p4", StorePanel, new Vector2(-StoreInnerX, StoreRow3Y),
                new Vector2(StoreInnerX, StoreRow3Y));
            Run(zone, "EndWest", "hoarding_panel_2p4", StorePanel, new Vector2(-endWallX, top),
                new Vector2(-endWallX, top - endWallLength));
            Run(zone, "EndEast", "hoarding_panel_2p4", StorePanel, new Vector2(endWallX, top),
                new Vector2(endWallX, top - endWallLength));

            var aisle1North = StoreRow1Y - halfDepth;
            var aisle1South = StoreRow2Y + halfDepth;
            var aisle2North = StoreRow2Y - halfDepth;
            var aisle2South = StoreRow3Y + halfDepth;
            zone.Features.ClearRects.Add(Rect("StoreEntry", -StoreInnerX, -gapEdge, aisle1North, YardOpenMinY));
            zone.Features.ClearRects.Add(Rect("StoreAisle1", -StoreInnerX, StoreInnerX, aisle1South, aisle1North));
            zone.Features.ClearRects.Add(Rect("StoreUTurn", gapEdge, StoreInnerX, aisle2North, aisle1South));
            zone.Features.ClearRects.Add(Rect("StoreAisle2", -StoreInnerX, StoreInnerX, aisle2South, aisle2North));
            return zone;
        }

        private static (float from, float to) FrameSpan(float centre, float frameWidth)
        {
            return (centre - frameWidth * 0.5f, centre + frameWidth * 0.5f);
        }

        // A kit door frame in a partition along x (alongY false) or y, and its DoorFeature.
        private static void Door(ZoneLayout zone, string name, string pieceId, float clearWidth, float x, float y,
            bool alongY, float wallYaw, float approachOut, float approachIn)
        {
            Piece(zone, $"{name}Frame", pieceId, x, y, 0.0, alongY ? HalfPi : 0.0f);
            zone.Features.Doors.Add(new DoorFeature
            {
                Name = name, CentreX = x, CentreY = y, WallYaw = wallYaw, ClearWidth = clearWidth,
                ApproachOut = approachOut, ApproachIn = approachIn
            });
        }
    }
}
