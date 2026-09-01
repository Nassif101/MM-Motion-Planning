using System;
using UnityEngine;

namespace MotionPlanningSim.Environment
{
    public readonly struct Nav2GridSpec
    {
        public Nav2GridSpec(
            float minRosX,
            float minRosY,
            float maxRosX,
            float maxRosY,
            float resolution)
        {
            if (!float.IsFinite(minRosX) || !float.IsFinite(minRosY) ||
                !float.IsFinite(maxRosX) || !float.IsFinite(maxRosY) ||
                !float.IsFinite(resolution) || resolution <= 0.0f ||
                maxRosX <= minRosX || maxRosY <= minRosY)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(resolution),
                    "Nav2 grid bounds and resolution must be finite and positive.");
            }

            MinRosX = minRosX;
            MinRosY = minRosY;
            MaxRosX = maxRosX;
            MaxRosY = maxRosY;
            Resolution = resolution;
            Width = CalculateCellCount(maxRosX - minRosX, resolution);
            Height = CalculateCellCount(maxRosY - minRosY, resolution);
        }

        public float MinRosX { get; }
        public float MinRosY { get; }
        public float MaxRosX { get; }
        public float MaxRosY { get; }
        public float Resolution { get; }
        public int Width { get; }
        public int Height { get; }

        private static int CalculateCellCount(float extent, float resolution)
        {
            var exact = extent / resolution;
            var rounded = Mathf.RoundToInt(exact);
            if (rounded <= 0 || Mathf.Abs(exact - rounded) > 1e-4f)
            {
                throw new ArgumentException(
                    "Nav2 map extents must be an integer multiple of the resolution.");
            }

            return rounded;
        }
    }

    public static class Nav2MapGeometry
    {
        private const float CellBoundaryTolerance = 1e-4f;

        // Unity world is right/up/forward. ROS map uses FLU: +X forward, +Y left.
        public static Vector2 UnityWorldToRosMap(Vector3 unityPosition)
        {
            return new Vector2(unityPosition.z, -unityPosition.x);
        }

        public static Vector3 RosMapToUnityWorld(Vector2 rosPosition, float unityHeight)
        {
            return new Vector3(-rosPosition.y, unityHeight, rosPosition.x);
        }

        public static Vector2 CellCentreRos(Nav2GridSpec grid, int cellX, int cellY)
        {
            ValidateCell(grid, cellX, cellY);
            return new Vector2(
                grid.MinRosX + ((cellX + 0.5f) * grid.Resolution),
                grid.MinRosY + ((cellY + 0.5f) * grid.Resolution));
        }

        public static bool TryGetOverlappingCells(
            Bounds unityBounds,
            Nav2GridSpec grid,
            float minimumUnityHeight,
            float maximumUnityHeight,
            out int minimumCellX,
            out int minimumCellY,
            out int maximumCellX,
            out int maximumCellY)
        {
            minimumCellX = minimumCellY = maximumCellX = maximumCellY = 0;
            if (unityBounds.max.y <= minimumUnityHeight ||
                unityBounds.min.y >= maximumUnityHeight)
            {
                return false;
            }

            var minimumRosX = unityBounds.min.z;
            var maximumRosX = unityBounds.max.z;
            var minimumRosY = -unityBounds.max.x;
            var maximumRosY = -unityBounds.min.x;
            if (maximumRosX <= grid.MinRosX || minimumRosX >= grid.MaxRosX ||
                maximumRosY <= grid.MinRosY || minimumRosY >= grid.MaxRosY)
            {
                return false;
            }

            minimumCellX = Mathf.Clamp(
                Mathf.FloorToInt(
                    ((minimumRosX - grid.MinRosX) / grid.Resolution) +
                    CellBoundaryTolerance),
                0,
                grid.Width - 1);
            minimumCellY = Mathf.Clamp(
                Mathf.FloorToInt(
                    ((minimumRosY - grid.MinRosY) / grid.Resolution) +
                    CellBoundaryTolerance),
                0,
                grid.Height - 1);
            maximumCellX = Mathf.Clamp(
                Mathf.CeilToInt(
                    ((maximumRosX - grid.MinRosX) / grid.Resolution) -
                    CellBoundaryTolerance) - 1,
                0,
                grid.Width - 1);
            maximumCellY = Mathf.Clamp(
                Mathf.CeilToInt(
                    ((maximumRosY - grid.MinRosY) / grid.Resolution) -
                    CellBoundaryTolerance) - 1,
                0,
                grid.Height - 1);
            return minimumCellX <= maximumCellX && minimumCellY <= maximumCellY;
        }

        public static int CellIndex(Nav2GridSpec grid, int cellX, int cellY)
        {
            ValidateCell(grid, cellX, cellY);
            return (cellY * grid.Width) + cellX;
        }

        private static void ValidateCell(Nav2GridSpec grid, int cellX, int cellY)
        {
            if (cellX < 0 || cellX >= grid.Width || cellY < 0 || cellY >= grid.Height)
                throw new ArgumentOutOfRangeException(nameof(cellX), "Cell is outside the Nav2 grid.");
        }
    }
}
