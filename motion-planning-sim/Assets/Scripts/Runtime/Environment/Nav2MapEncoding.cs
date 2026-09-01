using System;

namespace MotionPlanningSim.Environment
{
    public static class Nav2MapEncoding
    {
        public const byte OccupiedPixel = 0;
        public const byte FreePixel = 254;

        public static byte[] ToPgmImageRows(bool[] occupiedCells, int width, int height)
        {
            if (width <= 0 || height <= 0)
                throw new ArgumentOutOfRangeException(nameof(width));
            if (occupiedCells == null || occupiedCells.Length != checked(width * height))
                throw new ArgumentException("Occupancy data length must equal width times height.");

            var pixels = new byte[occupiedCells.Length];
            for (var imageRow = 0; imageRow < height; imageRow++)
            {
                var gridY = height - 1 - imageRow;
                for (var cellX = 0; cellX < width; cellX++)
                {
                    var gridIndex = (gridY * width) + cellX;
                    var imageIndex = (imageRow * width) + cellX;
                    pixels[imageIndex] = occupiedCells[gridIndex]
                        ? OccupiedPixel
                        : FreePixel;
                }
            }

            return pixels;
        }
    }
}
