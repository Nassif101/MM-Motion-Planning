using UnityEngine;

namespace MotionPlanningSim.Environment
{
    public static class PayloadClearance
    {
        public static float RequiredStraightPassage(float projectedWidth, float sideMargin)
        {
            return projectedWidth + (2.0f * sideMargin);
        }

        public static float RectangularSweptDiameter(float width, float depth)
        {
            return Mathf.Sqrt((width * width) + (depth * depth));
        }

        public static float RequiredTurningPocket(float width, float depth, float radialMargin)
        {
            return RectangularSweptDiameter(width, depth) + (2.0f * radialMargin);
        }

        public static bool FitsStraightPassage(
            float projectedWidth,
            float passageWidth,
            float sideMargin)
        {
            return passageWidth + Mathf.Epsilon >= RequiredStraightPassage(projectedWidth, sideMargin);
        }
    }
}
