using UnityEngine;

namespace MotionPlanningSim.Environment
{
    /// <summary>
    /// Physical constants shared by the construction experiment and its tests.
    /// ROS remains responsible for command generation and limit enforcement.
    /// </summary>
    public static class MobileManipulatorPhysicalContract
    {
        public const float UrdfRobotMassKg = 107.0f;
        public const float FrameOnlyArticulationMassKg = 0.001f;
        public const float FrameOnlyInertiaKgMetresSquared = 0.000001f;

        public const float WheelRadiusMetres = 0.14f;
        public const float WheelbaseMetres = 0.60f;
        public const float WheelTrackMetres = 0.64f;

        public const float BareFootprintLengthMetres = 0.88f;
        public const float BareFootprintWidthMetres = 0.73f;
        public const float BareFootprintAllowanceMetres = 0.02f;

        public const float PayloadWidthMetres = 1.20f;
        public const float PayloadHeightMetres = 1.20f;
        public const float PayloadThicknessMetres = 0.04f;
        public const float PayloadMountStandoffMetres = 0.015f;
        public const float PayloadMassKg = 3.0f;

        public static Vector3 PayloadSizeUnity => new Vector3(
            PayloadWidthMetres,
            PayloadThicknessMetres,
            PayloadHeightMetres);

        public static Vector3 PayloadCentreOfMassAtTool => new Vector3(
            0.0f,
            (PayloadThicknessMetres * 0.5f) + PayloadMountStandoffMetres,
            0.0f);

        public static Vector3 BoxInertiaDiagonal(float massKg, Vector3 sizeMetres)
        {
            var xSquared = sizeMetres.x * sizeMetres.x;
            var ySquared = sizeMetres.y * sizeMetres.y;
            var zSquared = sizeMetres.z * sizeMetres.z;
            return new Vector3(
                massKg * (ySquared + zSquared) / 12.0f,
                massKg * (xSquared + zSquared) / 12.0f,
                massKg * (xSquared + ySquared) / 12.0f);
        }

        public static Vector3 ReferencePayloadInertia => BoxInertiaDiagonal(
            PayloadMassKg,
            PayloadSizeUnity);
    }
}
