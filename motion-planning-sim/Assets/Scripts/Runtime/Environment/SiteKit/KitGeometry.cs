using System;
using UnityEngine;

namespace MotionPlanningSim.Environment
{
    // Kit piece geometry in the ROS map frame: placed proxy boxes and the imported-mesh check.
    public static class KitGeometry
    {
        // Proxy boxes of a piece placed at (rosX, rosY, rosZ) with yaw about ROS +z. lengthX > 0
        // stretches a parametric_x piece (and its proxies) to that length along x.
        public static RosBox[] ProxyBoxes(KitPiece piece, double rosX, double rosY, double rosZ, double yaw,
            float lengthX = 0.0f)
        {
            var scaleX = 1.0f;
            if (lengthX > 0.0f)
            {
                if (!piece.parametric_x)
                    throw new ArgumentException($"{piece.id} is not parametric in x.", nameof(lengthX));
                scaleX = lengthX / piece.size[0];
            }

            var cos = Math.Cos(yaw);
            var sin = Math.Sin(yaw);
            var boxes = new RosBox[piece.proxies.Length];
            for (var i = 0; i < boxes.Length; i++)
            {
                var p = piece.proxies[i];
                var lx = p.centre[0] * scaleX;
                var ly = p.centre[1];
                var centre = new Vector3(
                    (float)(rosX + cos * lx - sin * ly),
                    (float)(rosY + sin * lx + cos * ly),
                    (float)(rosZ + p.centre[2]));
                var size = new Vector3(p.size[0] * scaleX, p.size[1], p.size[2]);
                boxes[i] = new RosBox(centre, size, (float)(yaw + p.yaw));
            }
            return boxes;
        }

        // The piece-frame size (x length, y width, z height) of an imported mesh from its Unity
        // local bounds, where the model root is turned so piece x lies along Unity z: the
        // importer keeps Blender x along Unity x and puts Blender z up (Unity y), so Unity
        // (x, y, z) is piece (x, z, y).
        public static Vector3 ImportedRosSize(Vector3 unityMeshSize)
        {
            return new Vector3(unityMeshSize.x, unityMeshSize.z, unityMeshSize.y);
        }

        // Null when the imported size fits the spec within tolerance on every axis.
        public static string BoundsMismatch(KitPiece piece, Vector3 importedRosSize, float tolerance = 0.02f)
        {
            var spec = piece.Size;
            var d = importedRosSize - spec;
            if (Mathf.Abs(d.x) <= tolerance && Mathf.Abs(d.y) <= tolerance && Mathf.Abs(d.z) <= tolerance)
                return null;
            return $"{piece.id}: mesh {Fmt(importedRosSize)} vs spec {Fmt(spec)}";
        }

        private static string Fmt(Vector3 v) => $"({v.x:0.###}, {v.y:0.###}, {v.z:0.###})";
    }
}
