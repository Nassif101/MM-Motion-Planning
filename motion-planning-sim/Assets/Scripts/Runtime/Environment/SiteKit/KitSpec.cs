using System;
using System.Linq;
using UnityEngine;

namespace MotionPlanningSim.Environment
{
    [Serializable]
    public sealed class KitProxy
    {
        public float[] centre;
        public float[] size;
        public float yaw;
    }

    [Serializable]
    public sealed class KitPiece
    {
        public string id;
        public string family;
        public string mesh;
        public float[] size;
        public bool parametric_x;
        public bool round;
        public KitProxy[] proxies;
        public string material;
        public string physics_material;

        public Vector3 Size => new Vector3(size[0], size[1], size[2]);
    }

    [Serializable]
    public sealed class KitOverheadTop
    {
        public string name;
        public float top_m;
    }

    [Serializable]
    public sealed class KitOverhead
    {
        public float h1_bottom_m;
        public float h3_bottom_m;
        public KitOverheadTop[] tops_m;
    }

    // The kit spec (tools/site_kit/kit_v2.json): piece dimensions, box proxies, materials and the
    // overhead member heights. Piece frame: right-handed, x length, z up, origin at the bottom
    // centre, metres.
    [Serializable]
    public sealed class KitSpec
    {
        public int version;
        public string blender;
        public KitOverhead overhead;
        public KitPiece[] pieces;

        public KitPiece[] Pieces => pieces;
        public KitOverhead Overhead => overhead;

        public static KitSpec Parse(string json)
        {
            var spec = JsonUtility.FromJson<KitSpec>(json);
            if (spec == null || spec.pieces == null || spec.overhead == null)
                throw new ArgumentException("Kit spec JSON has no pieces or overhead section.");
            return spec;
        }

        public KitPiece Get(string id)
        {
            var piece = pieces.FirstOrDefault(p => p.id == id);
            if (piece == null)
                throw new ArgumentException($"Unknown kit piece '{id}'.");
            return piece;
        }
    }
}
