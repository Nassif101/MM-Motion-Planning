using System;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// The routing map the minimap draws and picks on (design 6.5): a Nav2 map YAML, its trinary PGM and the
    /// exporter's metadata sidecar. Pixel coordinates are continuous from the image's top-left corner, so row 0 is the
    /// map's y maximum. <see cref="MatchesFingerprint"/> compares the files with the digests the backend reports in
    /// its Hello <c>routing</c>, so goals are picked on the map the planner uses.
    /// </summary>
    public sealed class MapFrame
    {
        public double Resolution { get; private set; }
        public double OriginX { get; private set; }
        public double OriginY { get; private set; }
        public int Width { get; private set; }
        public int Height { get; private set; }

        /// <summary>The PGM's values, row-major from the top row (0 occupied, 254 free, 205 unknown).</summary>
        public byte[] Pixels { get; private set; }

        string yamlName, imageName, metadataName;
        string yamlSha, imageSha, metadataSha;

        public static MapFrame Load(string yamlPath)
        {
            var m = new MapFrame();
            var dir = Path.GetDirectoryName(Path.GetFullPath(yamlPath));
            string image = null;
            foreach (var raw in File.ReadAllLines(yamlPath))
            {
                var line = raw.Split('#')[0].Trim();
                var colon = line.IndexOf(':');
                if (colon < 0) continue;
                var key = line.Substring(0, colon).Trim();
                var value = line.Substring(colon + 1).Trim();
                if (key == "image") image = value.Trim('"', '\'');
                else if (key == "resolution") m.Resolution = Number(value);
                else if (key == "origin")
                {
                    var parts = value.Trim('[', ']').Split(',');
                    m.OriginX = Number(parts[0]);
                    m.OriginY = Number(parts[1]);
                }
            }
            if (image == null || m.Resolution <= 0) throw new InvalidDataException(yamlPath + ": no image or resolution");
            var imagePath = Path.Combine(dir, image);
            var metadataPath = Path.Combine(dir, Path.GetFileNameWithoutExtension(yamlPath) + ".metadata.json");
            m.ReadPgm(File.ReadAllBytes(imagePath));
            m.yamlName = Path.GetFileName(yamlPath);
            m.imageName = image;
            m.metadataName = Path.GetFileName(metadataPath);
            m.yamlSha = Sha256(File.ReadAllBytes(yamlPath));
            m.imageSha = Sha256(File.ReadAllBytes(imagePath));
            m.metadataSha = File.Exists(metadataPath) ? Sha256(File.ReadAllBytes(metadataPath)) : null;
            return m;
        }

        public (double x, double y) PixelToWorld(double px, double py) =>
            (OriginX + px * Resolution, OriginY + (Height - py) * Resolution);

        public (double px, double py) WorldToPixel(double x, double y) =>
            ((x - OriginX) / Resolution, Height - (y - OriginY) / Resolution);

        /// <summary>
        /// Whether the bundled files are the backend's map; otherwise <paramref name="reason"/> names the first file
        /// that differs (or says the server reported no routing inputs).
        /// </summary>
        public bool MatchesFingerprint(JObject routing, out string reason)
        {
            reason = null;
            if (routing == null || routing["map_image_sha256"] == null)
            {
                reason = "the server reports no routing map";
                return false;
            }
            foreach (var (key, name, sha) in new[]
                     {
                         ("map_yaml_sha256", yamlName, yamlSha), ("map_image_sha256", imageName, imageSha),
                         ("map_metadata_sha256", metadataName, metadataSha),
                     })
            {
                var theirs = routing[key]?.Type == JTokenType.String ? (string)routing[key] : null;
                if (!string.Equals(theirs, sha, StringComparison.OrdinalIgnoreCase))
                {
                    reason = $"{name} differs from the server's map";
                    return false;
                }
            }
            return true;
        }

        void ReadPgm(byte[] data)
        {
            var at = 0;
            if (Token(data, ref at) != "P5") throw new InvalidDataException("the map image is not a binary PGM (P5)");
            Width = int.Parse(Token(data, ref at), CultureInfo.InvariantCulture);
            Height = int.Parse(Token(data, ref at), CultureInfo.InvariantCulture);
            var max = int.Parse(Token(data, ref at), CultureInfo.InvariantCulture);
            if (max > 255) throw new InvalidDataException("16-bit PGM maps are not supported");
            ++at;  // the single whitespace before the raster
            if (data.Length - at < Width * Height) throw new InvalidDataException("the map image is truncated");
            Pixels = new byte[Width * Height];
            Array.Copy(data, at, Pixels, 0, Pixels.Length);
        }

        static string Token(byte[] data, ref int at)
        {
            for (;;)
            {
                while (at < data.Length && char.IsWhiteSpace((char)data[at])) ++at;
                if (at < data.Length && data[at] == '#')
                {
                    while (at < data.Length && data[at] != '\n') ++at;
                    continue;
                }
                break;
            }
            var start = at;
            while (at < data.Length && !char.IsWhiteSpace((char)data[at])) ++at;
            return Encoding.ASCII.GetString(data, start, at - start);
        }

        static double Number(string s) => double.Parse(s.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture);

        static string Sha256(byte[] data)
        {
            using var sha = SHA256.Create();
            var sb = new StringBuilder(64);
            foreach (var b in sha.ComputeHash(data)) sb.Append(b.ToString("x2"));
            return sb.ToString();
        }
    }

    /// <summary>The minimap's press-drag-release gesture as a goal pose (design 6.5).</summary>
    public static class MinimapGesture
    {
        /// <summary>
        /// The press sets the position; a drag of at least <paramref name="minDragM"/> sets the heading toward the
        /// release, and a shorter one keeps the heading from the robot to the point (0 on the robot). Presses off the
        /// map are not clamped: the backend rejects them with its reason.
        /// </summary>
        public static (double x, double y, double yaw) Target((double x, double y) press, (double x, double y) release,
            (double x, double y) robot, double minDragM = 0.3)
        {
            double dx = release.x - press.x, dy = release.y - press.y;
            if (dx * dx + dy * dy >= minDragM * minDragM) return (press.x, press.y, Math.Atan2(dy, dx));
            dx = press.x - robot.x;
            dy = press.y - robot.y;
            return (press.x, press.y, dx == 0 && dy == 0 ? 0.0 : Math.Atan2(dy, dx));
        }
    }
}
