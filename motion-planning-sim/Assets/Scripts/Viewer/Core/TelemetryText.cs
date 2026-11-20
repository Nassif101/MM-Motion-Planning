using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>The telemetry panel's B5 lines from the newest State header (4B-1 design 7.2); "—" when not received.</summary>
    public static class TelemetryText
    {
        public const string None = "—";

        public static string Of(JToken v) => v == null || v.Type == JTokenType.Null ? None
            : v.Type == JTokenType.Float ? ((double)v).ToString("0.###", System.Globalization.CultureInfo.InvariantCulture)
            : v.Type == JTokenType.String ? (string)v : v.ToString(Formatting.None);

        public static List<string> B5(JObject h)
        {
            var lines = new List<string> { "B5" };
            var fallback = (string)h?["fallback"];
            lines.Add($"  status {Of(h?["status"])}   holding (dwell) {Of(h?["dwell"])}   fallback {(h == null ? None : string.IsNullOrEmpty(fallback) ? "none" : fallback)}");
            var gate = h?["gate"] as JArray;
            lines.Add(gate != null && gate.Count >= 3 ? $"  gate slack {Of(gate[0])} m, violation {Of(gate[1])} m, unsafe {Of(gate[2])}" : "  gate " + None);
            if (h?["guard"] is JArray guard && guard.Count >= 2) lines.Add($"  guard held {Of(guard[0])}, deepest {Of(guard[1])} m");
            lines.Add("  contacts " + Of(h?["contacts"]));  // a count (session.cpp), the wheels on the ground excepted
            return lines;
        }
    }
}
