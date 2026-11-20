namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// The HUD's message line (rejections, errors, "not connected"). It shows in every recorded frame, so it expires
    /// after a hold time and is cleared by a new session, rather than contradicting the marker for the rest of a take
    /// (final review I1).
    /// </summary>
    public sealed class StatusMessage
    {
        readonly double holdS;
        string text = "";
        double setAt = double.NegativeInfinity;

        public StatusMessage(double holdS) => this.holdS = holdS;

        public void Set(string message, double wallNow)
        {
            text = message ?? "";
            setAt = wallNow;
        }

        public void Clear() => text = "";

        /// <summary>
        /// The warnings in <paramref name="now"/> not in <paramref name="before"/>: each reaches the console once. Numbers
        /// are ignored in the comparison, so a warning whose figures change ("stale: no state for 1.2 s") is one warning
        /// (final review I1).
        /// </summary>
        public static System.Collections.Generic.IEnumerable<string> Added(
            System.Collections.Generic.IReadOnlyCollection<string> before, System.Collections.Generic.IReadOnlyCollection<string> now)
        {
            var known = new System.Collections.Generic.HashSet<string>();
            foreach (var w in before) known.Add(Key(w));
            foreach (var w in now)
                if (known.Add(Key(w))) yield return w;
        }

        static string Key(string warning) => System.Text.RegularExpressions.Regex.Replace(warning, "[0-9]+(\\.[0-9]+)?", "#");

        public string Text(double wallNow) => wallNow - setAt <= holdS ? text : "";
    }
}
