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

        public string Text(double wallNow) => wallNow - setAt <= holdS ? text : "";
    }
}
