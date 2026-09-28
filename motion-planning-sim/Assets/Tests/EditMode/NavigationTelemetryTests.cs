using MotionPlanningSim.Visualization;
using NUnit.Framework;

namespace MotionPlanningSim.Tests
{
    public class NavigationTelemetryTests
    {
        private const string Sample =
            "{\"t\":12.4,\"scenario\":\"open_space_nav\",\"footprint_profile\":\"home\"," +
            "\"behavior_tree\":\"replan_if_invalid\",\"planner\":\"Lattice\",\"controller\":\"RPP\"," +
            "\"goal_seq\":3,\"goal\":{\"status\":\"executing\",\"recoveries\":1,\"distance_remaining_m\":2.5," +
            "\"eta_s\":8.0,\"elapsed_s\":4.2,\"pose\":[10.1,0.0]}," +
            "\"path\":{\"length_m\":4.0,\"points\":41,\"age_s\":4.1,\"cross_track_m\":-1,\"cross_track_max_m\":0.03}," +
            "\"monitor\":{\"action\":\"slowdown\",\"polygon\":\"SlowdownZone\"}," +
            "\"lidar\":{\"rate_hz\":10.0,\"gaps_over_0p5s\":0,\"max_gap_s\":0.1}," +
            "\"events\":[{\"t\":8.2,\"text\":\"goal received\"},{\"t\":8.3,\"text\":\"new plan 4.0 m\"}]}";

        [Test]
        public void ParsesTheRosTelemetryMessage()
        {
            var telemetry = NavigationTelemetry.Parse(Sample);

            Assert.That(telemetry.scenario, Is.EqualTo("open_space_nav"));
            Assert.That(telemetry.goal_seq, Is.EqualTo(3));
            Assert.That(telemetry.goal.status, Is.EqualTo("executing"));
            Assert.That(telemetry.goal.recoveries, Is.EqualTo(1));
            Assert.That(telemetry.goal.pose, Is.EqualTo(new[] { 10.1f, 0.0f }));
            Assert.That(telemetry.path.points, Is.EqualTo(41));
            Assert.That(telemetry.path.cross_track_m, Is.EqualTo(-1f), "unavailable values arrive as -1");
            Assert.That(telemetry.monitor.action, Is.EqualTo("slowdown"));
            Assert.That(telemetry.events.Length, Is.EqualTo(2));
            Assert.That(telemetry.events[1].text, Is.EqualTo("new plan 4.0 m"));
        }

        [Test]
        public void RejectsMalformedMessagesWithoutReplacingTheLastGoodOne()
        {
            var feed = new TelemetryFeed();
            feed.Accept(Sample, 100.0);
            feed.Accept("{not json", 100.5);
            feed.Accept("", 100.6);

            Assert.That(feed.Received, Is.EqualTo(1));
            Assert.That(feed.Rejected, Is.EqualTo(2));
            Assert.That(feed.Latest.scenario, Is.EqualTo("open_space_nav"));
        }

        [Test]
        public void ReportsStalenessAfterOneAndAHalfSeconds()
        {
            var feed = new TelemetryFeed();
            Assert.That(feed.IsStale(0.0), Is.True, "no data yet");
            feed.Accept(Sample, 10.0);
            Assert.That(feed.IsStale(11.4), Is.False);
            Assert.That(feed.IsStale(11.6), Is.True);
            Assert.That(feed.Age(11.0), Is.EqualTo(1.0).Within(1e-9));
        }
    }
}
