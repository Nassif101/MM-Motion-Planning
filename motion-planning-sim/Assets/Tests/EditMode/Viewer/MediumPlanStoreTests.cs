using System;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>4B-1 Task 1: F2's plan keyed to the active goal and epoch, data availability, the travelled path.</summary>
    public sealed class MediumPlanStoreTests
    {
        static Frame Hello(bool generated = true, bool medium = true, bool lidar = true, long epoch = 1) => new Frame
        {
            Type = FrameType.Hello,
            Header = new JObject
            {
                ["schema"] = 3, ["scenario"] = "v2_open_yard_nav", ["seed"] = 1, ["epoch"] = epoch, ["medium"] = medium,
                ["config"] = new JObject { ["generated_routes"] = generated, ["medium"] = medium, ["lidar"] = lidar },
            },
            Blocks = Array.Empty<float[]>(),
        };

        static Frame Goal(long epoch, long goalId, string state) => new Frame
        {
            Type = FrameType.Goal,
            Header = JObject.Parse($"{{\"event\":\"goal_{(state == "planning" ? "accepted" : state)}\",\"epoch\":{epoch}," +
                                   $"\"goal\":{{\"goal_id\":{goalId},\"state\":\"{state}\",\"target\":[1,2,0]}}}}"),
            Blocks = Array.Empty<float[]>(),
        };

        static Frame Plans(double t, long planId, long? epoch = null, long? goalId = null) => new Frame
        {
            Type = FrameType.Plans,
            Header = new JObject
            {
                ["t"] = t, ["plan_id"] = planId, ["route_id"] = 8, ["status"] = "ok", ["homotopy"] = "left", ["capped"] = false,
                ["seed"] = "previous", ["full_multistart"] = false, ["waiting"] = false, ["homotopy_switched"] = false,
                ["seeds"] = new JArray { new JArray("previous", 1.5, 1, 2.0, 0), new JArray("vertical_carry", 3.0, 0, 1.0, 1) },
                ["epoch"] = epoch, ["goal_id"] = goalId,
            },
            Blocks = new[] { new float[] { 0f, 0f, 0f, 0f, 1f, 2f, 3f, 4f, 5f, 0.1f, 0f, 0f, 0f, 1f, 2f, 3f, 4f, 5f } },
        };

        static Frame State(double t) => new Frame
        {
            Type = FrameType.State,
            Header = new JObject { ["t"] = t },
            Blocks = new[] { new float[9], new float[9], new float[8] },
        };

        [Test]
        public void PlansFollowTheActiveGoalAndEpoch()
        {
            var s = new ViewerState();
            s.Apply(Hello(epoch: 2), 0.0);
            s.Apply(Goal(2, 4, "planning"), 0.0);
            s.Apply(Plans(1.0, 11, 2, 4), 0.0);
            Assert.IsNotNull(s.Plan);
            Assert.AreEqual(11, s.Plan.PlanId);
            Assert.AreEqual(2, s.Plan.Samples.Count);
            Assert.AreEqual(5.0, s.Plan.Samples[1][8], 1e-6);
            Assert.AreEqual(("vertical_carry", 3.0, false, 1.0, true), s.Plan.Seeds[1]);
            s.Apply(Plans(1.4, 12, 1, 4), 0.0);  // an older epoch
            Assert.AreEqual(11, s.Plan.PlanId);
            s.Apply(Plans(1.4, 13, 2, 9), 0.0);  // another goal
            Assert.AreEqual(11, s.Plan.PlanId);
            s.Apply(Plans(1.8, 14, 2, 4), 0.0);  // a newer plan of the active goal replaces it
            Assert.AreEqual(14, s.Plan.PlanId);
            s.Apply(Goal(2, 6, "planning"), 0.0);  // a replacement becomes the active goal: the old plan goes at once
            Assert.IsNull(s.Plan);
            s.Apply(Plans(1.9, 16, 2, 6), 0.0);
            Assert.AreEqual(16, s.Plan.PlanId);
            s.Apply(Goal(2, 6, "reached"), 0.0);  // the goal ends
            Assert.IsNull(s.Plan);

            s.Apply(Goal(2, 5, "planning"), 0.0);
            s.Apply(Plans(2.0, 15, 2, 5), 0.0);
            s.Apply(Hello(epoch: 3), 0.0);  // a new session
            Assert.IsNull(s.Plan);

            var recorded = new ViewerState();  // a recorded-route session has no goal ids: every plan is kept
            recorded.Apply(Hello(generated: false), 0.0);
            recorded.Apply(Plans(1.0, 21), 0.0);
            Assert.AreEqual(21, recorded.Plan.PlanId);
        }

        [Test]
        public void StalePlanAfterOneSecond()
        {
            var s = new ViewerState();
            s.Apply(Hello(generated: false), 0.0);
            s.Apply(Plans(5.0, 1), 0.0);
            Assert.IsFalse(s.PlanIsStale(5.9));
            Assert.IsTrue(s.PlanIsStale(6.1));
        }

        [Test]
        public void AvailabilityFollowsTheSession()
        {
            var s = new ViewerState();
            Assert.IsFalse(s.HasMediumPlans);
            Assert.IsFalse(s.HasLidar);
            s.Apply(Hello(medium: true, lidar: true), 0.0);
            Assert.IsTrue(s.HasMediumPlans);
            Assert.IsTrue(s.HasLidar);
            s.Apply(Hello(medium: false, lidar: false), 0.0);
            Assert.IsFalse(s.HasMediumPlans);
            Assert.IsFalse(s.HasLidar);
        }

        [Test]
        public void TravelledPathIsBoundedAndSpaced()
        {
            var p = new PathHistory();
            p.Add(0.0, 0.0);
            p.Add(0.01, 0.0);  // under 0.02 m: merged
            Assert.AreEqual(1, p.Points.Count);
            for (var k = 1; k <= 6000; ++k) p.Add(0.05 * k, 0.0);
            Assert.AreEqual(5000, p.Points.Count);
            Assert.AreEqual(300.0, p.Points[p.Points.Count - 1].x, 1e-9);  // the newest kept
            p.Clear();
            Assert.AreEqual(0, p.Points.Count);
        }
    }
}
