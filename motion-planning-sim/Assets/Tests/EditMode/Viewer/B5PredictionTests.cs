using System;
using System.Linq;
using System.Numerics;
using MotionPlanningSim.Viewer.Core;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace MotionPlanningSim.Tests.Viewer
{
    /// <summary>
    /// 4B-2 Task 2: B5's prediction (the Prediction frame), F2's display samples, and the fallbacks for a server
    /// without the 4B-2 capabilities.
    /// </summary>
    public sealed class B5PredictionTests
    {
        static Frame Hello(params string[] capabilities) => new Frame
        {
            Type = FrameType.Hello,
            Header = new JObject
            {
                ["schema"] = 3, ["scenario"] = "v2_door_105_nav", ["seed"] = 1, ["epoch"] = 1,
                ["config"] = new JObject { ["generated_routes"] = false, ["medium"] = true, ["lidar"] = true },
                ["capabilities"] = new JArray(capabilities.Cast<object>().ToArray()),
            },
            Blocks = Array.Empty<float[]>(),
        };

        // 21 stages 0.1 s apart: the base drives along x at 0.5 m/s, joint 0 turns at 0.2 rad/s
        static Frame Prediction(double t, string status = "ok", int stages = 20)
        {
            var xs = new float[(stages + 1) * 9];
            for (var k = 0; k <= stages; ++k)
            {
                xs[9 * k] = (float)(0.05 * k);
                xs[9 * k + 3] = (float)(0.02 * k);
            }
            return new Frame
            {
                Type = FrameType.Prediction,
                Header = new JObject
                {
                    ["t"] = t, ["status"] = status, ["stage_dt"] = 0.1, ["stages"] = stages, ["plan_id"] = 7,
                    ["layout"] = "states x y yaw q0..q5 per stage",
                },
                Blocks = new[] { xs },
            };
        }

        static Frame State(double t) => new Frame
        {
            Type = FrameType.State,
            Header = new JObject { ["t"] = t },
            Blocks = new[] { new float[9], new float[9], new float[8] },
        };

        // F2's plan for an arm turn with the base at rest: block 0 has one sample (F2 samples base motion only),
        // block 1 the display samples every 0.1 s over 1 s, joint 0 turning from 0 to pi/2
        static Frame ArmTurnPlans(bool withDisplay)
        {
            var display = new float[11 * 10];
            for (var k = 0; k <= 10; ++k)
            {
                display[10 * k] = (float)(0.1 * k);
                display[10 * k + 4] = (float)(Math.PI / 2 * k / 10);
            }
            return new Frame
            {
                Type = FrameType.Plans,
                Header = new JObject { ["t"] = 1.0, ["plan_id"] = 3, ["status"] = "ok", ["seed"] = "previous" },
                Blocks = withDisplay ? new[] { new float[9], display } : new[] { new float[9] },
            };
        }

        [Test]
        public void PredictionDecodesAndClearsWithTheSession()
        {
            var s = new ViewerState();
            s.Apply(Hello("b5_prediction"), 0.0);
            Assert.IsTrue(s.HasPrediction);
            s.Apply(Prediction(2.0), 0.0);
            var p = s.Prediction;
            Assert.IsNotNull(p);
            Assert.AreEqual(2.0, p.T, 1e-12);
            Assert.AreEqual("ok", p.Status);
            Assert.AreEqual(0.1, p.StageDt, 1e-12);
            Assert.AreEqual(20, p.Stages);
            Assert.AreEqual(7, p.PlanId);
            Assert.AreEqual(21, p.States.Count);
            Assert.AreEqual(1.0, p.States[20][0], 1e-6);
            Assert.AreEqual(0.4, p.States[20][3], 1e-6);
            // between stages the state is interpolated; past the horizon it holds the last stage
            Assert.AreEqual(0.025, p.StateAt(0.05)[0], 1e-6);
            Assert.AreEqual(1.0, p.StateAt(5.0)[0], 1e-6);
            Assert.AreEqual(4.0, p.HorizonEndT, 1e-12);
            s.Apply(Hello("b5_prediction"), 0.0);  // a new session
            Assert.IsNull(s.Prediction);
        }

        [Test]
        public void HeldPredictionIsMarked()
        {
            // B5 executes its solved plan only on ok and fallback (the replay's rule); goal_reached is not a hold
            foreach (var executed in new[] { "ok", "fallback", "goal_reached" })
                Assert.IsFalse(B5Prediction.FromFrame(Prediction(1.0, executed)).Held, executed);
            foreach (var held in new[] { "holding", "blocked", "braking", "stale_state", "failed", "something_new" })
                Assert.IsTrue(B5Prediction.FromFrame(Prediction(1.0, held)).Held, held);
        }

        [Test]
        public void PredictionIsStaleAfterOneSecond()
        {
            var s = new ViewerState();
            s.Apply(Hello("b5_prediction"), 0.0);
            Assert.IsFalse(s.PredictionIsStale(5.0));  // no prediction: "no data", not stale
            s.Apply(Prediction(2.0), 0.0);
            Assert.IsFalse(s.PredictionIsStale(3.0));
            Assert.IsTrue(s.PredictionIsStale(3.01));
        }

        [Test]
        public void DisplaySamplesDecode()
        {
            var plan = MediumPlan.FromFrame(ArmTurnPlans(withDisplay: true));
            Assert.IsTrue(plan.HasDisplaySamples);
            Assert.AreEqual(11, plan.DisplaySamples.Count);
            Assert.AreEqual(10, plan.DisplaySamples[0].Length);
            Assert.AreEqual(0.5, plan.DisplaySamples[5][0], 1e-6);
            Assert.AreEqual(Math.PI / 4, plan.DisplaySamples[5][4], 1e-6);
            // the state at a plan time, 9 values like a sample
            var mid = plan.StateAt(0.25);
            Assert.AreEqual(9, mid.Length);
            Assert.AreEqual(Math.PI / 2 * 0.25, mid[3], 1e-6);
            Assert.AreEqual(1.0, plan.DisplayHorizonS, 1e-6);
        }

        [Test]
        public void SweepUsesDisplaySamples()
        {
            // the panel's corner at the end of a 1 m arm turned by joint 0, base at rest
            Func<double[], Vector3[]> corners = q => new[] { new Vector3((float)Math.Cos(q[3]), (float)Math.Sin(q[3]), 0f) };
            var withDisplay = PanelSweep.Rings(MediumPlan.FromFrame(ArmTurnPlans(true)).SweepSamples, corners);
            Assert.Greater(withDisplay.Count, 10);
            Assert.AreEqual(0.0, withDisplay[withDisplay.Count - 1][0].X, 1e-5);  // the turn ends at pi/2
            Assert.AreEqual(1.0, withDisplay[withDisplay.Count - 1][0].Y, 1e-5);
            Assert.IsTrue(withDisplay.Any(r => Math.Abs(r[0].X - r[0].Y) < 0.03), "a ring near the turn's middle");
            // 4B-1's picture without them: the one base sample, no turn
            Assert.AreEqual(1, PanelSweep.Rings(MediumPlan.FromFrame(ArmTurnPlans(false)).SweepSamples, corners).Count);
        }

        [Test]
        public void LidarUsesPointTimes()
        {
            (double x, double y, double yaw)? moving(double t) => (t - 1.0, 0.0, 0.0);  // 1 m/s along x from t 1
            var cloud = new LidarCloud();
            // five points, the last two late in the scan: their own times, not i/n of the period
            var times = new[] { 0f, 0.01f, 0.02f, 0.09f, 0.1f };
            Assert.IsTrue(cloud.AddScan(1.0, 1.1, new float[15], new float[5], moving, p => p, times));
            Assert.IsTrue(cloud.Exact);
            var points = cloud.Visible().ToList();
            Assert.AreEqual(0.0, points[0].position.X, 1e-5);
            Assert.AreEqual(0.09, points[3].position.X, 1e-5);
            Assert.AreEqual(0.1, points[4].position.X, 1e-5);
        }

        [Test]
        public void FallbacksWithoutTheCapabilities()
        {
            var s = new ViewerState();
            s.Apply(Hello(), 0.0);  // a 4B-1 server: none of the 4B-2 capabilities
            Assert.IsFalse(s.HasPrediction);
            Assert.IsFalse(s.HasDisplaySamples);
            Assert.IsFalse(s.HasPointTimes);
            Assert.IsNull(s.Prediction);
            Assert.IsFalse(s.PredictionIsStale(10.0));
            s.Apply(State(1.0), 0.0);
            s.Apply(ArmTurnPlans(false), 0.0);
            Assert.IsFalse(s.Plan.HasDisplaySamples);
            Assert.AreEqual(s.Plan.Samples.Count, s.Plan.SweepSamples.Count);  // the sweep uses F2's samples
            Assert.AreEqual(0.0, s.Plan.DisplayHorizonS, 1e-12);
            var cloud = new LidarCloud();
            Assert.IsTrue(cloud.AddScan(1.0, 1.1, new float[15], new float[5], t => (t - 1.0, 0.0, 0.0), p => p));
            Assert.IsFalse(cloud.Exact);  // the 4B-1 approximation
            Assert.AreEqual(0.08, cloud.Visible().Last().position.X, 1e-5);
            // the capabilities of a 4B-2 server
            s.Apply(Hello("b5_prediction", "plan_display_samples", "scan_point_times"), 0.0);
            Assert.IsTrue(s.HasPrediction && s.HasDisplaySamples && s.HasPointTimes);
        }
    }
}
