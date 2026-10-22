// The replay bench (Phase 4B-B spec Section 3): reading a B5 recording and replaying it through the controller.
// A closed loop written to a bag the way the node records it must replay to the same cycles.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <mobile_manipulator_interfaces/msg/mpc_cycle.hpp>
#include <mobile_manipulator_interfaces/msg/whole_body_plan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "closed_loop.hpp"
#include "mobile_manipulator_mpc/arm_stream.hpp"
#include "mobile_manipulator_mpc/executed_command.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "mobile_manipulator_mpc/replay_bag.hpp"
#include "test_support.hpp"

using namespace mobile_manipulator_mpc;
using namespace mpc_test;
using Eigen::Vector2d;
using Eigen::Vector3d;
namespace fs = std::filesystem;
namespace mmi = mobile_manipulator_interfaces;

namespace
{
constexpr double kT0 = 1000.0;      // bag time of stamp 0
constexpr double kPeriod = 0.05;    // 20 Hz
constexpr double kStateToCommand = 0.002;

std::array<std::string, 6> joint_names()
{
  std::array<std::string, 6> names;
  std::copy(kArmJoints.begin(), kArmJoints.end(), names.begin());
  return names;
}

rclcpp::Time at(double seconds) { return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9))); }

World beam_world()
{
  return world_of({mmn::Box{"beam", Vector3d(2.0, 0.0, 1.34 + 0.1), Vector3d(0.6, 4.0, 0.2), 0.0}});
}

PostureTargets targets() { return posture_targets(payload(), config().comfort_pose, config().posture_mode, config().tight_poses); }

ReplaySetup setup_for(const World & world)
{
  ReplaySetup s;
  s.world = world;
  s.posture = targets();
  s.limits = limits();
  return s;
}

// A closed loop of `cycles` cycles written to `uri` as the node records it: /plan once, then per cycle /odom
// (before the step), /cmd_vel_smoothed (sent), /cmd_vel (passed through), the streamed trajectory, the plan and
// the cycle. The loop's previous command is built as the replay rebuilds it. Returns the states the loop gave
// the controller.
std::vector<State> record_loop(const std::string & uri, int cycles)
{
  fs::remove_all(uri);
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage;
  storage.uri = uri;
  storage.storage_id = "mcap";
  writer.open(storage);

  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 1.0 / kPeriod, false);
  const World world = beam_world();
  Plant plant{state(0.0, 0.0, 0.0, targets().comfort)};
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x : {0.0, 4.0, 8.0}) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  writer.write(path, "/plan", at(kT0 - 0.01));
  c->start(Route({Vector2d(0, 0), Vector2d(4, 0), Vector2d(8, 0)}), {8.0, 0.0, 0.0}, {}, targets());

  ExecutedCommand executed;
  std::optional<trajectory_msgs::msg::JointTrajectory> trajectory;
  double trajectory_t = 0.0;
  std::vector<State> states;
  const double dt = config().dt;
  for (int i = 0; i < cycles; ++i) {
    const double stamp = i * kPeriod, t_state = kT0 + stamp + 0.001, t_send = t_state + kStateToCommand;
    nav_msgs::msg::Odometry odom;
    odom.twist.twist.linear.x = plant.executed(0);
    odom.twist.twist.angular.z = plant.executed(1);
    writer.write(odom, "/odom", at(t_state - 0.0005));

    Control previous = c->last_command();
    previous.head<2>() = executed.previous(t_send, kExecutedStaleS);
    Eigen::Matrix<double, 6, 1> q, v = Eigen::Matrix<double, 6, 1>::Zero();
    if (trajectory) sample_trajectory(*trajectory, t_state - trajectory_t, q, v);
    previous.tail<6>() = v.cwiseMax(-limits().qd_max).cwiseMin(limits().qd_max);
    c->reset_command(previous);
    states.push_back(plant.x);
    const CycleOutput & out = c->step(stamp, plant.x, true, world, plant.at_rest());

    geometry_msgs::msg::Twist sent;
    sent.linear.x = out.command(0);
    sent.angular.z = out.command(1);
    writer.write(sent, "/cmd_vel_smoothed", at(t_send));
    executed.on_sent(t_send, out.command.head<2>());
    writer.write(sent, "/cmd_vel", at(t_send + 0.0002));
    executed.on_executed(t_send + 0.0002, out.command.head<2>());

    trajectory_msgs::msg::JointTrajectory stream;
    stream.joint_names.assign(kArmJoints.begin(), kArmJoints.end());
    for (size_t k = 0; k < out.plan.x.size(); ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      const Eigen::Matrix<double, 6, 1> qk = out.plan.x[k].tail<6>();
      const Eigen::Matrix<double, 6, 1> vk = k < out.plan.u.size() ? Eigen::Matrix<double, 6, 1>(out.plan.u[k].tail<6>())
                                                                   : Eigen::Matrix<double, 6, 1>::Zero();
      point.positions.assign(qk.data(), qk.data() + 6);
      point.velocities.assign(vk.data(), vk.data() + 6);
      point.time_from_start = rclcpp::Duration::from_seconds(k * dt);
      stream.points.push_back(point);
    }
    writer.write(stream, "/arm_controller/joint_trajectory", at(t_send + 0.0004));
    trajectory = stream;
    trajectory_t = t_send + 0.0004;

    mmi::msg::WholeBodyPlan plan;
    plan.header.stamp = at(stamp);
    plan.joint_names.assign(kArmJoints.begin(), kArmJoints.end());
    for (size_t k = 0; k < out.plan.x.size(); ++k) {
      const State & x = k == 0 ? states.back() : out.plan.x[k];  // stage 0: the controller's input, as the node records it
      geometry_msgs::msg::Pose2D base;
      base.x = x(0);
      base.y = x(1);
      base.theta = x(2);
      plan.base.push_back(base);
      for (int j = 0; j < 6; ++j) plan.positions.push_back(x(3 + j));
    }
    writer.write(plan, "/wholebody_mpc/plan", at(t_send + 0.0006));
    mmi::msg::MpcCycle cycle;
    cycle.header.stamp = at(stamp);
    cycle.status = to_string(out.status);
    cycle.iterations = out.iterations;
    cycle.state_to_command_s = kStateToCommand;
    cycle.posture_choice = to_string(out.posture_choice);
    writer.write(cycle, "/wholebody_mpc/cycle", at(t_send + 0.0008));
    plant.step(out.command, kPeriod);
  }
  writer.close();
  return states;
}

std::string bag_path(const char * name) { return (fs::temp_directory_path() / name).string(); }

// An in-memory recording on a straight 3 m route: goals at the given bag times, cycles at `stamps` (bag time =
// stamp), the robot at the origin, each cycle with status `status_of(i)`.
Recording synthetic(const std::vector<double> & goal_times, const std::vector<double> & stamps,
                    const std::function<std::string(size_t)> & status_of, std::vector<double> bag_times = {})
{
  Recording r;
  if (bag_times.empty()) bag_times = stamps;
  for (double t : goal_times) r.goals.push_back({t, {Vector2d(0, 0), Vector2d(3, 0)}, {3.0, 0.0, 0.0}});
  for (size_t i = 0; i < stamps.size(); ++i) {
    RecordedCycle c;
    c.stamp = stamps[i];
    c.bag_time = bag_times[i];
    c.x = state(0.0, 0.0, 0.0, targets().comfort);
    c.status = status_of(i);
    r.cycles.push_back(c);
  }
  r.period_s = stamps.size() > 1 ? stamps[1] - stamps[0] : kPeriod;
  r.wall_period_s = bag_times.size() > 1 ? bag_times[1] - bag_times[0] : kPeriod;
  return r;
}

std::vector<double> evenly(double from, double step, int count)
{
  std::vector<double> out;
  for (int i = 0; i < count; ++i) out.push_back(from + i * step);
  return out;
}
}  // namespace

TEST(Replay, ReadsCyclesAndGoalsInBagOrder)
{
  const std::string uri = bag_path("mpc_replay_read");
  const std::vector<State> states = record_loop(uri, 12);
  const Recording r = read_recording(uri, joint_names());
  ASSERT_EQ(r.cycles.size(), 12u);
  ASSERT_EQ(r.goals.size(), 1u);
  EXPECT_EQ(r.goals[0].points.size(), 3u);
  EXPECT_NEAR(r.goals[0].goal[0], 8.0, 1e-12);
  EXPECT_NEAR(r.period_s, kPeriod, 1e-9);
  for (size_t i = 0; i < r.cycles.size(); ++i) {
    EXPECT_NEAR(r.cycles[i].stamp, i * kPeriod, 1e-9);
    EXPECT_NEAR(r.cycles[i].bag_time, kT0 + i * kPeriod + 0.001, 1e-9) << "the state time";
    EXPECT_TRUE(r.cycles[i].x.isApprox(states[i], 1e-12)) << "cycle " << i;
    EXPECT_EQ(r.cycles[i].status, "ok");
    EXPECT_FALSE(r.cycles[i].plan.empty());
  }
  fs::remove_all(uri);
}

TEST(Replay, FrozenConfigReproducesItsOwnRecording)
{
  const std::string uri = bag_path("mpc_replay_self");
  record_loop(uri, 60);
  const Recording r = read_recording(uri, joint_names());
  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 1.0 / kPeriod, true);
  ReplayOptions options;
  options.recorded_misses = true;
  const ReplayResult result = replay(*c, r, setup_for(beam_world()), options);
  ASSERT_TRUE(result.fidelity.has_value());
  EXPECT_EQ(result.fidelity->compared, 60);
  EXPECT_DOUBLE_EQ(result.fidelity->status_match, 1.0);
  EXPECT_DOUBLE_EQ(result.fidelity->stage1_match, 1.0);
  EXPECT_DOUBLE_EQ(result.fidelity->iterations_match, 1.0);
  EXPECT_EQ(result.goal_starts, 1);
  // The bench's diagnostics (on in the replayed controller only) leave the cycles unchanged and measure the plan.
  int measured = 0;
  for (size_t i = 0; i < result.status.size(); ++i) {
    ASSERT_TRUE(std::isfinite(result.true_clearance_m[i]));
    if (std::isfinite(result.lin_error_m[i])) {
      EXPECT_GE(result.lin_error_m[i], 0.0);
      ++measured;
    }
    EXPECT_GT(result.progress_mps[i], 0.0);
  }
  EXPECT_GT(measured, 0) << "no cycle had collision rows within the gate's horizon";
  fs::remove_all(uri);
}

TEST(Replay, SecondPlanAfterGoalEndStartsANewGoal)
{
  std::vector<double> stamps = evenly(0.05, kPeriod, 10);
  const std::vector<double> later = evenly(1.05, kPeriod, 10);
  stamps.insert(stamps.end(), later.begin(), later.end());
  const Recording r = synthetic({0.0, 1.0}, stamps, [](size_t i) { return i == 9 ? "goal_reached" : "ok"; });
  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 1.0 / kPeriod, false);
  const ReplayResult result = replay(*c, r, setup_for(World()), ReplayOptions{});
  EXPECT_EQ(result.goal_starts, 2);
  EXPECT_EQ(result.goal_updates, 0);
}

TEST(Replay, PlanDuringGoalUpdatesInPlace)
{
  const Recording r = synthetic({0.0, 0.52}, evenly(0.05, kPeriod, 20), [](size_t) { return "ok"; });
  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 1.0 / kPeriod, false);
  const ReplayResult result = replay(*c, r, setup_for(World()), ReplayOptions{});
  EXPECT_EQ(result.goal_starts, 1);
  EXPECT_EQ(result.goal_updates, 1);
}

TEST(Replay, RateEmulationKeepsEverySecondAndAHalfCycle)
{
  const Recording r = synthetic({0.0}, evenly(0.02, 0.02, 100), [](size_t) { return "ok"; });
  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 20.0, false);
  ReplayOptions options;
  options.rate_hz = 20.0;
  const ReplayResult result = replay(*c, r, setup_for(World()), options);
  // Due times every 0.05 s, each taking the first recorded cycle at or after it: on the 0.02 s grid the spacing
  // alternates 0.04 / 0.06 s and averages the emulated period.
  ASSERT_GE(result.stamp.size(), 39u);
  ASSERT_LE(result.stamp.size(), 41u);
  for (size_t i = 1; i < result.stamp.size(); ++i) EXPECT_GE(result.stamp[i] - result.stamp[i - 1], 0.04 - 1e-6);
  const double mean = (result.stamp.back() - result.stamp.front()) / static_cast<double>(result.stamp.size() - 1);
  EXPECT_NEAR(mean, 0.05, 1e-3);
  EXPECT_FALSE(result.fidelity.has_value()) << "fidelity only at the recorded rate";
}

TEST(ReplayBag, OldCycleLayoutStillParses)
{
  // The 4B-A bags hold MpcCycle without the Phase 4B-B fields (at least 42 trailing bytes): cut 40 bytes off a
  // serialized message, which removes only new fields, and decode it.
  mmi::msg::MpcCycle cycle;
  cycle.status = "holding";
  cycle.iterations = 123;
  cycle.deadline_missed = true;
  cycle.state_to_command_s = 0.0042;
  cycle.arm_reanchored = true;
  cycle.cycle_cpu_s = 0.5;
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<mmi::msg::MpcCycle>().serialize_message(&cycle, &serialized);
  const auto & raw = serialized.get_rcl_serialized_message();
  std::vector<uint8_t> old(raw.buffer, raw.buffer + raw.buffer_length - 40);
  const mmi::msg::MpcCycle decoded = decode_cycle(old);
  EXPECT_EQ(decoded.status, "holding");
  EXPECT_EQ(decoded.iterations, 123);
  EXPECT_TRUE(decoded.deadline_missed);
  EXPECT_DOUBLE_EQ(decoded.state_to_command_s, 0.0042);
  EXPECT_TRUE(decoded.arm_reanchored);
  EXPECT_DOUBLE_EQ(decoded.cycle_cpu_s, 0.0) << "a field the old layout lacks reads as zero";
}

TEST(Replay, ReportsTheFirstDivergence)
{
  const std::string uri = bag_path("mpc_replay_divergence");
  record_loop(uri, 30);
  Recording r = read_recording(uri, joint_names());
  ASSERT_EQ(r.cycles.size(), 30u);
  EXPECT_FALSE(r.cycles[12].posture_choice.empty()) << "the recorded side choice is read";
  r.cycles[12].plan[1](0) += 0.01;  // the recording disagrees at cycle 12 by 1 cm
  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 1.0 / kPeriod, true);
  ReplayOptions options;
  options.recorded_misses = true;
  const ReplayResult result = replay(*c, r, setup_for(beam_world()), options);
  ASSERT_TRUE(result.fidelity.has_value());
  EXPECT_NEAR(result.fidelity->first_divergence_stamp, r.cycles[12].stamp, 1e-12);
  EXPECT_NEAR(result.stage1_error_m[12], 0.01, 1e-9);
  EXPECT_LT(result.stage1_error_m[11], 1e-9);
  EXPECT_EQ(result.posture_choice[12], r.cycles[12].posture_choice);
  fs::remove_all(uri);
}

TEST(Replay, RestartsAfterAGoalEndWithoutANewPlan)
{
  // The behaviour tree resends FollowPath with the same path after a failed goal: no new /plan is recorded, the
  // recorded cycles resume after a gap.
  std::vector<double> stamps = evenly(0.05, kPeriod, 10);
  const std::vector<double> later = evenly(1.05, kPeriod, 10);
  stamps.insert(stamps.end(), later.begin(), later.end());
  const Recording r = synthetic({0.0}, stamps, [](size_t i) { return i == 9 ? "failed" : "ok"; });
  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 1.0 / kPeriod, false);
  const ReplayResult result = replay(*c, r, setup_for(World()), ReplayOptions{});
  EXPECT_EQ(result.goal_starts, 2);
  EXPECT_EQ(result.status.back(), "ok") << "the second goal tracks";
}

namespace
{
// Sim-time stamps as Unity's 50 Hz clock gives them to a 50 Hz wall loop: repeats and 0.04 s steps (spacing 0, 0.02,
// 0.04 s), on a regular wall clock.
std::vector<double> quantized(int count)
{
  std::vector<double> out;
  double t = 0.0;
  for (int i = 0; i < count; ++i) {
    out.push_back(t);
    t += (i % 3 == 0) ? 0.0 : (i % 3 == 1 ? 0.04 : 0.02);
  }
  return out;
}
}  // namespace

TEST(Replay, QuantizedStampsDoNotRestartTheGoal)
{
  const Recording r = synthetic({-0.01}, quantized(150), [](size_t) { return "ok"; }, evenly(0.0, 0.02, 150));
  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 50.0, false);
  const ReplayResult at_50 = replay(*c, r, setup_for(World()), ReplayOptions{});
  EXPECT_EQ(at_50.goal_starts, 1);
  EXPECT_EQ(at_50.stamp.size(), 150u);
  ReplayOptions emulated;
  emulated.rate_hz = 20.0;
  auto c20 = build_controller(config(), limits(), mpc_test::urdf(), payload(), 20.0, false);
  const ReplayResult at_20 = replay(*c20, r, setup_for(World()), emulated);
  EXPECT_EQ(at_20.goal_starts, 1);
  EXPECT_NEAR(static_cast<double>(at_20.stamp.size()), 60.0, 1.0) << "3 s of wall time at 20 Hz";
}

TEST(Replay, RateWithinOnePercentOfTheRecordingReplaysEveryCycle)
{
  const Recording r = synthetic({-0.01}, evenly(0.0, 0.02, 100), [](size_t) { return "ok"; });
  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 50.0, false);
  ReplayOptions options;
  options.rate_hz = 50.2;
  const ReplayResult result = replay(*c, r, setup_for(World()), options);
  EXPECT_EQ(result.stamp.size(), 100u);
  EXPECT_TRUE(result.fidelity.has_value());
}

TEST(Replay, RestartsAfterItsOwnFailure)
{
  // The replayed controller fails (its first 30 solves fail, past the fallback budget) while the recording tracks:
  // the behaviour tree would resend the goal, so the replay restarts it and tracks again.
  const Recording r = synthetic({-0.01}, evenly(0.0, kPeriod, 80), [](size_t) { return "ok"; });
  auto solver = std::make_unique<ScriptedSolver>([](int call) { return call < 30; });
  Controller c(formulation(), CollisionModel(primitives(), CollisionConfig{}), std::move(solver), limits().breakaway, 3);
  const ReplayResult result = replay(c, r, setup_for(World()), ReplayOptions{});
  EXPECT_GE(result.replay_restarts, 1);
  EXPECT_GE(result.solve_failures, 30);
  EXPECT_EQ(result.status.back(), "ok");
}

TEST(Replay, ProgressIsARateOverTheGateHorizon)
{
  const std::string uri = bag_path("mpc_replay_progress");
  record_loop(uri, 40);
  const Recording r = read_recording(uri, joint_names());
  auto c = build_controller(config(), limits(), mpc_test::urdf(), payload(), 1.0 / kPeriod, true);
  const ReplayResult result = replay(*c, r, setup_for(beam_world()), ReplayOptions{});
  for (size_t i = 5; i < result.progress_mps.size(); ++i) {
    ASSERT_TRUE(std::isfinite(result.progress_mps[i]));
    EXPECT_GT(result.progress_mps[i], 0.0);
    EXPECT_LE(result.progress_mps[i], limits().v_max + 1e-6) << "a speed along the route, not a distance";
  }
  fs::remove_all(uri);
}

TEST(ReplayBag, TrajectoriesThatDoNotMatchTheArmAreSkipped)
{
  const std::string uri = bag_path("mpc_replay_badtraj");
  fs::remove_all(uri);
  {
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage;
    storage.uri = uri;
    storage.storage_id = "mcap";
    writer.open(storage);
    trajectory_msgs::msg::JointTrajectory bad;  // the right joints, no velocities
    bad.joint_names.assign(kArmJoints.begin(), kArmJoints.end());
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.assign(6, 0.1);
    bad.points.push_back(p);
    writer.write(bad, "/arm_controller/joint_trajectory", at(kT0));
    geometry_msgs::msg::Twist sent;
    writer.write(sent, "/cmd_vel_smoothed", at(kT0 + 0.01));
    mmi::msg::WholeBodyPlan plan;
    plan.header.stamp = at(0.0);
    plan.joint_names.assign(kArmJoints.begin(), kArmJoints.end());
    plan.base.resize(2);
    plan.positions.assign(12, 0.0);
    writer.write(plan, "/wholebody_mpc/plan", at(kT0 + 0.011));
    mmi::msg::MpcCycle cycle;
    cycle.header.stamp = at(0.0);
    cycle.status = "ok";
    writer.write(cycle, "/wholebody_mpc/cycle", at(kT0 + 0.012));
  }
  const Recording r = read_recording(uri, joint_names());
  ASSERT_EQ(r.cycles.size(), 1u);
  EXPECT_TRUE(r.cycles[0].arm_v.isZero());
  fs::remove_all(uri);
}
