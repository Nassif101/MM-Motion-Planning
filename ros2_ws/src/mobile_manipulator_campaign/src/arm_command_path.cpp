#include "mobile_manipulator_campaign/arm_command_path.hpp"

#include <algorithm>

#include "mobile_manipulator_mpc/arm_stream.hpp"
#include "mobile_manipulator_plant/arm_servo.hpp"

namespace mobile_manipulator_campaign
{
namespace mmm = mobile_manipulator_mpc;
namespace mmp = mobile_manipulator_plant;

namespace
{
// wholebody_mpc_node constants
constexpr double kTailS = 0.5;         // shortest braking tail after the horizon
constexpr double kMinLeadS = 0.005;    // a streamed point closer than this to now is already due
constexpr double kReanchorRad = 0.15;  // reference farther than this from the measured joints: re-anchor
// The node streams with the state-to-command latency as the offset (a few ms, host-dependent); the plant's cycle is
// instantaneous, so 0 (deterministic). The transport delay is the plant's command delay.
constexpr double kOffsetS = 0.0;
}  // namespace

ArmCommandPath::ArmCommandPath(const mmm::Limits & limits, const mmm::MpcConfig & config,
                               const std::vector<mobile_manipulator_geometry::Primitive> & primitives)
: limits_(limits), config_(config), collision_(primitives, config.collision)
{
}

void ArmCommandPath::reset()
{
  sent_.reset();
  sent_at_ = 0.0;
  held_ = false;
  reanchored_ = false;
  reference_v_.setZero();
}

mmm::State ArmCommandPath::b5_state(const mmm::State & measured, double t)
{
  mmm::State x = measured;
  reanchored_ = false;
  reference_v_.setZero();
  if (!sent_) return x;
  mmp::Vec6 q, v;
  mmp::ArmReference since_sent = *sent_;
  since_sent.start_time = sent_at_;  // the node samples from the send time (the JTC starts on receipt)
  mmp::sample_stream(since_sent, measured.tail<6>(), t, q, v);
  reference_v_ = v.cwiseMax(-limits_.qd_max).cwiseMin(limits_.qd_max);
  // the plant's joint states are never stale
  if ((q - measured.tail<6>()).cwiseAbs().maxCoeff() <= kReanchorRad) {
    x.tail<6>() = q;
    return x;
  }
  reanchored_ = true;
  return x;
}

std::optional<mmp::ArmReference> ArmCommandPath::command(const mmm::CycleOutput & out, const mmm::State & measured, double t,
                                                         const mmm::World * world, const Eigen::Vector2d & base_vw)
{
  switch (out.status) {
    case mmm::CycleStatus::Ok:
    case mmm::CycleStatus::Fallback:
      sent_ = stream(out.plan);
      sent_at_ = t;
      held_ = false;
      return sent_;
    case mmm::CycleStatus::Braking:
    case mmm::CycleStatus::StaleState:
    case mmm::CycleStatus::Holding:
    case mmm::CycleStatus::Blocked:
      if (held_) return std::nullopt;  // one braking hold per hold
      sent_ = hold(measured, t, world, base_vw);
      sent_at_ = t;
      held_ = true;
      return sent_;
    case mmm::CycleStatus::Failed:
    case mmm::CycleStatus::GoalReached:
      break;
  }
  return std::nullopt;
}

// wholebody_mpc_node::stream: point 0 on the plan's stage 0 advanced by the offset at the reference velocity, the plan's
// points, a braking tail that ends at rest (constant deceleration, at least kTailS and long enough for every joint's
// acceleration limit).
mmp::ArmReference ArmCommandPath::stream(const mmm::Plan & plan)
{
  const int n = static_cast<int>(plan.u.size());
  const mmp::Vec6 v0 = reference_v_;
  const mmp::Vec6 q0 = (plan.x[0].tail<6>() + kOffsetS * v0).cwiseMax(limits_.q_min).cwiseMin(limits_.q_max);
  std::vector<Eigen::Matrix<double, 6, 1>> plan_q(static_cast<size_t>(n + 1)), plan_u(static_cast<size_t>(n));
  for (int k = 0; k <= n; ++k) plan_q[static_cast<size_t>(k)] = plan.x[static_cast<size_t>(k)].tail<6>();
  for (int k = 0; k < n; ++k) plan_u[static_cast<size_t>(k)] = plan.u[static_cast<size_t>(k)].tail<6>();
  const std::vector<mmm::StreamPoint> points =
    mmm::plan_stream_points(plan_q, plan_u, config_.dt, kOffsetS, kMinLeadS, config_.stream_dense_s, config_.stream_dense_dt_s);
  mmp::ArmReference r;
  r.points.push_back({0.0, q0, v0});
  for (const auto & p : points) r.points.push_back({p.t, p.q, p.v});
  const mmp::Vec6 v = plan.u[static_cast<size_t>(n - 1)].tail<6>();
  const double tail = std::max(kTailS, (v.cwiseAbs().array() / limits_.qdd_max.array()).maxCoeff());
  const mmp::Vec6 end = (plan.x[static_cast<size_t>(n)].tail<6>() + 0.5 * tail * v).cwiseMax(limits_.q_min).cwiseMin(limits_.q_max);
  r.points.push_back({std::max(n * config_.dt - kOffsetS, kMinLeadS) + tail, end, mmp::Vec6::Zero()});
  return r;
}

// wholebody_mpc_node::hold_arm: from the reference now (the measured joints if it is farther than kReanchorRad from
// them), the constant-deceleration stop checked against the obstacles (mmm::safe_arm_hold).
mmp::ArmReference ArmCommandPath::hold(const mmm::State & measured, double t, const mmm::World * world, const Eigen::Vector2d & base_vw)
{
  mmp::Vec6 q = measured.tail<6>(), v = mmp::Vec6::Zero();
  if (sent_) {
    mmp::ArmReference since_sent = *sent_;
    since_sent.start_time = sent_at_;
    mmp::Vec6 q_ref;
    mmp::sample_stream(since_sent, q, t, q_ref, v);
    v = v.cwiseMax(-limits_.qd_max).cwiseMin(limits_.qd_max);
    if ((q_ref - q).cwiseAbs().maxCoeff() <= kReanchorRad) q = q_ref;
  }
  mmm::State at = measured;
  at.tail<6>() = q;
  double duration = 0.0;
  mmp::Vec6 target;
  if (world) {
    const mmm::ArmHold checked = mmm::safe_arm_hold(collision_, *world, t, at, v, limits_, base_vw);
    duration = checked.duration_s;
    target = checked.target;
  } else {  // unchecked: the plain constant-deceleration stop
    duration = std::max(mmm::kArmHoldMinS, (v.cwiseAbs().array() / limits_.qdd_max.array()).maxCoeff());
    target = (q + 0.5 * duration * v).cwiseMax(limits_.q_min).cwiseMin(limits_.q_max);
  }
  mmp::ArmReference r;
  r.points.push_back({0.0, q, v});
  r.points.push_back({duration, target, mmp::Vec6::Zero()});
  return r;
}
}  // namespace mobile_manipulator_campaign
