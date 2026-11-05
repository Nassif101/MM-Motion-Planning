#include "mobile_manipulator_plant/arm_servo.hpp"

#include <algorithm>
#include <stdexcept>

namespace mobile_manipulator_plant
{
// mobile_manipulator_mpc::sample_trajectory on the plant's stream type (checked against it in the campaign package).
void sample_stream(const ArmReference & reference, const Vec6 & q_hold, double t_plant, Vec6 & q, Vec6 & v)
{
  const auto & points = reference.points;
  if (points.empty()) {
    q = q_hold;
    v.setZero();
    return;
  }
  const double t = t_plant - reference.start_time;
  if (t <= points.front().t) {
    q = points.front().q;
    v = points.front().v;
    return;
  }
  size_t i = 0;
  while (i + 1 < points.size() && points[i + 1].t < t) ++i;
  if (i + 1 >= points.size()) {
    q = points.back().q;
    v.setZero();
    return;
  }
  const double t0 = points[i].t, h = points[i + 1].t - t0;
  if (!(h > 0.0)) {
    q = points[i + 1].q;
    v = points[i + 1].v;
    return;
  }
  const double s = std::clamp((t - t0) / h, 0.0, 1.0), s2 = s * s, s3 = s2 * s;
  const Vec6 & p0 = points[i].q, & p1 = points[i + 1].q;
  const Vec6 m0 = h * points[i].v, m1 = h * points[i + 1].v;
  q = (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * m0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * m1;
  v = ((6 * s2 - 6 * s) * p0 + (3 * s2 - 4 * s + 1) * m0 + (-6 * s2 + 6 * s) * p1 + (3 * s2 - 2 * s) * m1) / h;
}

JtcSampler::JtcSampler(double rate_hz, double accel_limit) : period_(1.0 / rate_hz), accel_limit_(accel_limit)
{
  if (!(rate_hz > 0.0)) throw std::invalid_argument("JtcSampler: the rate must be positive");
}

void JtcSampler::reset(const Vec6 & q)
{
  held_ = {q, Vec6::Zero(), Vec6::Zero()};
  started_ = false;
  next_tick_ = 0.0;
}

ArmTarget JtcSampler::sample(const ArmReference & reference, double t)
{
  if (!started_ || t >= next_tick_ - 1e-9) {
    Vec6 q, v;
    sample_stream(reference, held_.q, t, q, v);
    // the realized acceleration: the commanded velocity's change since the last tick (ADR 0014), capped
    const Vec6 a = started_ ? Vec6(((v - held_.v) / period_).cwiseMax(-accel_limit_).cwiseMin(accel_limit_)) : Vec6::Zero();
    held_ = {q, v, a};
    next_tick_ = (started_ ? next_tick_ : t) + period_;
    while (next_tick_ <= t + 1e-9) next_tick_ += period_;
    started_ = true;
  }
  return held_;
}

ArmServo::ArmServo(const mjModel * m, const RobotProfile & profile, bool gravity_ff, bool motion_ff)
: m_(m), scratch_(mj_makeData(m)), kp_(profile.kp), kd_(profile.kd), gravity_ff_(gravity_ff), motion_ff_(motion_ff)
{
  for (int i = 0; i < 6; ++i) {
    const int j = mj_name2id(m, mjOBJ_JOINT, profile.arm_joints[i].c_str());
    actuator_[i] = mj_name2id(m, mjOBJ_ACTUATOR, ("servo/" + profile.arm_joints[i]).c_str());
    if (j < 0 || actuator_[i] < 0) throw std::runtime_error("ArmServo: no joint or servo for " + profile.arm_joints[i]);
    qadr_[i] = m->jnt_qposadr[j];
    dofadr_[i] = m->jnt_dofadr[j];
  }
}

ArmServo::~ArmServo() { mj_deleteData(scratch_); }

void ArmServo::apply(mjData * d, const ArmTarget & target)
{
  Vec6 tau = Vec6::Zero();
  if (gravity_ff_ || motion_ff_) {
    // the reference state (the arm at q_r, v_r, a_r; the base as it is) through MuJoCo's recursive Newton-Euler
    mju_copy(scratch_->qpos, d->qpos, m_->nq);
    if (motion_ff_) mju_copy(scratch_->qvel, d->qvel, m_->nv);
    else mju_zero(scratch_->qvel, m_->nv);
    mju_zero(scratch_->qacc, m_->nv);
    for (int i = 0; i < 6; ++i) {
      scratch_->qpos[qadr_[i]] = target.q(i);
      if (motion_ff_) {
        scratch_->qvel[dofadr_[i]] = target.v(i);
        scratch_->qacc[dofadr_[i]] = target.a(i);
      }
    }
    mj_kinematics(m_, scratch_);
    mj_comPos(m_, scratch_);
    mj_comVel(m_, scratch_);
    std::vector<mjtNum> out(m_->nv);
    mj_rne(m_, scratch_, motion_ff_ ? 1 : 0, out.data());
    for (int i = 0; i < 6; ++i) tau(i) = out[dofadr_[i]];
    if (!gravity_ff_) {  // the motion terms alone: remove the gravity part
      mju_zero(scratch_->qvel, m_->nv);
      mj_comVel(m_, scratch_);
      mj_rne(m_, scratch_, 0, out.data());
      for (int i = 0; i < 6; ++i) tau(i) -= out[dofadr_[i]];
    }
  }
  // force = kp (ctrl - q) - kd qdot = kp (q_r - q) + kd (v_r - qdot) + tau
  for (int i = 0; i < 6; ++i) d->ctrl[actuator_[i]] = target.q(i) + kd_(i) / kp_(i) * target.v(i) + tau(i) / kp_(i);
}
}  // namespace mobile_manipulator_plant
