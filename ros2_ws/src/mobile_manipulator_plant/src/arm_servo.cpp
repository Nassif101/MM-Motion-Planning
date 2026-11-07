#include "mobile_manipulator_plant/arm_servo.hpp"

#include <algorithm>
#include <cmath>
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

JtcSampler::JtcSampler(double rate_hz, const Vec6 & accel_limit) : period_(1.0 / rate_hz), accel_limit_(accel_limit)
{
  if (!(rate_hz > 0.0)) throw std::invalid_argument("JtcSampler: the rate must be positive");
}

void JtcSampler::reset(const Vec6 & q)
{
  held_ = {q, Vec6::Zero(), Vec6::Zero()};
  started_ = false;
  ticks_ = 0;
}

ArmTarget JtcSampler::sample(const ArmReference & reference, double t, const Vec6 & q_hold)
{
  if (!started_) {
    first_tick_ = t;
    ticks_ = 0;
  }
  if (!started_ || t >= first_tick_ + (ticks_ + 1) * period_ - 1e-9) {
    if (started_) ticks_ = static_cast<long>(std::floor((t - first_tick_) / period_ + 1e-9));
    Vec6 q, v;
    sample_stream(reference, q_hold, t, q, v);
    // the realized acceleration: the commanded velocity's change since the last tick (ADR 0014), capped per joint, zero
    // for a still command (Unity's HoldAcceleration)
    Vec6 a = Vec6::Zero();
    const bool still = (v.array().abs() < 1e-6).all();
    if (started_ && !still) a = ((v - held_.v) / period_).cwiseMax(-accel_limit_).cwiseMin(accel_limit_);
    held_ = {q, v, a};
    started_ = true;
  }
  return held_;
}

ArmServo::ArmServo(const mjModel * m, const RobotProfile & profile, bool gravity_ff, bool motion_ff)
: m_(m), scratch_(mj_makeData(m)), kp_(profile.kp), kd_(profile.kd), gravity_ff_(gravity_ff), motion_ff_(motion_ff),
  armature_(profile.armature), damping_(profile.damping), frictionloss_(profile.frictionloss), out_(m->nv)
{
  const int free = mj_name2id(m, mjOBJ_JOINT, "base_free");
  if (free >= 0) {  // the base on its wheels
    free_qadr_ = m->jnt_qposadr[free];
    const int dof = m->jnt_dofadr[free];
    base_dof_[0] = dof;
    base_dof_[1] = dof + 1;
    base_dof_[2] = dof + 5;
  } else {
    const char * base_joints[3] = {"base_x", "base_y", "base_yaw"};
    for (int i = 0; i < 3; ++i) {
      const int j = mj_name2id(m, mjOBJ_JOINT, base_joints[i]);
      if (j < 0) throw std::runtime_error(std::string("ArmServo: no joint ") + base_joints[i]);
      base_dof_[i] = m->jnt_dofadr[j];
      if (i == 2) base_qadr_yaw_ = m->jnt_qposadr[j];
    }
  }
  for (int i = 0; i < 6; ++i) {
    const int j = mj_name2id(m, mjOBJ_JOINT, profile.arm_joints[i].c_str());
    actuator_[i] = mj_name2id(m, mjOBJ_ACTUATOR, ("servo/" + profile.arm_joints[i]).c_str());
    if (j < 0 || actuator_[i] < 0) throw std::runtime_error("ArmServo: no joint or servo for " + profile.arm_joints[i]);
    qadr_[i] = m->jnt_qposadr[j];
    dofadr_[i] = m->jnt_dofadr[j];
  }
}

ArmServo::~ArmServo() { mj_deleteData(scratch_); }

void ArmServo::apply(mjData * d, const ArmTarget & target, const BaseMotion & base)
{
  Vec6 tau = Vec6::Zero();
  if (gravity_ff_ || motion_ff_) {
    // the reference state through MuJoCo's recursive Newton-Euler: the arm at q_r, v_r, a_r; the base where it is, moving
    // with its reference motion (world-frame planar joints from the base-frame speed, acceleration and yaw terms)
    mju_copy(scratch_->qpos, d->qpos, m_->nq);
    mju_zero(scratch_->qvel, m_->nv);
    mju_zero(scratch_->qacc, m_->nv);
    if (motion_ff_) {
      // the free base's quaternion (w, x, y, z), only formed when there is a free base (free_qadr_ -1 otherwise)
      const double * q = free_qadr_ >= 0 ? d->qpos + free_qadr_ + 3 : nullptr;
      const double yaw = q ? std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
                                         : d->qpos[base_qadr_yaw_];
      const double c = std::cos(yaw), s = std::sin(yaw);
      scratch_->qvel[base_dof_[0]] = base.speed * c;
      scratch_->qvel[base_dof_[1]] = base.speed * s;
      scratch_->qvel[base_dof_[2]] = base.yaw_rate;
      scratch_->qacc[base_dof_[0]] = base.acceleration * c - base.speed * base.yaw_rate * s;
      scratch_->qacc[base_dof_[1]] = base.acceleration * s + base.speed * base.yaw_rate * c;
      scratch_->qacc[base_dof_[2]] = base.yaw_acceleration;
    }
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
    mj_rne(m_, scratch_, motion_ff_ ? 1 : 0, out_.data());
    for (int i = 0; i < 6; ++i) tau(i) = out_[dofadr_[i]];
    if (motion_ff_) tau += armature_.cwiseProduct(target.a);  // mj_rne leaves out the rotors' armature
    if (!gravity_ff_) {  // the motion terms alone: remove the gravity part
      mju_zero(scratch_->qvel, m_->nv);
      mj_comVel(m_, scratch_);
      mj_rne(m_, scratch_, 0, out_.data());
      for (int i = 0; i < 6; ++i) tau(i) -= out_[dofadr_[i]];
    }
  }
  // the joints' passive terms at the reference velocity: viscous damping and (smoothed over 0.01 rad/s) dry friction
  if (motion_ff_)
    for (int i = 0; i < 6; ++i) tau(i) += damping_(i) * target.v(i) + frictionloss_(i) * std::tanh(target.v(i) / 0.01);
  // force = kp (ctrl - q) - kd qdot = kp (q_r - q) + kd (v_r - qdot) + tau
  for (int i = 0; i < 6; ++i) d->ctrl[actuator_[i]] = target.q(i) + kd_(i) / kp_(i) * target.v(i) + tau(i) / kp_(i);
}
}  // namespace mobile_manipulator_plant
