#pragma once
// The realistic arm (MuJoCo plant spec Section 4.4): the streamed reference as the JTC samples it, and the joint servos
// with MuJoCo's own inverse-dynamics feedforward, entered as a shift of the servo target (Unity's mechanism, ADR 0014),
// so the actuators' force limits cap the total.
#include <mujoco/mujoco.h>

#include <vector>

#include "mobile_manipulator_plant/robot_profile.hpp"
#include "mobile_manipulator_plant/types.hpp"

namespace mobile_manipulator_plant
{
// Position and velocity of a streamed trajectory at plant time `t`, as the JTC samples it: a cubic Hermite spline
// between points; the first point before it; at rest at the last point after it; `q_hold` at rest for an empty stream.
void sample_stream(const ArmReference & reference, const Vec6 & q_hold, double t, Vec6 & q, Vec6 & v);

struct ArmTarget
{
  Vec6 q, v, a;
};

// The JTC at its update rate: each tick samples the stream (q, v) and the realized acceleration (the velocity change
// since the last tick, capped as ADR 0014 caps it); between ticks the last sample holds (the drive's target).
// The base's reference motion for the arm's feedforward (Unity's ArmBaseMotion.FromBaseCommand): the skid-steer
// limiter's speed, acceleration, yaw rate and yaw acceleration, in the base frame.
struct BaseMotion
{
  double speed = 0.0, acceleration = 0.0, yaw_rate = 0.0, yaw_acceleration = 0.0;
};

class JtcSampler
{
public:
  // `accel_limit` per joint; a command with every velocity below 1e-6 has zero acceleration (Unity's HoldAcceleration).
  JtcSampler(double rate_hz, const Vec6 & accel_limit);
  void reset(const Vec6 & q);
  // `q_hold`: where an empty stream holds the arm (the reference commanded last).
  ArmTarget sample(const ArmReference & reference, double t, const Vec6 & q_hold);

private:
  double period_;
  Vec6 accel_limit_;
  double first_tick_ = 0.0;
  long ticks_ = 0;  // integer tick count: no drift over long runs
  bool started_ = false;
  ArmTarget held_{Vec6::Zero(), Vec6::Zero(), Vec6::Zero()};
};

class ArmServo
{
public:
  ArmServo(const mjModel * m, const RobotProfile & profile, bool gravity_ff, bool motion_ff);
  ~ArmServo();
  ArmServo(const ArmServo &) = delete;
  ArmServo & operator=(const ArmServo &) = delete;

  // Sets the six servos' controls for `target` at the state in `d`: ctrl = q_r + (kd / kp) v_r + tau_ff / kp, tau_ff the
  // gravity (and, with motion feedforward, the inertial, Coriolis and armature) torque of the reference from mj_rne, the
  // base moving as `base` says (Unity's base reference motion).
  void apply(mjData * d, const ArmTarget & target, const BaseMotion & base);
  int qadr(int joint) const { return qadr_[joint]; }
  int dofadr(int joint) const { return dofadr_[joint]; }

private:
  const mjModel * m_;
  mjData * scratch_;
  Vec6 kp_, kd_;
  bool gravity_ff_, motion_ff_;
  Vec6 armature_, damping_, frictionloss_;  // the joints' passive terms: the feedforward covers damping and friction
  int qadr_[6], dofadr_[6], actuator_[6];
  // the base: planar joints (x, y, yaw dofs) or a free joint (the wheels model: linear dofs 0-2, the body z rate dof 5)
  int base_qadr_yaw_ = -1, base_dof_[3] = {-1, -1, -1};
  int free_qadr_ = -1;
  std::vector<mjtNum> out_;
};
}  // namespace mobile_manipulator_plant
