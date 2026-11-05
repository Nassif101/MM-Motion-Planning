#pragma once
// The realistic arm (MuJoCo plant spec Section 4.4): the streamed reference as the JTC samples it, and the joint servos
// with MuJoCo's own inverse-dynamics feedforward, entered as a shift of the servo target (Unity's mechanism, ADR 0014),
// so the actuators' force limits cap the total.
#include <mujoco/mujoco.h>

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
class JtcSampler
{
public:
  JtcSampler(double rate_hz, double accel_limit);
  void reset(const Vec6 & q);
  ArmTarget sample(const ArmReference & reference, double t);

private:
  double period_, accel_limit_;
  double next_tick_ = 0.0;
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
  // gravity (and, with motion feedforward, the inertial and Coriolis) torque of the reference from mj_rne.
  void apply(mjData * d, const ArmTarget & target);
  int qadr(int joint) const { return qadr_[joint]; }
  int dofadr(int joint) const { return dofadr_[joint]; }

private:
  const mjModel * m_;
  mjData * scratch_;
  Vec6 kp_, kd_;
  bool gravity_ff_, motion_ff_;
  int qadr_[6], dofadr_[6], actuator_[6];
};
}  // namespace mobile_manipulator_plant
