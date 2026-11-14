#pragma once
// E2 (spec docs/superpowers/specs/2026-11-11-e2-tracking-prediction-design.md Section 3.3 and Amendment 1; plan Task 5):
// how the base executes the command, in B5's prediction. Parameters as data, one file per plant
// (config/tracking_model_<plant>.yaml); all deltas 0 is the identity, which calls today's generated step unchanged.
#include <array>
#include <string>
#include <vector>

#include "mobile_manipulator_mpc/model.hpp"

namespace mobile_manipulator_mpc
{
struct TrackingModel
{
  // the turn map: yaw rate = g_inf omega - (g_inf - g_0) turn_scale tanh(omega / turn_scale), g_inf = 1 + turn_delta_inf,
  // g_0 = 1 + turn_delta_zero (the slope at zero, which the linearized QP sees for a small turn); both deltas 0:
  // yaw rate = omega
  double turn_delta_inf = 0.0;
  double turn_delta_zero = 0.0;
  double turn_scale = 0.05;  // rad/s; only used when a turn delta is non-zero
  double speed_turn = 0.0;   // forward speed = v (1 - speed_turn sabs(yaw rate)), sabs(x) = sqrt(x^2 + 1e-4) - 0.01
  double sideslip = 0.0;     // lateral speed = sideslip x forward speed x yaw rate (s; body left positive)
  // spec Amendment 3: the base turns about a point off its frame that follows the centre of mass: lateral speed +=
  // (turn_offset + turn_offset_com x com_x(arm)) x yaw rate, com_x the whole robot's centre of mass x in base_footprint
  double turn_offset = 0.0;      // m
  double turn_offset_com = 0.0;  // dimensionless
  double base_delay_s = 0.0;  // the base acts on a command this long after it was sent (Task 8)
  double arm_delay_s = 0.0;   // the arm likewise (Task 8: the pipeline's delay plus lag, an effective value)

  // turn_delta_inf, turn_delta_zero, speed_turn, sideslip, turn_offset and turn_offset_com all exactly 0
  bool map_identity() const;
  // map_identity() and both delays 0
  bool identity() const;
  // turn_delta_inf, turn_delta_zero, turn_scale, speed_turn, sideslip, turn_offset, turn_offset_com: the generated
  // step's p
  std::array<double, 7> map_parameters() const;
};

// Reads a tracking model file (a YAML mapping of the fields above; a missing key keeps its default). Throws
// std::runtime_error naming the file and the key for an unknown key, a value that is not a finite number, or one
// outside its range: turn_delta_inf [-0.5, 0.5], turn_delta_zero [-0.99, 0.5], turn_scale [0.001, 1], speed_turn
// [0, 1], sideslip [-0.5, 0.5], turn_offset [-0.3, 0.3] m, turn_offset_com [-5, 5], the delays [0, 0.2] s. A file that cannot be read: std::runtime_error naming it.
TrackingModel load_tracking_model(const std::string & yaml_path);

// E2 Task 8: a command and the time it was sent.
struct SentCommand
{
  double t = 0.0;
  Control u = Control::Zero();
};

// The state at actuation: x (measured at `now`) advanced through the commands not yet acted on. The base states roll
// through the base parts (v, omega) of the commands sent in [now - base_delay_s, now) with the model's map, each held
// until the next one (the last until now); the arm joints likewise integrate the arm parts over
// [now - arm_delay_s, now). Before the first sent command: zero; commands sent at or after now: not yet acting.
// `sent` in time order. Zero delays return x unchanged.
State advance_to_actuation(const State & x, const std::vector<SentCommand> & sent, double now,
                           const TrackingModel & model);

// One RK4 step with the tracking model's map and its Jacobians; an identity map calls discrete_step(x, u, dt).
Step discrete_step(const State & x, const Control & u, double dt, const TrackingModel & model);
}  // namespace mobile_manipulator_mpc
