#pragma once
// The wholebody_mpc_node's arm command path, mirrored for the MuJoCo plant (MuJoCo plant plan Task 6; node functions
// arm_reference, stream and hold_arm, rulings R21 and R24, ADR 0014): B5's arm state is the last trajectory sent,
// sampled since it was sent, unless it is farther than 0.15 rad from the measured joints (a re-anchor); Ok and Fallback
// cycles stream the plan (point 0 on the reference, the plan's points by mmm::plan_stream_points, a braking tail at
// rest); braking, holding, stale and blocked cycles send one braking hold (mmm::safe_arm_hold); goal and failure
// cycles send nothing. Mirrored, not shared: the frozen node stays untouched (Task 6 ruling).
#include <optional>
#include <vector>

#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/collision.hpp"
#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/world.hpp"
#include "mobile_manipulator_plant/types.hpp"

namespace mobile_manipulator_campaign
{
class ArmCommandPath
{
public:
  ArmCommandPath(const mobile_manipulator_mpc::Limits & limits, const mobile_manipulator_mpc::MpcConfig & config,
                 const std::vector<mobile_manipulator_geometry::Primitive> & primitives);
  void reset();

  // B5's state at `t`: the measured one with the arm replaced by the reference (the last trajectory sent, sampled at
  // t - its send time) while every joint is within 0.15 rad of the measured joints; else the measured joints (a
  // re-anchor). Before anything was sent: the measured state. Sets reference_v() and reanchored().
  mobile_manipulator_mpc::State b5_state(const mobile_manipulator_mpc::State & measured, double t);
  const mobile_manipulator_plant::Vec6 & reference_v() const { return reference_v_; }
  bool reanchored() const { return reanchored_; }

  // The trajectory to send for cycle `out` at `t` (stamped for the JTC to start on receipt: start_time is set by the
  // caller), or nullopt to keep the last one. `world` (may be null: the hold is not checked) and `base_vw` (the base
  // command) feed the hold's obstacle check.
  std::optional<mobile_manipulator_plant::ArmReference> command(const mobile_manipulator_mpc::CycleOutput & out,
                                                                 const mobile_manipulator_mpc::State & measured, double t,
                                                                 const mobile_manipulator_mpc::World * world,
                                                                 const Eigen::Vector2d & base_vw);

private:
  mobile_manipulator_plant::ArmReference stream(const mobile_manipulator_mpc::Plan & plan);
  mobile_manipulator_plant::ArmReference hold(const mobile_manipulator_mpc::State & measured, double t,
                                              const mobile_manipulator_mpc::World * world, const Eigen::Vector2d & base_vw);

  mobile_manipulator_mpc::Limits limits_;
  mobile_manipulator_mpc::MpcConfig config_;
  mobile_manipulator_mpc::CollisionModel collision_;
  std::optional<mobile_manipulator_plant::ArmReference> sent_;
  double sent_at_ = 0.0;
  bool held_ = false, reanchored_ = false;
  mobile_manipulator_plant::Vec6 reference_v_ = mobile_manipulator_plant::Vec6::Zero();
};
}  // namespace mobile_manipulator_campaign
