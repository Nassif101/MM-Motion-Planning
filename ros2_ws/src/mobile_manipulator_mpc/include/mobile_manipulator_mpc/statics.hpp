#pragma once
// The arm's static gravity torque (Phase 5A Task 2): per arm joint, the torque gravity exerts about the joint's axis
// on everything below it (the URDF links' masses and the qualified payload); the actuator holds its negative. It
// depends on the arm joints only (the base turns about the vertical). The medium layer's torque constraint and
// carry_pose_search use it.
#include <memory>
#include <string>

#include <Eigen/Core>

#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_mpc
{
class StaticTorqueModel
{
public:
  // `urdf_text` the robot description, `qualified_payload` qualified_payload.json (payload box, link and mass).
  // Throws std::runtime_error when a joint of kArmJoints is missing from the URDF.
  StaticTorqueModel(const std::string & urdf_text, const mobile_manipulator_navigation::Json & qualified_payload);
  ~StaticTorqueModel();
  StaticTorqueModel(StaticTorqueModel &&) noexcept;
  StaticTorqueModel & operator=(StaticTorqueModel &&) noexcept;

  // N m about each joint axis, in the model's joint order (kArmJoints).
  Eigen::Matrix<double, 6, 1> torque(const Eigen::Matrix<double, 6, 1> & q) const;
  // d torque / d q by central differences (step 1e-6 rad).
  Eigen::Matrix<double, 6, 6> jacobian(const Eigen::Matrix<double, 6, 1> & q) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace mobile_manipulator_mpc
