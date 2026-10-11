#pragma once
// Whole-body kinematic model of the mobile manipulator, generated with CasADi at build time
// (scripts/generate_model.py) from the URDF and the qualified payload.
//
// State x = (x, y, yaw, q1..q6): base_footprint pose in the map frame and the six arm joints
// in qualified_payload.json joint_order. Control u = (v, omega, qdot1..qdot6): unicycle base
// (no lateral motion) and arm joint velocities. Joints outside the arm (the wheels) are at
// zero, as in mobile_manipulator_geometry::FootprintProjector.
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mobile_manipulator_mpc
{
constexpr int kNx = 9, kNu = 8;
using State = Eigen::Matrix<double, kNx, 1>;
using Control = Eigen::Matrix<double, kNu, 1>;

struct Step
{
  State x_next;
  Eigen::Matrix<double, kNx, kNx> A;  // d x_next / d x
  Eigen::Matrix<double, kNx, kNu> B;  // d x_next / d u
};

// One RK4 step of length dt with u held constant, and its Jacobians.
Step discrete_step(const State & x, const Control & u, double dt);

// Index of a link that carries a collision primitive (URDF collision geometry or the payload
// box, whose link is tool0), or of "panel": the payload box centre with the payload link's
// axes. Throws std::invalid_argument for any other name.
int link_index(const std::string & link);

// map -> link (base pose from x). Functions taking a link index throw std::out_of_range for
// an index that link_index did not return.
Eigen::Isometry3d link_pose(int link, const State & x);

struct PointJacobian
{
  Eigen::Vector3d p;               // point in the map frame
  Eigen::Matrix<double, 3, kNx> J;  // d p / d x
};

// A point fixed in the link frame (p_link), in the map frame, and its Jacobian.
PointJacobian point_jacobian(int link, const State & x, const Eigen::Vector3d & p_link);

struct PanelError
{
  Eigen::Vector4d e;
  Eigen::Matrix<double, 4, kNx> J;  // d e / d x; the x, y, yaw columns are zero
};

// Panel pose error in base_footprint against the reference panel_in_base (base_footprint ->
// panel centre): e = (p - p_ref, 0.5 * trace(I - R_ref^T R)).
PanelError panel_error(const State & x, const Eigen::Isometry3d & panel_in_base);

// Translational manipulability sqrt(det(Jt Jt^T)) of tool0 over the six arm joints.
double manipulability(const State & x);
}  // namespace mobile_manipulator_mpc
