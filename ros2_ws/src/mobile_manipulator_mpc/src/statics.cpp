#include "mobile_manipulator_mpc/statics.hpp"

#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_mpc/model.hpp"

namespace mobile_manipulator_mpc
{
namespace mmg = mobile_manipulator_geometry;
using Eigen::Vector3d;

namespace
{
constexpr double kGravity = 9.81;
constexpr double kJacobianStep = 1e-6;
}  // namespace

struct StaticTorqueModel::Impl
{
  mmg::Payload payload;
  mmg::FootprintProjector projector;
  double payload_mass;
  std::vector<std::string> child_links;  // per arm joint, the link it moves

  Impl(const std::string & urdf_text, const mobile_manipulator_navigation::Json & qualified_payload)
  : payload(mmg::payload_from_json(qualified_payload)),
    projector(urdf_text, payload),
    payload_mass(qualified_payload.at("payload").at("mass_kg").get<double>())
  {
    for (const char * name : kArmJoints) {
      const auto joint = projector.model().getJoint(name);
      if (!joint) throw std::runtime_error(std::string("StaticTorqueModel: no joint ") + name + " in the URDF");
      child_links.push_back(joint->child_link_name);
    }
  }
};

StaticTorqueModel::StaticTorqueModel(const std::string & urdf_text,
                                     const mobile_manipulator_navigation::Json & qualified_payload)
: impl_(std::make_unique<Impl>(urdf_text, qualified_payload))
{
}
StaticTorqueModel::~StaticTorqueModel() = default;
StaticTorqueModel::StaticTorqueModel(StaticTorqueModel &&) noexcept = default;
StaticTorqueModel & StaticTorqueModel::operator=(StaticTorqueModel &&) noexcept = default;

Eigen::Matrix<double, 6, 1> StaticTorqueModel::torque(const Eigen::Matrix<double, 6, 1> & q) const
{
  mmg::JointMap joints;
  for (int i = 0; i < 6; ++i) joints[kArmJoints[i]] = q(i);
  const auto & model = impl_->projector.model();
  const Vector3d force_dir(0.0, 0.0, -kGravity);
  const Vector3d payload_com = impl_->projector.link_pose(impl_->payload.link, joints) * impl_->payload.center;
  Eigen::Matrix<double, 6, 1> tau;
  for (int i = 0; i < 6; ++i) {
    const std::string & child = impl_->child_links[i];
    const Eigen::Isometry3d frame = impl_->projector.link_pose(child, joints);
    const auto & a = model.getLink(child)->parent_joint->axis;
    const Vector3d axis = frame.linear() * Vector3d(a.x, a.y, a.z);
    double t = axis.dot((payload_com - frame.translation()).cross(impl_->payload_mass * force_dir));
    std::vector<std::string> stack = {child};
    while (!stack.empty()) {
      const auto link = model.getLink(stack.back());
      stack.pop_back();
      if (link->inertial) {
        const auto & o = link->inertial->origin.position;
        const Vector3d c = impl_->projector.link_pose(link->name, joints) * Vector3d(o.x, o.y, o.z);
        t += axis.dot((c - frame.translation()).cross(link->inertial->mass * force_dir));
      }
      for (const auto & child_link : link->child_links) stack.push_back(child_link->name);
    }
    tau(i) = t;
  }
  return tau;
}

Eigen::Matrix<double, 6, 6> StaticTorqueModel::jacobian(const Eigen::Matrix<double, 6, 1> & q) const
{
  Eigen::Matrix<double, 6, 6> J;
  for (int j = 0; j < 6; ++j) {
    Eigen::Matrix<double, 6, 1> dq = Eigen::Matrix<double, 6, 1>::Zero();
    dq(j) = kJacobianStep;
    J.col(j) = (torque(q + dq) - torque(q - dq)) / (2.0 * kJacobianStep);
  }
  return J;
}
}  // namespace mobile_manipulator_mpc
