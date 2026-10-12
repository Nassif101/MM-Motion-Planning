#pragma once
// Shared fixtures of the MPC tests that run closed loops on the installed robot description,
// payload, operating envelope and MPC configuration (test_formulation, test_controller).
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/collision.hpp"
#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/osqp_solver.hpp"
#include "mobile_manipulator_mpc/world.hpp"
#include "mobile_manipulator_navigation/known_world.hpp"

namespace mpc_test
{
using namespace mobile_manipulator_mpc;
using Joints = Eigen::Matrix<double, 6, 1>;

inline std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

inline std::string share(const std::string & package)
{
  return ament_index_cpp::get_package_share_directory(package);
}

inline const Json & payload()
{
  static const Json json = Json::parse(read_file(share("mobile_manipulator_control") + "/config/qualified_payload.json"));
  return json;
}

inline const std::string & urdf()
{
  static const std::string text = read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf");
  return text;
}

inline const std::vector<mobile_manipulator_geometry::Primitive> & primitives()
{
  static const auto instance =
    mobile_manipulator_geometry::load_primitives(urdf(), mobile_manipulator_geometry::payload_from_json(payload()));
  return instance;
}

inline const Limits & limits()
{
  static const Limits instance =
    load_limits(share("mobile_manipulator_navigation") + "/config/nav_operating_envelope.yaml", urdf());
  return instance;
}

inline const MpcConfig & config()
{
  static const MpcConfig instance = load_mpc_config(share("mobile_manipulator_mpc") + "/config/wholebody_mpc.yaml");
  return instance;
}

inline OsqpOptions solver_options()
{
  OsqpOptions options;
  options.eps_abs = options.eps_rel = config().solver_eps;
  options.max_iter = config().solver_max_iter;
  return options;
}

inline Formulation formulation() { return Formulation(config().horizon, config().dt, limits(), config().weights); }

inline const PostureTargets & posture()
{
  static const PostureTargets targets = posture_targets(payload());
  return targets;
}

inline State state(double x, double y, double yaw, const Joints & q)
{
  State s;
  s << x, y, yaw, q;
  return s;
}

inline World world_of(const std::vector<mobile_manipulator_navigation::Box> & boxes)
{
  World world;
  world.set_static(boxes);
  return world;
}
}  // namespace mpc_test
