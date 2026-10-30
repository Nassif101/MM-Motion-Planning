#pragma once
// Test helpers of the medium layer: the installed robot description, payload, envelope and a problem on a route.
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_medium/problem.hpp"
#include "mobile_manipulator_navigation/known_world.hpp"

namespace medium_test
{
namespace mmd = mobile_manipulator_medium;
namespace mmm = mobile_manipulator_mpc;
namespace mmn = mobile_manipulator_navigation;
using Joints = mmd::Joints;

inline std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

inline std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

inline const mmn::Json & payload()
{
  static const mmn::Json json = mmn::Json::parse(read_file(share("mobile_manipulator_control") + "/config/qualified_payload.json"));
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

inline const mmm::Limits & limits()
{
  static const mmm::Limits instance =
    mmm::load_limits(share("mobile_manipulator_navigation") + "/config/nav_operating_envelope.yaml", urdf());
  return instance;
}

inline std::shared_ptr<const mmm::StaticTorqueModel> statics()
{
  static const auto instance = std::make_shared<const mmm::StaticTorqueModel>(urdf(), payload());
  return instance;
}

inline Joints pose(const std::string & name)
{
  Joints q;
  for (int i = 0; i < 6; ++i) q(i) = payload().at("poses_rad").at(name).at(i).get<double>();
  return q;
}

inline mmd::MediumProblem problem(mmd::MediumConfig config = {})
{
  return mmd::MediumProblem(config, limits(), primitives(), mmm::CollisionConfig{}, statics(), pose("carry"));
}

inline mmm::World world_of(const std::vector<mmn::Box> & boxes)
{
  mmm::World world;
  world.set_static(boxes);
  return world;
}

inline mmm::State state(double x, double y, double yaw, const Joints & q)
{
  mmm::State s;
  s << x, y, yaw, q;
  return s;
}

// A straight route along +x from (x0, y) to (x1, y).
inline mmm::Route straight_route(double x0, double x1, double y = 0.0)
{
  return mmm::Route({Eigen::Vector2d(x0, y), Eigen::Vector2d(x1, y)});
}
}  // namespace medium_test
