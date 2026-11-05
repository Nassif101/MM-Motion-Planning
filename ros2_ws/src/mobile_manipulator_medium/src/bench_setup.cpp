#include "mobile_manipulator_medium/bench_setup.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_medium/dense_qp.hpp"

namespace mobile_manipulator_medium
{
namespace mmm = mobile_manipulator_mpc;

std::string package_share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

std::string read_text_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

BenchSetup::BenchSetup(MediumConfig config, const std::string & scenario) : config_(std::move(config))
{
  urdf_ = read_text_file(package_share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf");
  payload_ = mobile_manipulator_navigation::Json::parse(
    read_text_file(package_share("mobile_manipulator_control") + "/config/qualified_payload.json"));
  primitives_ = mobile_manipulator_geometry::load_primitives(urdf_, mobile_manipulator_geometry::payload_from_json(payload_));
  limits_ = mmm::load_limits(package_share("mobile_manipulator_navigation") + "/config/nav_operating_envelope.yaml", urdf_);
  statics_ = std::make_shared<const mmm::GeneratedStaticTorque>();
  for (const auto & item : payload_.at("poses_rad").items()) {
    Joints q;
    for (int i = 0; i < 6; ++i) q(i) = item.value().at(i).get<double>();
    poses_[item.key()] = q;
  }
  scene_ = mmm::load_scene(package_share("mobile_manipulator_navigation"), "", scenario, "");
  world_.set_static(scene_.boxes);
}

MediumPlanner BenchSetup::planner() const
{
  MediumProblem problem(config_, limits_, primitives_, mmm::CollisionConfig{}, statics_, poses_.at("carry"));
  return MediumPlanner(std::move(problem), make_dense_solver(config_.qp_backend, DenseQpOptions{config_.qp_tolerance}),
                       poses_, primitives_.back().dims);
}
}  // namespace mobile_manipulator_medium
