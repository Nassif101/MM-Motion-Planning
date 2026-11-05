#pragma once
// The benches' common setup (moved from medium_bench.cpp for the MuJoCo plant's runner, plan Task 4): the robot (URDF,
// qualified payload, primitives, limits, qualified poses), a scenario's known world, and medium planners built from
// one configuration. Files come from the installed packages' share directories.
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_medium/planner.hpp"
#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/scene.hpp"
#include "mobile_manipulator_mpc/statics.hpp"
#include "mobile_manipulator_mpc/world.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_medium
{
std::string package_share(const std::string & package);
std::string read_text_file(const std::string & path);

class BenchSetup
{
public:
  BenchSetup(MediumConfig config, const std::string & scenario);

  MediumPlanner planner() const;

  const MediumConfig & config() const { return config_; }
  const mobile_manipulator_mpc::World & world() const { return world_; }
  const mobile_manipulator_mpc::Scene & scene() const { return scene_; }
  const std::string & urdf() const { return urdf_; }
  const mobile_manipulator_navigation::Json & payload() const { return payload_; }
  const std::vector<mobile_manipulator_geometry::Primitive> & primitives() const { return primitives_; }
  const mobile_manipulator_mpc::Limits & limits() const { return limits_; }
  const std::map<std::string, Joints> & poses() const { return poses_; }

private:
  MediumConfig config_;
  std::string urdf_;
  mobile_manipulator_navigation::Json payload_;
  std::vector<mobile_manipulator_geometry::Primitive> primitives_;
  mobile_manipulator_mpc::Limits limits_;
  std::shared_ptr<const mobile_manipulator_mpc::StaticTorque> statics_;
  std::map<std::string, Joints> poses_;
  mobile_manipulator_mpc::Scene scene_;
  mobile_manipulator_mpc::World world_;
};
}  // namespace mobile_manipulator_medium
