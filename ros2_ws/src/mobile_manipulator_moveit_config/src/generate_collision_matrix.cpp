// Generate the SRDF self-collision block headlessly with the MoveIt Setup Assistant's own
// sampling (computeDefaultCollisions), so the matrix is reproducible without the GUI.
//
// Usage: generate_collision_matrix --urdf FILE --srdf FILE [--trials N]
// Prints one <disable_collisions .../> line per disabled pair. Existing disable entries in
// the given SRDF are ignored so the matrix is always recomputed from geometry.
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_setup_srdf_plugins/compute_default_collisions.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string urdf_path, srdf_path;
  unsigned int trials = 10000;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    if (key == "--urdf") urdf_path = argv[i + 1];
    else if (key == "--srdf") srdf_path = argv[i + 1];
    else if (key == "--trials") trials = static_cast<unsigned int>(std::stoul(argv[i + 1]));
  }
  if (urdf_path.empty() || srdf_path.empty()) {
    std::cerr << "usage: generate_collision_matrix --urdf FILE --srdf FILE [--trials N]\n";
    return 1;
  }
  const auto urdf_model = urdf::parseURDF(read_file(urdf_path));
  const std::string srdf_text = std::regex_replace(
    read_file(srdf_path), std::regex("<disable_collisions[^>]*/>\\s*"), "");
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!urdf_model || !srdf_model->initString(*urdf_model, srdf_text)) {
    std::cerr << "cannot parse URDF/SRDF\n";
    return 1;
  }
  const auto robot = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  const auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  unsigned int progress = 0;
  const auto pairs = moveit_setup::srdf_setup::computeDefaultCollisions(
    scene, &progress, true, trials, 0.95, false);
  for (const auto & [links, data] : pairs) {
    if (!data.disable_check) continue;
    std::cout << "  <disable_collisions link1=\"" << links.first << "\" link2=\"" << links.second
              << "\" reason=\"" << moveit_setup::srdf_setup::disabledReasonToString(data.reason)
              << "\"/>\n";
  }
  return 0;
}
