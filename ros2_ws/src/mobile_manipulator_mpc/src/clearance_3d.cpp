// clearance_3d: the minimum 3D distance between the robot's collision primitives and the known
// boxes over a recorded run (Phase 4 spec, Section 7: scoring of B5, usable for B1-B4).
//
// The primitives are the whole-body MPC's CollisionModel (URDF boxes and cylinders plus the
// qualified payload box), without a margin. The states come from the bag's /tf and /tf_static
// (map -> base_footprint through every parent) and /joint_states; the boxes are the map export
// plus the scenario's obstacles (`known_boxes`, movers excluded). The result is one JSON object on
// stdout (and in --output): the minimum distance in metres (negative: penetration), the robot
// link and the obstacle at the minimum, and the simulation time of that state.
//
// --compare-reference (Phase 4B-A spec Section 5) adds, over the samples that have the JTC's reference
// (/arm_controller/controller_state), the minimum with the measured joints, the clearance with the reference
// joints at that moment, the per-joint lag there and the verdict of judge_shortfall at the 0.05 m margin.
//
// Usage: clearance_3d --bag DIR [--scenario NAME] [--site SITE] [--boxes-file FILE] [--period S] [--output FILE]
//        [--compare-reference]
// Exit codes: 0 measured, 2 the bag gave no robot state or the world no box, 1 bad arguments or
// an unreadable bag.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/clearance.hpp"
#include "mobile_manipulator_mpc/clearance_bag.hpp"
#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/world.hpp"
#include "mobile_manipulator_navigation/known_world.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmm = mobile_manipulator_mpc;
namespace mmn = mobile_manipulator_navigation;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  return std::string(std::istreambuf_iterator<char>(stream), {});
}

std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

double round_to(double value, double scale) { return std::round(value * scale) / scale; }
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const mobile_manipulator_control::Args args(argc, argv);
    if (!args.has("bag")) {
      std::cerr << "usage: clearance_3d --bag DIR [--scenario NAME] [--boxes-file FILE] [--period S] [--output FILE] "
                   "[--compare-reference]"
                << std::endl;
      return 1;
    }
    const std::string navigation = share("mobile_manipulator_navigation");
    const std::string scenario_name = args.get("scenario", "");
    const double period = args.number("period", 0.05);

    const std::string urdf = read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf");
    const mmn::Json payload =
      mmn::Json::parse(read_file(share("mobile_manipulator_control") + "/config/qualified_payload.json"));
    const mmm::CollisionModel model(mmg::load_primitives(urdf, mmg::payload_from_json(payload)),
                                    mmm::CollisionConfig{});
    std::array<std::string, 6> joint_names;
    const auto order = payload.at("joint_order").get<std::vector<std::string>>();
    if (order.size() != 6) throw std::runtime_error("qualified_payload.json joint_order needs six joints");
    std::copy(order.begin(), order.end(), joint_names.begin());

    std::optional<mmn::Json> scenario;
    std::optional<std::string> scenario_site;
    if (!scenario_name.empty()) {
      const auto resolved = mmn::ScenarioConfig(navigation).resolve(scenario_name);
      scenario = resolved.at("scenario");
      scenario_site = resolved.at("site").get<std::string>();
    }
    const std::string site = mmn::effective_site(args.get("site", ""), scenario_site);
    const std::string boxes_file = args.get("boxes-file", mmn::site_boxes_file(navigation, site));
    std::cerr << "site " << site << ", boxes " << boxes_file << std::endl;
    mmm::World world;
    world.set_static(mmn::known_boxes(boxes_file, scenario ? &*scenario : nullptr));
    const auto boxes = world.at(0.0, 0.0);

    const auto samples = mmm::read_clearance_samples(args.get("bag"), joint_names, period);
    const mmm::Clearance result = mmm::minimum_clearance(model, samples, boxes);

    const bool measured = std::isfinite(result.distance_m);
    const auto when = [&](const mmn::Json & value) { return measured ? value : mmn::Json(nullptr); };
    mmn::Json report = {{"min_distance_m", when(round_to(result.distance_m, 1e4))},
                              {"link", when(result.link)},
                              {"obstacle", when(result.obstacle)},
                              {"time_s", when(round_to(result.time_s, 1e3))},
                              {"samples", result.samples},
                              {"boxes", boxes.size()},
                              {"period_s", period},
                              {"scenario", scenario_name.empty() ? mmn::Json(nullptr) : mmn::Json(scenario_name)}};
    if (args.has("compare-reference")) {
      const mmm::ReferenceComparison c = mmm::compare_reference(model, samples, boxes, mmm::CollisionConfig{}.margin_m);
      const bool compared = std::isfinite(c.measured_min);
      const auto known = [&](const mmn::Json & value) { return compared ? value : mmn::Json(nullptr); };
      std::vector<double> lag;
      for (int i = 0; i < 6; ++i) lag.push_back(round_to(c.lag_at_measured_min(i), 1e4));
      report["reference"] = {{"samples", c.samples_with_reference},
                             {"measured_min_m", known(round_to(c.measured_min, 1e4))},
                             {"reference_at_min_m", known(round_to(c.reference_at_measured_min, 1e4))},
                             {"time_s", known(round_to(c.time_s, 1e3))},
                             {"reference_min_m", known(round_to(c.reference_min, 1e4))},
                             {"reference_min_time_s", known(round_to(c.reference_min_time_s, 1e3))},
                             {"lag_rad", known(lag)},
                             {"below_margin", c.verdict.samples_below_margin},
                             {"worst_shortfall_m", round_to(c.verdict.worst_shortfall_m, 1e4)},
                             {"covered_fraction", round_to(c.verdict.covered_fraction, 1e3)},
                             {"explained", c.verdict.explained}};
    }
    // The panel against the arm links (Phase 4B-B), with the installed wholebody_mpc.yaml's collision.self_links.
    // A failure here (an unreadable configuration) leaves the obstacle clearance reported and the self fields null.
    report["self_min_m"] = nullptr;
    report["self_link"] = nullptr;
    try {
      mmm::CollisionConfig self_config;
      self_config.self_links =
        mmm::load_mpc_config(share("mobile_manipulator_mpc") + "/config/wholebody_mpc.yaml").collision.self_links;
      const mmm::CollisionModel self_model(mmg::load_primitives(urdf, mmg::payload_from_json(payload)), self_config);
      const mmm::Clearance self = mmm::minimum_self_clearance(self_model, samples);
      if (std::isfinite(self.distance_m)) {
        report["self_min_m"] = round_to(self.distance_m, 1e4);
        report["self_link"] = self.obstacle;
      }
    } catch (const std::exception & error) {
      std::cerr << "clearance_3d: no arm-link clearance: " << error.what() << std::endl;
    }
    std::cout << report.dump() << std::endl;
    if (args.has("output")) std::ofstream(args.get("output")) << report.dump(2) << "\n";
    return measured ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "clearance_3d: " << error.what() << std::endl;
    return 1;
  }
}
