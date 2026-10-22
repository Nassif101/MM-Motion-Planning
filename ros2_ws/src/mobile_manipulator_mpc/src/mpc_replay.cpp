// mpc_replay: the Phase 4B-B replay bench (spec Section 3). Replays one recorded B5 run (its bag and summary.json)
// through a controller built from a wholebody_mpc.yaml variant and writes timing, solver and plan-quality statistics.
//
// The replay is open-loop: the robot follows the recorded states. Statistics of timing are the median over the
// repeats of each repeat's statistic; everything else comes from the first repeat (it does not depend on time).
//
// Usage: mpc_replay --run DIR --config YAML --output FILE [--rate-hz R] [--recorded-misses] [--repeats N]
//        [--variant NAME] [--clearance-stages K]
// Exit codes: 0 written, 1 bad arguments or an unreadable run, 2 a run the bench cannot replay (B5 options given
// on the command line, no cycles).
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "mobile_manipulator_mpc/replay_bag.hpp"
#include "mobile_manipulator_mpc/scene.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmm = mobile_manipulator_mpc;
namespace mmn = mobile_manipulator_navigation;
namespace fs = std::filesystem;

namespace
{
std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

double round_to(double value, double step) { return std::isfinite(value) ? std::round(value / step) * step : value; }

// Nearest-rank percentile (q in [0, 1]) of the finite values; NaN without any.
double percentile(std::vector<double> values, double q)
{
  values.erase(std::remove_if(values.begin(), values.end(), [](double v) { return !std::isfinite(v); }), values.end());
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const size_t rank = static_cast<size_t>(std::ceil(q * static_cast<double>(values.size())));
  return values[std::min(values.size() - 1, rank == 0 ? 0 : rank - 1)];
}

double mean(const std::vector<double> & values)
{
  double sum = 0.0;
  size_t n = 0;
  for (double v : values) {
    if (!std::isfinite(v)) continue;
    sum += v;
    ++n;
  }
  return n ? sum / static_cast<double>(n) : std::numeric_limits<double>::quiet_NaN();
}

mmn::Json json_number(double value) { return std::isfinite(value) ? mmn::Json(value) : mmn::Json(nullptr); }

mmn::Json quantiles(const std::vector<double> & values)
{
  return {{"p50", json_number(percentile(values, 0.50))}, {"p95", json_number(percentile(values, 0.95))},
          {"p99", json_number(percentile(values, 0.99))}, {"max", json_number(percentile(values, 1.0))}};
}

mmn::Json rounded(const std::vector<double> & values, double step)
{
  mmn::Json out = mmn::Json::array();
  for (double v : values) out.push_back(json_number(round_to(v, step)));
  return out;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const mobile_manipulator_control::Args args(argc, argv);
    if (!args.has("run") || !args.has("config") || !args.has("output")) {
      std::cerr << "usage: mpc_replay --run DIR --config YAML --output FILE [--rate-hz R] [--recorded-misses] "
                   "[--repeats N] [--variant NAME] [--clearance-stages K]"
                << std::endl;
      return 1;
    }
    const fs::path run = args.get("run");
    const mmn::Json summary = mmn::Json::parse(read_file((run / "summary.json").string()));
    const auto overrides = summary.value("mpc_overrides", mmn::Json());
    if (overrides.is_object() && !overrides.empty()) {
      std::cerr << run << ": recorded with B5 options on the command line (" << overrides.dump()
                << "); the bench replays runs on a configuration file only" << std::endl;
      return 2;
    }
    // summary.json holds the scenario as its specification (with its name); older runs may hold the name only.
    const mmn::Json scenario_entry = summary.value("scenario", mmn::Json());
    const std::string scenario_name = scenario_entry.is_object() ? scenario_entry.value("name", std::string())
                                      : scenario_entry.is_string() ? scenario_entry.get<std::string>() : std::string();
    const std::string references = summary.value("references", std::string("none"));
    const std::string site = summary.value("site", mmn::Json(std::string())).is_string()
                               ? summary.value("site", std::string())
                               : std::string();

    const std::string navigation = share("mobile_manipulator_navigation");
    const std::string urdf = read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf");
    const mmn::Json payload =
      mmn::Json::parse(read_file(share("mobile_manipulator_control") + "/config/qualified_payload.json"));
    mmm::require_model_joint_order(payload);
    const mmm::Limits limits = mmm::load_limits(navigation + "/config/nav_operating_envelope.yaml", urdf);
    const mmm::MpcConfig config = mmm::load_mpc_config(args.get("config"));

    std::array<std::string, 6> joint_names;
    std::copy(mmm::kArmJoints.begin(), mmm::kArmJoints.end(), joint_names.begin());
    const mmm::Recording recording = mmm::read_recording((run / "bag").string(), joint_names);
    if (recording.cycles.empty() || !(recording.period_s > 0.0)) {
      std::cerr << run << ": no MPC cycles in the bag" << std::endl;
      return 2;
    }

    const mmm::Scene scene = mmm::load_scene(navigation, site, scenario_name, "");
    mmm::ReplaySetup setup;
    setup.world.set_static(scene.boxes);
    setup.movers = scene.movers;
    setup.posture = mmm::posture_targets(payload, config.comfort_pose, config.posture_mode, config.tight_poses);
    setup.limits = limits;
    if (references == "scripted") {
      if (!scene.scenario) throw std::runtime_error("references scripted without a scenario");
      const mmg::FootprintProjector projector(urdf, mmg::payload_from_json(payload));
      const auto anchored = mmm::mission_references(*scene.scenario, projector, payload);
      setup.references = [anchored](const mmm::Route & route) { return mmm::anchor_on_remaining_route(anchored, route); };
    }

    mmm::ReplayOptions options;
    options.rate_hz = args.number("rate-hz", 0.0);
    options.recorded_misses = args.has("recorded-misses");
    options.clearance_stages =
      static_cast<int>(args.number("clearance-stages", std::ceil(config.slack_gate.horizon_s / config.dt - 1e-9)));
    const double rate = options.rate_hz > 0.0 ? options.rate_hz : 1.0 / recording.period_s;
    const int repeats = std::max(1, static_cast<int>(args.number("repeats", 3)));

    std::vector<mmm::ReplayResult> results;
    for (int r = 0; r < repeats; ++r) {
      auto controller = mmm::build_controller(config, limits, urdf, payload, rate, true);
      results.push_back(mmm::replay(*controller, recording, setup, options));
    }
    const mmm::ReplayResult & first = results.front();
    // Timing: each statistic is the median over the repeats.
    const auto timing = [&](std::vector<double> mmm::ReplayResult::*field) {
      std::vector<mmn::Json> per;
      mmn::Json out;
      for (const char * q : {"p50", "p95", "p99", "max"}) {
        std::vector<double> values;
        for (const auto & r : results) {
          const mmn::Json s = quantiles(r.*field);
          if (s.at(q).is_number()) values.push_back(s.at(q).get<double>());
        }
        out[q] = json_number(percentile(values, 0.5));
      }
      return out;
    };
    std::vector<double> iterations(first.iterations.begin(), first.iterations.end());
    mmn::Json statuses = mmn::Json::object();
    for (const auto & s : first.status) statuses[s] = statuses.value(s, 0) + 1;
    const int failures = statuses.value("fallback", 0);
    const std::string cell = scenario_name + (references == "scripted" ? "-ref" : "");
    const std::string variant = args.get("variant", fs::path(args.get("config")).stem().string());

    mmn::Json report = {
      {"run", run.filename().string()}, {"cell", cell}, {"variant", variant}, {"rate_hz", rate},
      {"recorded_rate_hz", 1.0 / recording.period_s}, {"cycles", first.stamp.size()}, {"repeats", repeats},
      {"timing",
       {{"total_s", timing(&mmm::ReplayResult::total_s)}, {"solve_s", timing(&mmm::ReplayResult::solve_s)},
        {"collision_s", timing(&mmm::ReplayResult::collision_s)}, {"prep_s", timing(&mmm::ReplayResult::prep_s)},
        {"feedback_s", timing(&mmm::ReplayResult::feedback_s)}}},
      {"iterations", {{"p50", json_number(percentile(iterations, 0.5))}, {"p99", json_number(percentile(iterations, 0.99))},
                      {"max", json_number(percentile(iterations, 1.0))}}},
      {"statuses", statuses}, {"solver_failures", failures},
      {"goal_starts", first.goal_starts}, {"goal_updates", first.goal_updates},
      {"true_clearance_min_m", json_number(percentile(first.true_clearance_m, 0.0))},
      {"true_clearance_p01_m", json_number(percentile(first.true_clearance_m, 0.01))},
      {"lin_error_p95_m", json_number(percentile(first.lin_error_m, 0.95))},
      {"progress_median_m", json_number(percentile(first.progress_m, 0.5))},
      {"contour_mean_m", json_number(mean(first.contour_m))},
      {"per_cycle",
       {{"stamp", rounded(first.stamp, 1e-4)}, {"total_s", rounded(first.total_s, 1e-6)},
        {"solve_s", rounded(first.solve_s, 1e-6)}, {"iterations", first.iterations}, {"status", first.status},
        {"true_clearance_m", rounded(first.true_clearance_m, 1e-5)}, {"lin_error_m", rounded(first.lin_error_m, 1e-5)},
        {"progress_m", rounded(first.progress_m, 1e-4)}, {"contour_m", rounded(first.contour_m, 1e-4)}}}};
    if (first.fidelity) {
      report["fidelity"] = {{"status_match", first.fidelity->status_match},
                            {"stage1_match", first.fidelity->stage1_match},
                            {"iterations_match", first.fidelity->iterations_match},
                            {"compared", first.fidelity->compared}};
    }
    std::ofstream(args.get("output")) << report.dump() << "\n";
    std::cerr << run.filename().string() << " [" << variant << "]: " << first.stamp.size() << " cycles";
    if (first.fidelity) {
      std::cerr << ", fidelity status " << first.fidelity->status_match << " stage1 " << first.fidelity->stage1_match
                << " iterations " << first.fidelity->iterations_match;
    }
    std::cerr << std::endl;
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "mpc_replay: " << e.what() << std::endl;
    return 1;
  }
}
