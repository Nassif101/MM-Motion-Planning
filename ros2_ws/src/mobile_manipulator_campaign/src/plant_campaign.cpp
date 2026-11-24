// One campaign run on the MuJoCo plant (MuJoCo plant plan Task 8): a cell through a Session to its end (every recorded
// goal in order with a bag, else the scenario's start and goal), DIR/summary.json and DIR/run.mmlog.
//   plant_campaign --scenario NAME [--bag DIR] --mode ideal|realistic [--b5-only] [--medium-config FILE] [--seed N]
//                  [--lidar] [--profile FILE] [--timeout-s T] [--profile-timing] [--stall-s 45] [--cell NAME] --out DIR
//                  [--routes recorded|lattice] [--goals FILE] [--sim-time-limit S] [--prediction] [--seed-detail]
//                  [--fork-at T --fork-set KEY=VALUE ... [--fork-cycles N]]
// Spec 4B-2: --prediction adds B5's Prediction frames to the run log (off by default; the run is unchanged).
//   plant_campaign --print-routing-fingerprint (the routing inputs' fingerprint of this build, for resume)
// Spec 2 Task 8: --routes lattice plans the routes with the Smac Lattice port (no bag): the scenario's destination, or
// the goal script's goals (--goals, goal_script.hpp); the run ends by the script's policy or --sim-time-limit.
#include <cmath>
#include <filesystem>
#include <optional>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mobile_manipulator_campaign/goal_script.hpp"
#include "mobile_manipulator_campaign/session.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;

int main(int argc, char ** argv)
{
  try {
    mmc::SessionConfig config;
    config.stall_s = 45.0;  // campaigns: a run without progress for 45 s ends as stalled (user, 2026-11-07); 0: off
    std::string scenario, bag, out, mode, routes = "recorded", goals;
    std::optional<double> sim_time_limit;
    uint64_t seed = 1;
    std::vector<std::string> args;  // the run's arguments but --out, into its summary
    // plant_campaign [--b5-config FILE] --print-config-tracking-model: the tracking model the B5 config names, as B5's
    // loader resolves it ("" for none), then exit (E2 final review C3: the ensemble scripts decide from it)
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--print-config-tracking-model") {
        std::string b5;
        for (int j = 1; j + 1 < argc; ++j)
          if (std::string(argv[j]) == "--b5-config") b5 = argv[j + 1];
        std::cout << mmc::config_tracking_model(b5) << "\n";
        return 0;
      }
    // plant_campaign --print-routing-fingerprint: the routing inputs' fingerprint as a lattice run of this build would
    // record it (its summary's `routing`), then exit; resume compares the two (Task 9 review I5, I6)
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--print-routing-fingerprint") {
        mmc::SessionConfig probe;
        probe.generated_routes = true;
        probe.auto_goal = false;
        mmc::Session session(probe);
        session.reset("v2_open_yard_nav", "", std::nullopt, 1);
        std::cout << session.summary().at("routing").dump() << "\n";
        return 0;
      }
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--out") ++i;
      else args.push_back(argv[i]);
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc) throw std::invalid_argument(a + " needs a value");
        return argv[++i];
      };
      if (a == "--scenario") scenario = next();
      else if (a == "--bag") bag = next();
      else if (a == "--mode") mode = next();
      else if (a == "--b5-only") config.medium = false;
      else if (a == "--medium-config") config.medium_config = next();
      else if (a == "--b5-config") config.b5_config = next();
      else if (a == "--start-nudge") config.start_nudge_rad = std::stod(next());
      else if (a == "--diagnostics") config.diagnostics = true;
      else if (a == "--row-trace") config.row_trace = true;  // E3 Task 1
      else if (a == "--prediction") config.prediction = true;  // spec 4B-2: B5's solved plan in the run log
      else if (a == "--seed-detail") config.seed_detail = true;  // medium diagnosis Task 1: the richer seed rows
      else if (a == "--fork-at") config.fork.at = std::stod(next());  // medium diagnosis Task 4: the fork
      else if (a == "--fork-set") {
        const std::string kv = next();
        const auto eq = kv.find('=');
        if (eq == std::string::npos || eq == 0) throw std::invalid_argument("--fork-set key=value");
        config.fork.set[kv.substr(0, eq)] = kv.substr(eq + 1);
      } else if (a == "--fork-cycles") config.fork.cycles = std::stoi(next());
      else if (a == "--tracking-model") config.tracking_model = next();  // E2 Task 7: a file, or none
      else if (a == "--tracking-adaptation") {
        const std::string v = next();
        if (v != "on" && v != "off" && v != "gain") throw std::invalid_argument("--tracking-adaptation on|off|gain");
        config.tracking_adaptation = v == "on" ? 1 : v == "gain" ? 2 : 0;
      }
      else if (a == "--seed") seed = std::stoull(next());
      else if (a == "--lidar") config.lidar = true;
      else if (a == "--profile") config.profile = next();
      else if (a == "--timeout-s") config.timeout_s = std::stod(next());
      else if (a == "--profile-timing") config.profile_timing = true;
      else if (a == "--stall-s") config.stall_s = std::stod(next());
      else if (a == "--cell") config.cell = next();
      else if (a == "--out") out = next();
      else if (a == "--routes") routes = next();
      else if (a == "--goals") goals = next();
      else if (a == "--sim-time-limit") sim_time_limit = std::stod(next());
      else throw std::invalid_argument("unknown option " + a);
    }
    if (scenario.empty() || out.empty() || (mode != "ideal" && mode != "realistic"))
      throw std::invalid_argument("--scenario, --mode ideal|realistic and --out are required");
    config.mode = mode == "realistic" ? mmp::PlantMode::Realistic : mmp::PlantMode::Ideal;
    if (routes != "recorded" && routes != "lattice") throw std::invalid_argument("--routes recorded|lattice");
    const bool lattice = routes == "lattice";
    if (lattice && !bag.empty()) throw std::invalid_argument("--routes lattice plans its own routes: no --bag");
    if (!lattice && (!goals.empty() || sim_time_limit)) throw std::invalid_argument("--goals and --sim-time-limit need --routes lattice");
    mmc::GoalScript script;
    if (lattice) {
      if (!goals.empty()) script = mmc::load_goal_script(goals);
      if (sim_time_limit) {
        if (!std::isfinite(*sim_time_limit) || *sim_time_limit <= 0.0) throw std::invalid_argument("--sim-time-limit must be positive");
        script.time_limit_s = script.time_limit_s ? std::min(*script.time_limit_s, *sim_time_limit) : *sim_time_limit;
      }
      config.generated_routes = true;
      config.auto_goal = goals.empty();  // without a script: the scenario's destination, submitted at reset
    }
    config.provenance = mmc::git_provenance();
    std::filesystem::create_directories(out);
    config.log_path = out + "/run.mmlog";
    nlohmann::json summary;
    {
      mmc::Session session(config);
      session.reset(scenario, bag, std::nullopt, seed);
      if (lattice) {
        const auto run = mmc::run_goal_script(session, script);
        summary = session.summary();
        summary["goal_script"] = mmc::to_json(run);
        // Task 8 review I4: a scripted run's success is the script's (every goal reached or superseded, the last one
        // reached); `reached` keeps the session's current-destination meaning
        if (!goals.empty()) summary["script_success"] = run.success;
        summary["goal_script_sha256"] = goals.empty() ? nlohmann::json(nullptr) : nlohmann::json(script.sha256);
      } else {
        while (session.step_cycle()) {
        }
        summary = session.summary();
      }
      summary["routes"] = routes;
    }  // the run log is closed before the summary marks the run done
    summary["args"] = args;  // what tools/plant/plant_campaign.py's resume compares
    const std::string done = out + "/summary.json", partial = done + ".partial";
    {
      std::ofstream file(partial);
      file << summary.dump(1) << "\n";
      if (!file.flush()) throw std::runtime_error("cannot write " + partial);
    }
    std::filesystem::rename(partial, done);  // atomic: a summary is whole or absent
    std::cout << scenario << " seed " << seed << ": " << summary.at("end_status").get<std::string>() << " after "
              << summary.at("time_s").get<double>() << " s (" << summary.at("real_time_factor").get<double>() << "x real time)\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "plant_campaign: " << e.what() << "\n";
    return 1;
  }
}
