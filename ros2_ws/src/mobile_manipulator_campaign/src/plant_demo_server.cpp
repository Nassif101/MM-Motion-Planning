// The MuJoCo plant's demo server (MuJoCo plant plan Task 9): a session behind one TCP client.
//   plant_demo_server [--port 7700] [--profile FILE] --mode ideal|realistic [--b5-only] [--lidar] [--log FILE]
//                     [--routes recorded|lattice] [--no-prediction]
// Spec 4B-2: B5's Prediction frames are made unless --no-prediction (in the --log file too), and sent only to a client
// whose hello accepts them.
// Spec 2 Task 8: --routes lattice serves a generated-route session; clients send submit_goal / schedule_goal /
// request_replan, and the session runs on while idle. A disconnect pauses the run (the demo's policy).
#include <atomic>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mobile_manipulator_campaign/demo_server.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;

namespace
{
std::atomic<bool> stop{false};
void on_signal(int) { stop = true; }
}  // namespace

int main(int argc, char ** argv)
{
  try {
    mmc::DemoOptions o;
    o.session.prediction = true;  // spec 4B-2: made for viewers that accept it (--no-prediction turns it off)
    std::string mode = "ideal", routes = "recorded";
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc) throw std::invalid_argument(a + " needs a value");
        return argv[++i];
      };
      if (a == "--port") o.port = std::stoi(next());
      else if (a == "--profile") o.session.profile = next();
      else if (a == "--mode") mode = next();
      else if (a == "--b5-only") o.session.medium = false;
      else if (a == "--lidar") o.session.lidar = true;
      else if (a == "--no-prediction") o.session.prediction = false;  // spec 4B-2: on by default, sent to accepting clients
      else if (a == "--log") o.session.log_path = next();
      else if (a == "--routes") routes = next();
      else throw std::invalid_argument("unknown option " + a);
    }
    if (mode != "ideal" && mode != "realistic") throw std::invalid_argument("--mode ideal|realistic");
    o.session.mode = mode == "realistic" ? mmp::PlantMode::Realistic : mmp::PlantMode::Ideal;
    if (routes != "recorded" && routes != "lattice") throw std::invalid_argument("--routes recorded|lattice");
    if (routes == "lattice") {
      o.session.generated_routes = true;
      o.session.auto_goal = false;  // the client chooses the destinations
    }
    o.session.provenance = mmc::git_provenance();
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    mmc::DemoServer server(o);
    std::cout << "plant_demo_server: listening on port " << server.port() << " (" << mode << ")" << std::endl;
    server.serve(stop);
    std::cout << "plant_demo_server: stopped (" << server.dropped() << " frames dropped)" << std::endl;
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "plant_demo_server: " << e.what() << "\n";
    return 1;
  }
}
