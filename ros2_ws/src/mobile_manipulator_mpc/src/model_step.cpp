// model_step: B5's prediction model one step at a time, for the rollout tool (E2 plan Task 2, tools/plant/model_rollout.py).
//
// Reads lines of 18 numbers from stdin, "dt x0..x8 u0..u7" (the state and the control in the model's order), or 23 with
// the tracking model's map parameters after them (E2 Task 5: turn_delta_inf turn_delta_zero turn_scale speed_turn
// sideslip), or 25 with spec Amendment 3's turn_offset turn_offset_com after those, and writes one line of 9 numbers per input line: discrete_step's x_next, printed with %.17g. Blank lines
// are skipped. Exit codes: 0 done, 2 a malformed line (its line number on stderr).
// model_step --check-model FILE loads a tracking model file with B5's loader and prints it (exit 2 naming the problem).
// model_step --com-x reads lines of 6 arm joints and writes the whole robot's centre of mass x in base_footprint for
// each (spec Amendment 3; exit 2 for a malformed line).
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>

#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/tracking_model.hpp"

namespace mmm = mobile_manipulator_mpc;

int main(int argc, char ** argv)
{
  // model_step --check-model FILE: load a tracking model file with B5's own loader (Task 6 review F3); prints its
  // parameters and exits 0, or names the problem on stderr and exits 2
  if (argc == 3 && std::string(argv[1]) == "--check-model") {
    try {
      const mmm::TrackingModel m = mmm::load_tracking_model(argv[2]);
      std::printf("turn_delta_inf %.17g turn_delta_zero %.17g turn_scale %.17g speed_turn %.17g sideslip %.17g "
                  "base_delay_s %.17g arm_delay_s %.17g turn_offset %.17g turn_offset_com %.17g\n",
                  m.turn_delta_inf, m.turn_delta_zero, m.turn_scale, m.speed_turn, m.sideslip, m.base_delay_s,
                  m.arm_delay_s, m.turn_offset, m.turn_offset_com);
      return 0;
    } catch (const std::exception & e) {
      std::cerr << "model_step: " << e.what() << "\n";
      return 2;
    }
  }
  if (argc == 2 && std::string(argv[1]) == "--com-x") {
    std::string line;
    long number = 0;
    while (std::getline(std::cin, line)) {
      ++number;
      if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
      std::istringstream in(line);
      mmm::State x = mmm::State::Zero();
      bool ok = true;
      for (int i = 0; ok && i < 6; ++i) ok = static_cast<bool>(in >> x(3 + i)) && std::isfinite(x(3 + i));
      std::string extra;
      if (!ok || (in >> extra)) {
        std::cerr << "model_step: line " << number << ": expected 6 arm joints\n";
        return 2;
      }
      std::printf("%.17g\n", mmm::com_x(x));
    }
    return 0;
  }
  if (argc != 1) {
    std::cerr << "usage: model_step < lines, model_step --check-model FILE, or model_step --com-x < lines\n";
    return 2;
  }
  std::string line;
  long number = 0;
  while (std::getline(std::cin, line)) {
    ++number;
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    std::istringstream in(line);
    double dt = 0.0;
    mmm::State x;
    mmm::Control u;
    bool ok = static_cast<bool>(in >> dt);
    for (int i = 0; ok && i < mmm::kNx; ++i) ok = static_cast<bool>(in >> x(i));
    for (int i = 0; ok && i < mmm::kNu; ++i) ok = static_cast<bool>(in >> u(i));
    // after the 18 required numbers: nothing, or exactly five or seven whole finite numbers (Task 5 review F1: a token whose
    // conversion fails must not be swallowed into an identity request)
    std::vector<double> p;
    std::string token;
    bool params_ok = true;
    while (params_ok && (in >> token)) {
      errno = 0;
      char * end = nullptr;
      const double value = std::strtod(token.c_str(), &end);
      params_ok = end != token.c_str() && *end == '\0' && errno != ERANGE && std::isfinite(value);
      p.push_back(value);
    }
    if (!ok || !params_ok || !(p.empty() || p.size() == 5 || p.size() == 7) || !(dt >= 0.0)) {
      std::cerr << "model_step: line " << number << ": expected 18, 23 or 25 numbers (dt >= 0, 9 state, 8 control, "
                << "optionally the 5 or 7 tracking-map parameters)\n";
      return 2;
    }
    mmm::TrackingModel model;
    if (p.size() >= 5) {
      model.turn_delta_inf = p[0];
      model.turn_delta_zero = p[1];
      model.turn_scale = p[2];
      model.speed_turn = p[3];
      model.sideslip = p[4];
    }
    if (p.size() == 7) {
      model.turn_offset = p[5];
      model.turn_offset_com = p[6];
    }
    const mmm::State next = mmm::discrete_step(x, u, dt, model).x_next;
    for (int i = 0; i < mmm::kNx; ++i) std::printf(i ? " %.17g" : "%.17g", next(i));
    std::printf("\n");
    std::fflush(stdout);  // a persistent caller reads each answer before it writes the next batch
  }
  return 0;
}
