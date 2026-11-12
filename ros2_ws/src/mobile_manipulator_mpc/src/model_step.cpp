// model_step: B5's prediction model one step at a time, for the rollout tool (E2 plan Task 2, tools/plant/model_rollout.py).
//
// Reads lines of 18 numbers from stdin, "dt x0..x8 u0..u7" (the state and the control in the model's order), or 23 with
// the tracking model's map parameters after them (E2 Task 5: turn_delta_inf turn_delta_zero turn_scale speed_turn
// sideslip), and writes one line of 9 numbers per input line: discrete_step's x_next, printed with %.17g. Blank lines
// are skipped. Exit codes: 0 done, 2 a malformed line (its line number on stderr).
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

int main()
{
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
    // after the 18 required numbers: nothing, or exactly five whole finite numbers (Task 5 review F1: a token whose
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
    if (!ok || !params_ok || !(p.empty() || p.size() == 5) || !(dt >= 0.0)) {
      std::cerr << "model_step: line " << number << ": expected 18 or 23 numbers (dt >= 0, 9 state, 8 control, "
                << "optionally the 5 tracking-map parameters)\n";
      return 2;
    }
    mmm::TrackingModel model;
    if (p.size() == 5) {
      model.turn_delta_inf = p[0];
      model.turn_delta_zero = p[1];
      model.turn_scale = p[2];
      model.speed_turn = p[3];
      model.sideslip = p[4];
    }
    const mmm::State next = mmm::discrete_step(x, u, dt, model).x_next;
    for (int i = 0; i < mmm::kNx; ++i) std::printf(i ? " %.17g" : "%.17g", next(i));
    std::printf("\n");
    std::fflush(stdout);  // a persistent caller reads each answer before it writes the next batch
  }
  return 0;
}
