// model_step: B5's prediction model one step at a time, for the rollout tool (E2 plan Task 2, tools/plant/model_rollout.py).
//
// Reads lines of 18 numbers from stdin, "dt x0..x8 u0..u7" (the state and the control in the model's order), and writes
// one line of 9 numbers per input line: discrete_step(x, u, dt).x_next, printed with %.17g. Blank lines are skipped.
// Exit codes: 0 done, 2 a malformed line (its line number on stderr).
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "mobile_manipulator_mpc/model.hpp"

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
    std::string extra;
    if (!ok || (in >> extra) || !(dt >= 0.0)) {
      std::cerr << "model_step: line " << number << ": expected 18 numbers (dt >= 0, 9 state, 8 control)\n";
      return 2;
    }
    const mmm::State next = mmm::discrete_step(x, u, dt).x_next;
    for (int i = 0; i < mmm::kNx; ++i) std::printf(i ? " %.17g" : "%.17g", next(i));
    std::printf("\n");
    std::fflush(stdout);  // a persistent caller reads each answer before it writes the next batch
  }
  return 0;
}
