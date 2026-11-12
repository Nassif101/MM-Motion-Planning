// tracking_replay: the online correction (TrackingEstimator) replayed over a logged run, for the horizon check T11
// (E2 Task 6b review; tools/plant/identify_tracking.py horizon).
//
// Usage: tracking_replay --model FILE < lines. Each line is either "show" (print the current estimate), "reset" (back to
// the prior), or a transition "dt x0 y0 yaw0 x1 y1 yaw1 v w" (observe it, then print the estimate). The estimate is
// printed as "g_inf g_0 k" with %.17g. Exit 2 for a bad model file or line.
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "mobile_manipulator_mpc/tracking_estimator.hpp"

namespace mmm = mobile_manipulator_mpc;

int main(int argc, char ** argv)
{
  if (argc != 3 || std::string(argv[1]) != "--model") {
    std::cerr << "usage: tracking_replay --model FILE < lines\n";
    return 2;
  }
  mmm::TrackingModel prior;
  try {
    prior = mmm::load_tracking_model(argv[2]);
  } catch (const std::exception & e) {
    std::cerr << "tracking_replay: " << e.what() << "\n";
    return 2;
  }
  mmm::TrackingEstimator estimator(prior);
  std::string line;
  long number = 0;
  while (std::getline(std::cin, line)) {
    ++number;
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    if (line.rfind("reset", 0) == 0) {
      estimator.reset();
      continue;
    }
    if (line.rfind("show", 0) != 0) {
      std::istringstream in(line);
      double dt = 0.0, v = 0.0, w = 0.0;
      Eigen::Vector3d before, after;
      if (!(in >> dt >> before(0) >> before(1) >> before(2) >> after(0) >> after(1) >> after(2) >> v >> w)) {
        std::cerr << "tracking_replay: line " << number << ": expected show, reset or 9 numbers\n";
        return 2;
      }
      mmm::Control u = mmm::Control::Zero();
      u(0) = v;
      u(1) = w;
      estimator.observe(before, after, u, dt);
    }
    const Eigen::Vector3d e = estimator.estimate();
    std::printf("%.17g %.17g %.17g\n", e(0), e(1), e(2));
  }
  return 0;
}
