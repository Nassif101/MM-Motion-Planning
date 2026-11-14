// tracking_replay: the online correction (TrackingEstimator) replayed over a logged run, for the horizon check T11
// (E2 Task 6b review; tools/plant/identify_tracking.py horizon).
//
// Usage: tracking_replay --model FILE [--frozen-sideslip] < lines (--frozen-sideslip: the turn gain alone is learned,
// as with tracking_adapt_sideslip false; Task 10c). Each line is either "show" (print the current estimate), "reset" (back to
// the prior), or a transition "dt x0 y0 yaw0 x1 y1 yaw1 v w [q1..q6]" (observe it, then print the estimate). The six
// optional numbers are the arm joints at the transition's end (spec Amendment 3: the centre of mass of a model with a
// turning-centre offset; such a model needs them). The estimate is printed as "g_inf g_0 k" with %.17g. Exit 2 for a
// bad model file or line.
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/tracking_estimator.hpp"

namespace mmm = mobile_manipulator_mpc;

int main(int argc, char ** argv)
{
  const bool frozen = argc == 4 && std::string(argv[3]) == "--frozen-sideslip";
  if (!(argc == 3 || frozen) || std::string(argv[1]) != "--model") {
    std::cerr << "usage: tracking_replay --model FILE [--frozen-sideslip] < lines\n";
    return 2;
  }
  mmm::TrackingModel prior;
  try {
    prior = mmm::load_tracking_model(argv[2]);
  } catch (const std::exception & e) {
    std::cerr << "tracking_replay: " << e.what() << "\n";
    return 2;
  }
  mmm::EstimatorConfig config;
  config.estimate_sideslip = !frozen;
  mmm::TrackingEstimator estimator(prior, config);
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
        std::cerr << "tracking_replay: line " << number << ": expected show, reset or 9 or 15 numbers\n";
        return 2;
      }
      mmm::State arm = mmm::State::Zero();
      int joints = 0;
      while (joints < 6 && (in >> arm(3 + joints))) ++joints;
      if (joints != 0 && joints != 6) {
        std::cerr << "tracking_replay: line " << number << ": expected 9 or 15 numbers\n";
        return 2;
      }
      mmm::Control u = mmm::Control::Zero();
      u(0) = v;
      u(1) = w;
      try {
        if (joints == 6)
          estimator.observe(before, after, u, dt, mmm::com_x(arm));
        else
          estimator.observe(before, after, u, dt);
      } catch (const std::logic_error & e) {
        std::cerr << "tracking_replay: line " << number << ": " << e.what() << "\n";
        return 2;
      }
    }
    const Eigen::Vector3d e = estimator.estimate();
    std::printf("%.17g %.17g %.17g\n", e(0), e(1), e(2));
  }
  return 0;
}
