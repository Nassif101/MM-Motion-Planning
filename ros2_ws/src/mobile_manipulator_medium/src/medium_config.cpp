#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

#include "mobile_manipulator_medium/problem.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_medium
{
namespace
{
namespace mmn = mobile_manipulator_navigation;

// Every key of `node` must be one of `known` (a misspelt key would silently keep its default).
void only(const mmn::Json & node, const std::set<std::string> & known, const std::string & where)
{
  if (!node.is_object()) throw std::runtime_error(where + ": expected a mapping");
  for (const auto & item : node.items())
    if (!known.count(item.key())) throw std::runtime_error(where + ": unknown key " + item.key());
}

double number(const mmn::Json & node, const char * key, double fallback, const std::string & where)
{
  if (!node.contains(key)) return fallback;
  const auto & v = node.at(key);
  if (!v.is_number()) throw std::runtime_error(where + ": " + key + " must be a number");
  return v.get<double>();
}

bool boolean(const mmn::Json & node, const char * key, bool fallback, const std::string & where)
{
  if (!node.contains(key)) return fallback;
  const auto & v = node.at(key);
  if (!v.is_boolean()) throw std::runtime_error(where + ": " + key + " must be true or false");
  return v.get<bool>();
}

int integer(const mmn::Json & node, const char * key, int fallback, const std::string & where)
{
  if (!node.contains(key)) return fallback;
  const auto & v = node.at(key);
  if (!v.is_number_integer()) throw std::runtime_error(where + ": " + key + " must be an integer");
  return v.get<int>();
}

std::vector<std::string> strings(const mmn::Json & node, const char * key, std::vector<std::string> fallback,
                                 const std::string & where)
{
  if (!node.contains(key)) return fallback;
  const auto & v = node.at(key);
  if (!v.is_array()) throw std::runtime_error(where + ": " + key + " must be a list");
  std::vector<std::string> out;
  for (const auto & item : v) {
    if (!item.is_string()) throw std::runtime_error(where + ": " + key + " entries must be strings");
    out.push_back(item.get<std::string>());
  }
  return out;
}
}  // namespace

MediumConfig load_medium_config(const std::string & yaml_path)
{
  const mmn::Json root = mmn::load_yaml_file(yaml_path);
  const std::string where = yaml_path;
  MediumConfig c;
  if (root.is_null()) return c;
  only(root, {"rate_hz", "bezier", "arm_joints", "collision_dt", "torque_dt", "margins", "contact_points",
              "contact_band_m", "clearance_target_m", "speed_fraction", "tight_speed_scale", "weights", "scp", "slack_feasible_m", "homotopy_hysteresis", "solve_cap_s",
              "full_multistart_period_s", "seeds_per_cycle", "sample_ds", "sample_dyaw", "torque_limits_nm", "seeds", "qp_backend",
              "qp_tolerance"},
       where);
  c.rate_hz = number(root, "rate_hz", c.rate_hz, where);
  if (root.contains("bezier")) {
    const auto & b = root.at("bezier");
    only(b, {"degree", "segments", "horizon_s", "continuity"}, where + ": bezier");
    c.bezier.degree = integer(b, "degree", c.bezier.degree, where);
    c.bezier.segments = integer(b, "segments", c.bezier.segments, where);
    c.bezier.horizon_s = number(b, "horizon_s", c.bezier.horizon_s, where);
    c.bezier.continuity = integer(b, "continuity", c.bezier.continuity, where);
  }
  c.arm_joints = strings(root, "arm_joints", c.arm_joints, where);
  c.collision_dt = number(root, "collision_dt", c.collision_dt, where);
  c.torque_dt = number(root, "torque_dt", c.torque_dt, where);
  if (root.contains("margins")) {
    const auto & m = root.at("margins");
    only(m, {"base", "arm"}, where + ": margins");
    c.base_margin_m = number(m, "base", c.base_margin_m, where);
    c.arm_margin_m = number(m, "arm", c.arm_margin_m, where);
  }
  c.contact_points = boolean(root, "contact_points", c.contact_points, where);
  c.contact_band_m = number(root, "contact_band_m", c.contact_band_m, where);
  c.clearance_target_m = number(root, "clearance_target_m", c.clearance_target_m, where);
  c.speed_fraction = number(root, "speed_fraction", c.speed_fraction, where);
  c.tight_speed_scale = number(root, "tight_speed_scale", c.tight_speed_scale, where);
  if (root.contains("weights")) {
    const auto & w = root.at("weights");
    only(w, {"progress", "contour", "goal_position", "goal_heading", "clearance", "comfort", "smooth_base", "smooth_arm", "skid",
             "slack", "slack_quadratic"},
         where + ": weights");
    auto & o = c.weights;
    o.progress = number(w, "progress", o.progress, where);
    o.contour = number(w, "contour", o.contour, where);
    o.goal_position = number(w, "goal_position", o.goal_position, where);
    o.goal_heading = number(w, "goal_heading", o.goal_heading, where);
    o.clearance = number(w, "clearance", o.clearance, where);
    o.comfort = number(w, "comfort", o.comfort, where);
    o.smooth_base = number(w, "smooth_base", o.smooth_base, where);
    o.smooth_arm = number(w, "smooth_arm", o.smooth_arm, where);
    o.skid = number(w, "skid", o.skid, where);
    o.slack = number(w, "slack", o.slack, where);
    o.slack_quadratic = number(w, "slack_quadratic", o.slack_quadratic, where);
  }
  if (root.contains("scp")) {
    const auto & s = root.at("scp");
    only(s, {"max_iter", "trust_init", "trust_min", "trust_max", "step_tol", "merit_tol"}, where + ": scp");
    auto & o = c.scp;
    o.max_iter = integer(s, "max_iter", o.max_iter, where);
    o.trust_init = number(s, "trust_init", o.trust_init, where);
    o.trust_min = number(s, "trust_min", o.trust_min, where);
    o.trust_max = number(s, "trust_max", o.trust_max, where);
    o.step_tol = number(s, "step_tol", o.step_tol, where);
    o.merit_tol = number(s, "merit_tol", o.merit_tol, where);
  }
  c.slack_feasible_m = number(root, "slack_feasible_m", c.slack_feasible_m, where);
  c.homotopy_hysteresis = number(root, "homotopy_hysteresis", c.homotopy_hysteresis, where);
  c.solve_cap_s = number(root, "solve_cap_s", c.solve_cap_s, where);
  c.full_multistart_period_s = number(root, "full_multistart_period_s", c.full_multistart_period_s, where);
  c.seeds_per_cycle = integer(root, "seeds_per_cycle", c.seeds_per_cycle, where);
  if (c.seeds_per_cycle < 0) throw std::runtime_error(where + ": seeds_per_cycle must not be negative");
  c.sample_ds = number(root, "sample_ds", c.sample_ds, where);
  c.sample_dyaw = number(root, "sample_dyaw", c.sample_dyaw, where);
  if (root.contains("torque_limits_nm")) {
    const auto & t = root.at("torque_limits_nm");
    if (!t.is_array() || t.size() != 6) throw std::runtime_error(where + ": torque_limits_nm must list six values");
    for (int j = 0; j < 6; ++j) {
      if (t[j].is_null()) c.torque_limits_nm(j) = std::numeric_limits<double>::infinity();
      else if (t[j].is_number() && t[j].get<double>() > 0.0) c.torque_limits_nm(j) = t[j].get<double>();
      else throw std::runtime_error(where + ": torque_limits_nm entries must be positive numbers or null");
    }
  }
  c.seeds = strings(root, "seeds", c.seeds, where);
  if (root.contains("qp_backend")) {
    if (!root.at("qp_backend").is_string()) throw std::runtime_error(where + ": qp_backend must be a string");
    c.qp_backend = root.at("qp_backend").get<std::string>();
  }
  c.qp_tolerance = number(root, "qp_tolerance", c.qp_tolerance, where);
  if (!(c.rate_hz > 0.0) || !(c.collision_dt > 0.0) || !(c.torque_dt > 0.0) || !(c.solve_cap_s > 0.0) ||
      !(c.sample_ds > 0.0) || !(c.sample_dyaw > 0.0) || !(c.qp_tolerance > 0.0))
    throw std::runtime_error(where + ": rates, sample times, the solve cap and the QP tolerance must be positive");
  if (!(c.contact_band_m >= 0.0)) throw std::runtime_error(where + ": contact_band_m must not be negative");
  // A sample time that does not divide the horizon (or the torque interval) would shorten the last interval silently.
  auto multiple = [](double a, double b) { return std::abs(a / b - std::round(a / b)) < 1e-9 && std::round(a / b) >= 1; };
  if (!multiple(c.bezier.horizon_s, c.collision_dt) || !multiple(c.torque_dt, c.collision_dt))
    throw std::runtime_error(where + ": bezier.horizon_s and torque_dt must be whole multiples of collision_dt");
  if (!(c.speed_fraction > 0.0 && c.speed_fraction <= 1.0))
    throw std::runtime_error(where + ": speed_fraction must be in (0, 1]");
  if (!(c.tight_speed_scale > 0.0 && c.tight_speed_scale <= 1.0))
    throw std::runtime_error(where + ": tight_speed_scale must be in (0, 1]");
  const auto & s = c.scp;
  if (!(s.trust_min > 0.0 && s.trust_min <= s.trust_init && s.trust_init <= s.trust_max) || s.max_iter < 1 ||
      !(s.step_tol > 0.0) || !(s.merit_tol > 0.0))
    throw std::runtime_error(where + ": scp needs 0 < trust_min <= trust_init <= trust_max, max_iter >= 1 and positive "
                                     "tolerances");
  const auto & w = c.weights;
  for (const double v : {w.progress, w.contour, w.goal_position, w.goal_heading, w.clearance, w.comfort, w.smooth_base, w.smooth_arm,
                         w.skid, w.slack, w.slack_quadratic})
    if (!(v >= 0.0) || !std::isfinite(v)) throw std::runtime_error(where + ": weights must be finite and >= 0");
  if (c.qp_backend != "hpipm" && c.qp_backend != "osqp" && c.qp_backend != "daqp")
    throw std::runtime_error(where + ": qp_backend must be hpipm, osqp or daqp, not " + c.qp_backend);
  return c;
}
}  // namespace mobile_manipulator_medium
