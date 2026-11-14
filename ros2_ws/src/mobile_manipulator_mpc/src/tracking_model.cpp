#include "mobile_manipulator_mpc/tracking_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_mpc
{
namespace mmn = mobile_manipulator_navigation;

bool TrackingModel::map_identity() const
{
  return turn_delta_inf == 0.0 && turn_delta_zero == 0.0 && speed_turn == 0.0 && sideslip == 0.0 &&
         turn_offset == 0.0 && turn_offset_com == 0.0;
}

bool TrackingModel::identity() const { return map_identity() && base_delay_s == 0.0 && arm_delay_s == 0.0; }

std::array<double, 7> TrackingModel::map_parameters() const
{
  return {turn_delta_inf, turn_delta_zero, turn_scale, speed_turn, sideslip, turn_offset, turn_offset_com};
}

TrackingModel load_tracking_model(const std::string & yaml_path)
{
  mmn::Json root;
  try {
    root = mmn::load_yaml_file(yaml_path);
  } catch (const std::exception & e) {
    throw std::runtime_error(yaml_path + ": cannot be read as a tracking model (" + e.what() + ")");
  }
  TrackingModel m;
  if (root.is_null()) return m;  // an empty file: the identity
  if (!root.is_object()) throw std::runtime_error(yaml_path + ": a tracking model is a mapping of its parameters");
  struct Field
  {
    double * value;
    double low, high;
  };
  const std::pair<const char *, Field> fields[] = {
    {"turn_delta_inf", {&m.turn_delta_inf, -0.5, 0.5}}, {"turn_delta_zero", {&m.turn_delta_zero, -0.99, 0.5}},
    {"turn_scale", {&m.turn_scale, 0.001, 1.0}},        {"speed_turn", {&m.speed_turn, 0.0, 1.0}},
    {"sideslip", {&m.sideslip, -0.5, 0.5}},             {"base_delay_s", {&m.base_delay_s, 0.0, 0.2}},
    {"arm_delay_s", {&m.arm_delay_s, 0.0, 0.2}},       {"turn_offset", {&m.turn_offset, -0.3, 0.3}},
    {"turn_offset_com", {&m.turn_offset_com, -5.0, 5.0}}};
  for (const auto & item : root.items()) {
    const Field * field = nullptr;
    for (const auto & [name, f] : fields)
      if (item.key() == name) field = &f;
    if (!field) throw std::runtime_error(yaml_path + ": unknown tracking model key '" + item.key() + "'");
    if (!item.value().is_number() || !std::isfinite(item.value().get<double>()))
      throw std::runtime_error(yaml_path + ": " + item.key() + " must be a finite number");
    const double v = item.value().get<double>();
    if (v < field->low || v > field->high)
      throw std::runtime_error(yaml_path + ": " + item.key() + " = " + std::to_string(v) + " is outside [" +
                               std::to_string(field->low) + ", " + std::to_string(field->high) + "]");
    *field->value = v;
  }
  return m;
}
State advance_to_actuation(const State & x, const std::vector<SentCommand> & sent, double now,
                           const TrackingModel & model)
{
  State out = x;
  // each command over its share of [now - delay, now): from its send time (or the window's start) to the next one's
  // (or now)
  auto through = [&](double delay, auto && act) {
    if (!(delay > 0.0)) return;
    const double start = now - delay;
    for (size_t i = 0; i < sent.size() && sent[i].t < now; ++i) {
      const double a = std::max(sent[i].t, start);
      const double b = i + 1 < sent.size() ? std::min(sent[i + 1].t, now) : now;
      if (b > a) act(sent[i].u, b - a);
    }
  };
  through(model.base_delay_s, [&](const Control & u, double h) {
    Control base = Control::Zero();
    base.head<2>() = u.head<2>();
    out = discrete_step(out, base, h, model).x_next;
  });
  through(model.arm_delay_s, [&](const Control & u, double h) { out.tail<6>() += h * u.tail<6>(); });
  return out;
}

}  // namespace mobile_manipulator_mpc
