#pragma once
// Pure helpers for the navigation telemetry node (no ROS dependencies).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace mobile_manipulator_navigation
{
struct PathPoint
{
  double x, y, yaw;
  bool operator==(const PathPoint & other) const
  {
    return x == other.x && y == other.y && yaw == other.yaw;
  }
};

inline std::string goal_status_name(int status)
{
  static const char * names[] = {"unknown", "accepted", "executing", "canceling",
                                 "succeeded", "canceled", "aborted"};
  return status >= 0 && status <= 6 ? names[status] : std::to_string(status);
}

inline std::string monitor_action_name(int action)
{
  static const char * names[] = {"none", "stop", "slowdown", "approach", "limit"};
  return action >= 0 && action <= 4 ? names[action] : std::to_string(action);
}

// Rounds like Python's round(value, digits) for display (ties away from zero).
inline double round_to(double value, int digits)
{
  const double scale = std::pow(10.0, digits);
  return std::round(value * scale) / scale;
}

inline double path_length(const std::vector<PathPoint> & points)
{
  double length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    length += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  }
  return length;
}

// Indices to keep: the first point, then points at least `spacing` apart, and always the last.
inline std::vector<size_t> downsample(const std::vector<PathPoint> & points, double spacing)
{
  std::vector<size_t> kept;
  if (points.empty()) return kept;
  kept.push_back(0);
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    const auto & last = points[kept.back()];
    if (std::hypot(points[i].x - last.x, points[i].y - last.y) >= spacing - 1e-9) {  // float steps
      kept.push_back(i);
    }
  }
  if (points.size() > 1) kept.push_back(points.size() - 1);
  return kept;
}

// Distance from (px, py) to the polyline through points (empty without a path).
inline std::optional<double> cross_track(double px, double py, const std::vector<PathPoint> & points)
{
  if (points.empty()) return std::nullopt;
  if (points.size() == 1) return std::hypot(px - points[0].x, py - points[0].y);
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < points.size(); ++i) {
    const double ax = points[i - 1].x, ay = points[i - 1].y;
    const double dx = points[i].x - ax, dy = points[i].y - ay;
    const double length_sq = dx * dx + dy * dy;
    const double s = length_sq == 0.0
      ? 0.0 : std::clamp(((px - ax) * dx + (py - ay) * dy) / length_sq, 0.0, 1.0);
    best = std::min(best, std::hypot(px - (ax + s * dx), py - (ay + s * dy)));
  }
  return best;
}

struct Event
{
  double t;
  std::string text;
};

// Bounded log of (simulation time, text) events, newest last.
class EventLog
{
public:
  explicit EventLog(size_t size = 15) : size_(size) {}

  void add(double time_s, std::string text)
  {
    if (events_.size() == size_) events_.pop_front();
    events_.push_back({round_to(time_s, 2), std::move(text)});
  }

  const std::deque<Event> & events() const { return events_; }

private:
  size_t size_;
  std::deque<Event> events_;
};

// Rate and long gaps of a periodic stream from its message stamps.
class GapCounter
{
public:
  explicit GapCounter(double gap_threshold, size_t window = 20)
  : threshold_(gap_threshold), window_(window) {}

  // Returns the gap when it exceeds the threshold.
  std::optional<double> add(double stamp)
  {
    std::optional<double> gap;
    if (!stamps_.empty() && stamp > stamps_.back()) {
      const double delta = stamp - stamps_.back();
      max_gap_ = std::max(max_gap_, delta);
      if (delta > threshold_) {
        ++long_gaps_;
        gap = delta;
      }
    }
    if (stamps_.size() == window_) stamps_.pop_front();
    stamps_.push_back(stamp);
    return gap;
  }

  double rate() const
  {
    if (stamps_.size() < 2 || stamps_.back() <= stamps_.front()) return 0.0;
    return static_cast<double>(stamps_.size() - 1) / (stamps_.back() - stamps_.front());
  }

  int long_gaps() const { return long_gaps_; }
  double max_gap() const { return max_gap_; }

private:
  double threshold_;
  size_t window_;
  std::deque<double> stamps_;
  int long_gaps_ = 0;
  double max_gap_ = 0.0;
};
}  // namespace mobile_manipulator_navigation
