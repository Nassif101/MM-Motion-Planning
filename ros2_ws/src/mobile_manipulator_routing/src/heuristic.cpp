#include "mobile_manipulator_routing/heuristic.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace mobile_manipulator_routing
{
namespace
{
// The table's values are taken below their computed cost by a margin larger than float rounding and than the
// difference between relative and absolute floating-point path lengths, so they stay lower bounds.
float lowered(double value)
{
  const double safe = value*(1.0-1e-6)-1e-6;
  float stored = static_cast<float>(safe);
  if (static_cast<double>(stored) > safe) stored = std::nextafter(stored,-std::numeric_limits<float>::infinity());
  return stored;
}
}  // namespace

double free_space_cost(const MotionPrimitive & motion, const PrimitiveSet & primitives, double non_straight_penalty,
                       double rotation_cost_m_per_rad)
{
  // Search::Impl::path_cost on primitive_path's poses, relative to the start cell's centre, with zero soft cost
  std::vector<Pose2> path(motion.poses.begin(),motion.poses.end());
  path.back() = {motion.dx*primitives.resolution,motion.dy*primitives.resolution,primitives.headings[motion.end_heading]};
  double length = 0.0, rotation = 0.0;
  bool moving_turn = motion.turning && !motion.pure_rotation;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i-1]; const auto & b = path[i];
    const double segment = std::hypot(b.x-a.x,b.y-a.y);
    const double turn = std::abs(wrap_angle(b.yaw-a.yaw));
    if (segment <= 1e-12) { rotation += turn; continue; }
    length += segment;
    moving_turn = moving_turn || turn > 1e-10;
  }
  return std::max(length,motion.length_m)*(moving_turn ? non_straight_penalty : 1.0) + rotation_cost_m_per_rad*rotation;
}

FreeSpaceHeuristic::FreeSpaceHeuristic(std::shared_ptr<const PrimitiveSet> primitives, double non_straight_penalty,
                                       double rotation_cost_m_per_rad, int radius_cells)
: primitives_(std::move(primitives)), penalty_(non_straight_penalty), rotation_(rotation_cost_m_per_rad),
  radius_(radius_cells)
{
  if (!primitives_ || primitives_->headings.empty()) throw std::invalid_argument("heuristic needs a primitive bank");
  if (radius_ < 3 || radius_ > 400) throw std::invalid_argument("heuristic window must be 3..400 cells");
  if (!std::isfinite(penalty_) || penalty_ < 1.0 || !std::isfinite(rotation_) || rotation_ <= 0.0)
    throw std::invalid_argument("heuristic needs the search's penalty (>= 1) and a positive rotation cost");
  ending_.resize(primitives_->headings.size());
  for (std::size_t m = 0; m < primitives_->motions.size(); ++m) {
    const auto & motion = primitives_->motions[m];
    edge_cost_.push_back(free_space_cost(motion,*primitives_,penalty_,rotation_));
    ending_.at(motion.end_heading).push_back(m);
  }
  tables_.resize(primitives_->headings.size());
}

bool FreeSpaceHeuristic::serves(const PrimitiveSet & primitives, double non_straight_penalty,
                                double rotation_cost_m_per_rad) const
{
  return &primitives == primitives_.get() && non_straight_penalty == penalty_ && rotation_cost_m_per_rad == rotation_;
}

std::size_t FreeSpaceHeuristic::storage_bytes() const
{
  std::size_t bytes = sizeof(*this)+edge_cost_.capacity()*sizeof(double);
  for (const auto & t : tables_) bytes += t.capacity()*sizeof(float);
  return bytes;
}

const std::vector<float> & FreeSpaceHeuristic::table(std::size_t bin)
{
  auto & values = tables_.at(bin);
  if (!values.empty()) return values;
  const int side = 2*radius_+1;
  const std::size_t headings = primitives_->headings.size();
  const auto index = [&](int x, int y, std::size_t h) {
    return (static_cast<std::size_t>(y+radius_)*static_cast<std::size_t>(side)+static_cast<std::size_t>(x+radius_))*
           headings+h;
  };
  std::vector<double> cost(static_cast<std::size_t>(side)*static_cast<std::size_t>(side)*headings,
                           std::numeric_limits<double>::infinity());
  using Entry = std::pair<double,std::size_t>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  const double r = primitives_->resolution;
  const double goal_heading = primitives_->headings[bin];
  // goal-edge states: the connector's cost is at least the distance to the goal cell's square and the rotation
  for (int y = -2; y <= 2; ++y)
    for (int x = -2; x <= 2; ++x)
      for (std::size_t h = 0; h < headings; ++h) {
        const double gap = std::hypot(std::max(0.0,(std::abs(x)-0.5)*r),std::max(0.0,(std::abs(y)-0.5)*r));
        const double value = gap+rotation_*std::abs(wrap_angle(goal_heading-primitives_->headings[h]));
        const auto at = index(x,y,h);
        if (value < cost[at]) { cost[at] = value; open.push({value,at}); }
      }
  // backward over the primitives: a state's value is its edge's cost plus the value of where the edge ends
  while (!open.empty()) {
    const auto [value,at] = open.top();
    open.pop();
    if (value > cost[at]) continue;
    const std::size_t h = at%headings;
    const std::size_t cellular = at/headings;
    const int x = static_cast<int>(cellular%static_cast<std::size_t>(side))-radius_;
    const int y = static_cast<int>(cellular/static_cast<std::size_t>(side))-radius_;
    for (const auto m : ending_[h]) {
      const auto & motion = primitives_->motions[m];
      const int sx = x-motion.dx, sy = y-motion.dy;
      if (std::abs(sx) > radius_ || std::abs(sy) > radius_) continue;
      const auto from = index(sx,sy,motion.start_heading);
      const double candidate = value+edge_cost_[m];
      if (candidate < cost[from]) { cost[from] = candidate; open.push({candidate,from}); }
    }
  }
  values.resize(cost.size());
  for (std::size_t i = 0; i < cost.size(); ++i)
    values[i] = std::isfinite(cost[i]) ? lowered(cost[i]) : std::numeric_limits<float>::infinity();
  return values;
}

double FreeSpaceHeuristic::bound(int dx, int dy, std::size_t heading, std::size_t bin)
{
  const double r = primitives_->resolution;
  const int reach = std::max(std::abs(dx),std::abs(dy));
  // A path leaving the window reaches a lattice state at least radius+1 cells from the goal's cell (Chebyshev, so at
  // least that far in metres) and comes back to the goal inside the goal cell; translation costs at least its length.
  const auto exit = [&](int from) { return std::max(0.0,(radius_+1-from)*r)+(radius_+0.5)*r; };
  if (reach > radius_) return 0.0;  // outside the window: the caller's Euclidean bound
  const double inside = table(bin)[(static_cast<std::size_t>(dy+radius_)*static_cast<std::size_t>(2*radius_+1)+
                                    static_cast<std::size_t>(dx+radius_))*primitives_->headings.size()+heading];
  return std::min(static_cast<double>(inside),static_cast<double>(lowered(exit(reach))));
}
}  // namespace mobile_manipulator_routing
