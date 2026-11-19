#include "mobile_manipulator_routing/footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace mobile_manipulator_routing
{
namespace
{
constexpr std::size_t kMaxVertices = 256;
constexpr std::size_t kMaxPoses = 100000;
constexpr std::size_t kMaxSubdivisions = 10000000;
constexpr double kEpsilon = 1e-12;

double cross(Point2 a, Point2 b, Point2 c)
{
  return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
}

bool on_segment(Point2 a, Point2 b, Point2 p, double tolerance)
{
  return std::abs(cross(a,b,p)) <= tolerance &&
    p.x >= std::min(a.x,b.x)-tolerance && p.x <= std::max(a.x,b.x)+tolerance &&
    p.y >= std::min(a.y,b.y)-tolerance && p.y <= std::max(a.y,b.y)+tolerance;
}

bool segments_intersect(Point2 a, Point2 b, Point2 c, Point2 d, double tolerance)
{
  const double ab_c = cross(a,b,c), ab_d = cross(a,b,d);
  const double cd_a = cross(c,d,a), cd_b = cross(c,d,b);
  const bool proper = ((ab_c > tolerance && ab_d < -tolerance) ||
                       (ab_c < -tolerance && ab_d > tolerance)) &&
                      ((cd_a > tolerance && cd_b < -tolerance) ||
                       (cd_a < -tolerance && cd_b > tolerance));
  return proper || on_segment(a,b,c,tolerance) || on_segment(a,b,d,tolerance) ||
    on_segment(c,d,a,tolerance) || on_segment(c,d,b,tolerance);
}

double validate_polygon(const Polygon & polygon)
{
  if (polygon.size() < 3 || polygon.size() > kMaxVertices)
    throw std::invalid_argument("footprint needs 3..256 ordered convex vertices");
  double radius = 0.0;
  for (const auto & p : polygon) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x) > 1e6 || std::abs(p.y) > 1e6)
      throw std::invalid_argument("footprint coordinates must be finite and bounded");
    radius = std::max(radius, std::hypot(p.x,p.y));
  }
  const double tolerance = kEpsilon * std::max(1.0,radius*radius);
  int direction = 0;
  long double twice_area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i+1)%polygon.size()];
    const auto & c = polygon[(i+2)%polygon.size()];
    if (std::hypot(a.x-b.x,a.y-b.y) <= kEpsilon)
      throw std::invalid_argument("footprint has duplicate consecutive vertices");
    twice_area += static_cast<long double>(a.x)*b.y - static_cast<long double>(a.y)*b.x;
    const double turn = cross(a,b,c);
    if (std::abs(turn) > tolerance) {
      const int next = turn > 0.0 ? 1 : -1;
      if (direction && next != direction) throw std::invalid_argument("footprint is not convex");
      direction = next;
    }
    for (std::size_t j = i+1; j < polygon.size(); ++j) {
      if (j == i+1 || (i == 0 && j == polygon.size()-1)) continue;
      if (segments_intersect(a,b,polygon[j],polygon[(j+1)%polygon.size()],tolerance))
        throw std::invalid_argument("footprint intersects itself");
    }
  }
  if (!direction || std::abs(twice_area) <= tolerance)
    throw std::invalid_argument("footprint has no area");
  return radius;
}

// Convex polygon versus closed axis-aligned rectangle. Contact is collision, including
// a cell wholly inside the footprint (which a vertex-only test would miss).
bool intersects_cell(const Polygon & polygon, double min_x, double min_y, double max_x, double max_y)
{
  const double cx = min_x + (max_x-min_x)*0.5, cy = min_y + (max_y-min_y)*0.5;
  const double hx = (max_x-min_x)*0.5, hy = (max_y-min_y)*0.5;
  const auto separated = [&](double nx, double ny) {
    double low = std::numeric_limits<double>::infinity();
    double high = -low;
    for (const auto & p : polygon) {
      const double projection = nx*(p.x-cx) + ny*(p.y-cy);
      low = std::min(low,projection);
      high = std::max(high,projection);
    }
    const double reach = std::abs(nx)*hx + std::abs(ny)*hy;
    const double tolerance = kEpsilon * (1.0+std::abs(nx)+std::abs(ny)+reach);
    return high < -reach-tolerance || low > reach+tolerance;
  };
  if (separated(1.0,0.0) || separated(0.0,1.0)) return false;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i+1)%polygon.size()];
    if (separated(a.y-b.y,b.x-a.x)) return false;
  }
  return true;
}

struct Interval
{
  std::size_t count = 1;
  double angular = 0.0, bound = 0.0;
};

Interval interval(const Pose2 & a, const Pose2 & b, double radius, double maximum_step)
{
  const double angular = wrap_angle(b.yaw-a.yaw);
  const double travel = std::hypot(b.x-a.x,b.y-a.y) + radius*std::abs(angular);
  const double subdivisions = std::max(1.0,std::ceil(travel/maximum_step));
  if (!std::isfinite(travel) || !std::isfinite(subdivisions) || subdivisions > kMaxSubdivisions)
    throw std::invalid_argument("motion segment exceeds bounded interpolation work");
  const auto count = static_cast<std::size_t>(subdivisions);
  return {count,angular,travel/static_cast<double>(count)};
}
}  // namespace

struct MotionCheck::Impl
{
  std::shared_ptr<const GridSnapshot> grid;
  Polygon footprint, world;
  std::vector<Pose2> poses;
  double radius = 0.0, maximum_step = 0.0125;
  std::size_t segment = 0, substep = 0;
  Interval current;
  bool prepared = false, final_pose = false;
  int min_x = 0, max_x = 0, min_y = 0, max_y = 0, cell_x = 0, cell_y = 0;
  double dilation = 0.0;
  CheckStatus status = CheckStatus::Pending;
  // masked(): a lattice motion's precomputed cells from `start` (prepared: its envelope is inside the map)
  std::shared_ptr<const SweptMask> mask;
  CellIndex start{0,0};
  std::size_t cursor = 0;

  CheckStatus advance_masked(std::size_t & cell_budget, const std::function<bool()> & canceled)
  {
    if (!prepared) {
      const auto origin = grid->origin();
      const double resolution = grid->resolution();
      const auto centre = grid->cell_center(start.x,start.y);
      // the envelope touching the unknown exterior blocks, as in prepare()
      if (!(centre.x+mask->left > origin.x && centre.y+mask->bottom > origin.y &&
            centre.x+mask->right < origin.x+grid->width()*resolution &&
            centre.y+mask->top < origin.y+grid->height()*resolution))
        return status = CheckStatus::Blocked;
      prepared = true;
    }
    while (cell_budget > 0 && cursor < mask->offsets.size()) {
      if (canceled && canceled()) return status = CheckStatus::Canceled;
      --cell_budget;
      const auto & offset = mask->offsets[cursor++];
      const int x = start.x+offset.x, y = start.y+offset.y;
      // a mask cell already met the dilated closed-contact test: occupied or unknown there blocks
      if (!grid->contains(x,y) || grid->blocked(x,y)) return status = CheckStatus::Blocked;
    }
    if (cursor >= mask->offsets.size()) status = CheckStatus::Clear;
    return status;
  }

  void prepare()
  {
    Pose2 pose = poses.back();
    final_pose = segment+1 >= poses.size();
    dilation = 0.0;
    if (!final_pose) {
      const auto & a = poses[segment];
      const auto & b = poses[segment+1];
      if (substep == 0) current = interval(a,b,radius,maximum_step);
      const double t = static_cast<double>(substep)/static_cast<double>(current.count);
      pose = {a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.yaw+current.angular*t};
      dilation = current.bound;
    }
    const double cosine = std::cos(pose.yaw), sine = std::sin(pose.yaw);
    double left = std::numeric_limits<double>::infinity(), bottom = left;
    double right = -left, top = right;
    for (std::size_t i = 0; i < footprint.size(); ++i) {
      const auto & p = footprint[i];
      world[i] = {pose.x+cosine*p.x-sine*p.y,pose.y+sine*p.x+cosine*p.y};
      left = std::min(left,world[i].x); right = std::max(right,world[i].x);
      bottom = std::min(bottom,world[i].y); top = std::max(top,world[i].y);
    }
    const auto origin = grid->origin();
    const double resolution = grid->resolution();
    const double map_right = origin.x+grid->width()*resolution;
    const double map_top = origin.y+grid->height()*resolution;
    // Cell boundaries and transformed vertices reach the same mathematical contact
    // through different floating-point operations. Include a coordinate-scale rounding
    // envelope before enumeration as well as SAT, so a cell cannot be omitted because
    // (for example) 0.35/0.05 rounds just below 7. This is not a whole-cell allowance.
    const double coordinate_scale = std::max({1.0,std::abs(left),std::abs(right),std::abs(bottom),
      std::abs(top),std::abs(origin.x),std::abs(origin.y),std::abs(map_right),std::abs(map_top)});
    dilation += 64.0*std::numeric_limits<double>::epsilon()*coordinate_scale;
    left -= dilation; right += dilation; bottom -= dilation; top += dilation;
    // The conservative envelope touching unknown exterior is blocked. This also keeps
    // all following floating-to-integer conversions within validated grid dimensions.
    if (!std::isfinite(left) || !std::isfinite(right) || !std::isfinite(bottom) || !std::isfinite(top) ||
        left <= origin.x || bottom <= origin.y || right >= map_right || top >= map_top) {
      status = CheckStatus::Blocked;
      return;
    }
    min_x = std::max(0,static_cast<int>(std::floor(std::nextafter((left-origin.x)/resolution,
      -std::numeric_limits<double>::infinity()))));
    min_y = std::max(0,static_cast<int>(std::floor(std::nextafter((bottom-origin.y)/resolution,
      -std::numeric_limits<double>::infinity()))));
    max_x = std::min(grid->width()-1,static_cast<int>(std::floor((right-origin.x)/resolution)));
    max_y = std::min(grid->height()-1,static_cast<int>(std::floor((top-origin.y)/resolution)));
    cell_x = min_x; cell_y = min_y;
    prepared = true;
  }

  void finish_query()
  {
    prepared = false;
    if (final_pose) {
      status = CheckStatus::Clear;
    } else if (++substep >= current.count) {
      ++segment;
      substep = 0;
    }
  }
};

MotionCheck::MotionCheck(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
MotionCheck::~MotionCheck() = default;
MotionCheck::MotionCheck(MotionCheck &&) noexcept = default;
MotionCheck & MotionCheck::operator=(MotionCheck &&) noexcept = default;

CheckStatus MotionCheck::advance(std::size_t & cell_budget, const std::function<bool()> & canceled)
{
  if (!impl_) throw std::logic_error("moved-from motion check");
  auto & state = *impl_;
  if (state.status != CheckStatus::Pending) return state.status;
  if (canceled && canceled()) return state.status = CheckStatus::Canceled;
  if (state.mask) return state.advance_masked(cell_budget,canceled);
  while (cell_budget > 0) {
    if (canceled && canceled()) return state.status = CheckStatus::Canceled;
    if (!state.prepared) {
      state.prepare();
      if (state.status != CheckStatus::Pending) return state.status;
    }
    --cell_budget;
    if (state.grid->blocked(state.cell_x,state.cell_y)) {
      const auto origin = state.grid->origin();
      const double r = state.grid->resolution();
      const double x = origin.x+state.cell_x*r, y = origin.y+state.cell_y*r;
      if (intersects_cell(state.world,x-state.dilation,y-state.dilation,
                          x+r+state.dilation,y+r+state.dilation))
        return state.status = CheckStatus::Blocked;
    }
    if (++state.cell_x > state.max_x) {
      state.cell_x = state.min_x;
      if (++state.cell_y > state.max_y) state.finish_query();
    }
    if (state.status != CheckStatus::Pending) return state.status;
  }
  return state.status;
}

std::size_t MotionCheck::storage_bytes() const
{
  if (!impl_) return sizeof(*this);
  return sizeof(*this)+sizeof(Impl)+impl_->poses.capacity()*sizeof(Pose2)+
    (impl_->footprint.capacity()+impl_->world.capacity())*sizeof(Point2);
}

FootprintChecker::FootprintChecker(std::shared_ptr<const GridSnapshot> grid, Polygon footprint)
: grid_(std::move(grid)), footprint_(std::move(footprint))
{
  if (!grid_) throw std::invalid_argument("footprint checker needs a grid");
  radius_ = validate_polygon(footprint_);
}

MotionCheck FootprintChecker::motion(std::vector<Pose2> poses, double max_point_step_m) const
{
  if (poses.empty() || poses.size() > kMaxPoses || !std::isfinite(max_point_step_m) || max_point_step_m <= 0.0)
    throw std::invalid_argument("motion needs bounded poses and a positive finite interpolation step");
  for (auto & pose : poses) {
    if (!finite(pose)) throw std::invalid_argument("motion has a non-finite pose");
    pose.yaw = wrap_angle(pose.yaw);
  }
  // Validate interpolation bounds once; only cursor state and raw poses are retained.
  for (std::size_t i = 1; i < poses.size(); ++i) interval(poses[i-1],poses[i],radius_,max_point_step_m);
  auto impl = std::make_unique<MotionCheck::Impl>();
  impl->grid = grid_;
  impl->footprint = footprint_;
  impl->world.resize(footprint_.size());
  impl->poses = std::move(poses);
  impl->radius = radius_;
  impl->maximum_step = max_point_step_m;
  return MotionCheck(std::move(impl));
}

SweptMask FootprintChecker::swept_mask(std::vector<Pose2> relative, double max_point_step_m) const
{
  if (relative.empty() || relative.size() > kMaxPoses || !std::isfinite(max_point_step_m) || max_point_step_m <= 0.0)
    throw std::invalid_argument("swept mask needs bounded poses and a positive finite interpolation step");
  for (auto & pose : relative) {
    if (!finite(pose) || std::abs(pose.x) > 1e3 || std::abs(pose.y) > 1e3)
      throw std::invalid_argument("swept mask needs finite, bounded relative poses");
    pose.yaw = wrap_angle(pose.yaw);
  }
  const double resolution = grid_->resolution();
  const auto origin = grid_->origin();
  // Any start cell's centre lies within the map, so its absolute coordinates are bounded by the map's extents; the
  // rounding envelope below is then at least motion()'s for every start (which takes the same 64-ulp form).
  const double map_scale = std::max({1.0,std::abs(origin.x),std::abs(origin.y),
    std::abs(origin.x+grid_->width()*resolution),std::abs(origin.y+grid_->height()*resolution)});
  SweptMask mask;
  mask.left = mask.bottom = std::numeric_limits<double>::infinity();
  mask.right = mask.top = -std::numeric_limits<double>::infinity();
  std::set<std::pair<int,int>> cells;  // (y, x)
  Polygon world(footprint_.size());
  const auto sample = [&](const Pose2 & pose, double bound) {
    const double cosine = std::cos(pose.yaw), sine = std::sin(pose.yaw);
    double left = std::numeric_limits<double>::infinity(), bottom = left;
    double right = -left, top = right;
    for (std::size_t i = 0; i < footprint_.size(); ++i) {
      const auto & p = footprint_[i];
      world[i] = {pose.x+cosine*p.x-sine*p.y,pose.y+sine*p.x+cosine*p.y};
      left = std::min(left,world[i].x); right = std::max(right,world[i].x);
      bottom = std::min(bottom,world[i].y); top = std::max(top,world[i].y);
    }
    const double reach = std::max({std::abs(left),std::abs(right),std::abs(bottom),std::abs(top)});
    const double dilation = bound+64.0*std::numeric_limits<double>::epsilon()*(map_scale+reach);
    left -= dilation; right += dilation; bottom -= dilation; top += dilation;
    mask.left = std::min(mask.left,left); mask.right = std::max(mask.right,right);
    mask.bottom = std::min(mask.bottom,bottom); mask.top = std::max(mask.top,top);
    // relative cell j spans [(j - 1/2) r, (j + 1/2) r] about the start cell's centre; one cell of slack each side
    const int x0 = static_cast<int>(std::floor(left/resolution+0.5))-1;
    const int x1 = static_cast<int>(std::floor(right/resolution+0.5))+1;
    const int y0 = static_cast<int>(std::floor(bottom/resolution+0.5))-1;
    const int y1 = static_cast<int>(std::floor(top/resolution+0.5))+1;
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) {
        if (cells.count({y,x})) continue;
        if (intersects_cell(world,(x-0.5)*resolution-dilation,(y-0.5)*resolution-dilation,
                            (x+0.5)*resolution+dilation,(y+0.5)*resolution+dilation))
          cells.insert({y,x});
      }
  };
  // motion()'s interpolation: each segment's substeps from its start, then the final pose undilated
  for (std::size_t segment = 0; segment+1 < relative.size(); ++segment) {
    const auto & a = relative[segment];
    const auto & b = relative[segment+1];
    const auto step = interval(a,b,radius_,max_point_step_m);
    for (std::size_t substep = 0; substep < step.count; ++substep) {
      const double t = static_cast<double>(substep)/static_cast<double>(step.count);
      sample({a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.yaw+step.angular*t},step.bound);
    }
  }
  sample(relative.back(),0.0);
  mask.offsets.reserve(cells.size());
  for (const auto & [y,x] : cells) mask.offsets.push_back({x,y});
  return mask;
}

MotionCheck FootprintChecker::masked(std::shared_ptr<const SweptMask> mask, CellIndex start) const
{
  if (!mask || mask->offsets.empty()) throw std::invalid_argument("masked check needs a nonempty swept mask");
  if (!grid_->contains(start.x,start.y)) throw std::invalid_argument("masked check starts outside the map");
  auto impl = std::make_unique<MotionCheck::Impl>();
  impl->grid = grid_;
  impl->mask = std::move(mask);
  impl->start = start;
  return MotionCheck(std::move(impl));
}

bool FootprintChecker::pose_clear(const Pose2 & pose) const
{
  auto check = motion({pose});
  std::size_t budget = std::numeric_limits<std::size_t>::max();
  return check.advance(budget) == CheckStatus::Clear;
}

std::size_t FootprintChecker::storage_bytes() const
{
  return sizeof(*this)+footprint_.capacity()*sizeof(Point2);
}
}  // namespace mobile_manipulator_routing
