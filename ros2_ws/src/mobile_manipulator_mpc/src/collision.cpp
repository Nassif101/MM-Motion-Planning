#include "mobile_manipulator_mpc/collision.hpp"

#include <algorithm>
#include <map>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <utility>

#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/narrowphase/distance.h>

namespace mobile_manipulator_mpc
{
namespace
{
namespace mmg = mobile_manipulator_geometry;
using Eigen::Vector3d;
constexpr double kInf = std::numeric_limits<double>::infinity();

// FCL: libccd GJK (the default), unsigned distance with the nearest points. The signed
// variant runs EPA for penetrating pairs, which in FCL 0.7 can abort on an internal assertion
// (ComputeVisiblePatch) and whose points are no reliable direction; penetration is estimated
// below instead. The tolerance keeps the distance of the curved cylinders accurate to about
// 1e-10 m, so finite differences of the distance match the gradient.
constexpr double kGjkTolerance = 1e-10;
constexpr double kSeparatedM = 1e-6;  // below this the nearest points give no normal

// A convex primitive in the map frame. Box: dims = half extents; Cylinder: (radius, half
// length, unused) along the local z.
struct Solid
{
  Eigen::Isometry3d pose;
  mmg::Shape shape;
  Vector3d dims;
};

// Half extents of the solid's bounding box in its own frame.
Vector3d own_half(const Solid & s)
{
  return s.shape == mmg::Shape::Box ? s.dims : Vector3d(s.dims(0), s.dims(0), s.dims(1));
}

// Half width of the solid along the unit direction u (its support).
double half_width(const Solid & s, const Vector3d & u)
{
  if (s.shape == mmg::Shape::Box) return (s.pose.linear().transpose() * u).cwiseAbs().dot(s.dims);
  const double c = s.pose.linear().col(2).dot(u);
  return std::abs(c) * s.dims(1) + s.dims(0) * std::sqrt(std::max(0.0, 1.0 - c * c));
}

// The solid's point farthest along u (a face or edge centre where u is perpendicular to it).
Vector3d support_point(const Solid & s, const Vector3d & u)
{
  auto side = [](double v) { return std::abs(v) < 1e-12 ? 0.0 : (v > 0.0 ? 1.0 : -1.0); };
  if (s.shape == mmg::Shape::Box) {
    const Vector3d local = s.pose.linear().transpose() * u;
    return s.pose * Vector3d(side(local.x()) * s.dims.x(), side(local.y()) * s.dims.y(), side(local.z()) * s.dims.z());
  }
  const Vector3d axis = s.pose.linear().col(2);
  const double c = axis.dot(u);
  Vector3d p = s.pose.translation() + side(c) * s.dims(1) * axis;
  const Vector3d radial = u - c * axis;
  if (radial.norm() > 1e-12) p += s.dims(0) * radial.normalized();
  return p;
}

// Closest point of the solid to p.
Vector3d project(const Solid & s, const Vector3d & p)
{
  Vector3d local = s.pose.inverse() * p;
  if (s.shape == mmg::Shape::Box) {
    local = local.cwiseMax(-s.dims).cwiseMin(s.dims);
  } else {
    local.z() = std::clamp(local.z(), -s.dims(1), s.dims(1));
    const double radial = local.head<2>().norm();
    if (radial > s.dims(0)) local.head<2>() *= s.dims(0) / radial;
  }
  return s.pose * local;
}

// World-frame bounding box.
struct Aabb
{
  Vector3d lo, hi;
};

Aabb aabb(const Solid & s)
{
  Vector3d e;
  for (int i = 0; i < 3; ++i) e(i) = half_width(s, Vector3d::Unit(i));
  return {s.pose.translation() - e, s.pose.translation() + e};
}

double gap(const Aabb & a, const Aabb & b)
{
  return (b.lo - a.hi).cwiseMax(a.lo - b.hi).cwiseMax(0.0).norm();
}

bool vertically_apart(const Aabb & a, const Aabb & b)
{
  return a.hi.z() < b.lo.z() || b.hi.z() < a.lo.z();
}

// Lower bound on the signed distance from the separating axes of both solids' frames (the
// same axes as penetration() below): the gap between their bounding boxes in either frame
// when one is positive, otherwise the least overlap along those axes, which is then exactly
// the penetration estimate.
double lower_bound(const Solid & a, const Solid & b)
{
  double apart = 0.0, overlap = -kInf;
  for (const auto & [x, frame] : {std::pair<const Solid *, const Solid *>{&a, &b}, {&b, &a}}) {
    const Vector3d d = frame->pose.linear().transpose() * (x->pose.translation() - frame->pose.translation());
    const Vector3d half = own_half(*frame);
    Vector3d gaps;
    for (int i = 0; i < 3; ++i) {
      const double separation = std::abs(d(i)) - half(i) - half_width(*x, frame->pose.linear().col(i));
      overlap = std::max(overlap, separation);
      gaps(i) = std::max(0.0, separation);
    }
    apart = std::max(apart, gaps.norm());
  }
  return apart > 0.0 ? apart : overlap;
}

// Whether the solids are surely within `distance`: alternating projections give a point of
// each solid, whose separation is an upper bound on the distance (at most three rounds).
bool surely_within(const Solid & a, const Solid & b, double distance)
{
  Vector3d q = b.pose.translation();
  for (int i = 0; i < 3; ++i) {
    const Vector3d p = project(a, q);
    q = project(b, p);
    if ((p - q).norm() <= distance) return true;
  }
  return false;
}

struct Contact
{
  double distance;   // signed
  Vector3d n;        // unit, out of the other solid towards the robot
  Vector3d p_robot;  // robot point the row is linearized at
  Vector3d p_other;
};

// Penetrating pair: the separating axis with the least overlap among the face axes of both
// solids (cylinders: their frame axes). distance = that (negative) separation, n = the axis
// pointing from the other solid towards the robot, p_robot = the robot's deepest point along
// -n. The gradient is exact when the axis is a face normal of the other solid.
Contact penetration(const Solid & robot, const Solid & other)
{
  Contact best{-kInf, Vector3d::UnitZ(), robot.pose.translation(), robot.pose.translation()};
  const Vector3d between = robot.pose.translation() - other.pose.translation();
  for (const Solid * frame : {&other, &robot}) {
    for (int i = 0; i < 3; ++i) {
      const Vector3d u = frame->pose.linear().col(i);
      const double along = u.dot(between);
      const double separation = std::abs(along) - half_width(robot, u) - half_width(other, u);
      if (separation > best.distance) {
        best.distance = separation;
        best.n = along >= 0.0 ? u : Vector3d(-u);
      }
    }
  }
  best.p_robot = support_point(robot, -best.n);
  best.p_other = best.p_robot - best.distance * best.n;
  return best;
}

Contact exact(const fcl::CollisionGeometryd & robot_geometry, const Solid & robot,
              const fcl::CollisionGeometryd & other_geometry, const Solid & other)
{
  const fcl::DistanceRequestd request(true, false, 0.0, 0.0, kGjkTolerance, fcl::GST_LIBCCD);
  fcl::DistanceResultd result;
  fcl::distance(&robot_geometry, robot.pose, &other_geometry, other.pose, request, result);
  if (result.min_distance > kSeparatedM) {
    const Vector3d d = result.nearest_points[0] - result.nearest_points[1];
    if (d.norm() > 1e-12)
      return {result.min_distance, d.normalized(), result.nearest_points[0], result.nearest_points[1]};
  }
  return penetration(robot, other);
}

struct Body
{
  int link;                // whole-body model link index (the "panel" alias for the payload box)
  Eigen::Isometry3d pose;  // link -> primitive
  mmg::Shape shape;
  Vector3d dims;
  bool base = false;       // independent of the arm joints
  bool planar = false;     // every point keeps its height whatever the state
  bool panel = false;
  bool floor = false;      // kept above the floor plane
  std::string link_name;   // the primitive's link, "panel" for the payload box
  std::string self_name;   // "self/<link>"
  bool self_arm = false;   // one of collision.self_links (Phase 4B-B panel-against-arm-link rows)
  std::shared_ptr<fcl::CollisionGeometryd> geometry;
};

struct Placed
{
  Eigen::Isometry3d link;  // map -> link
  Solid solid;
  Aabb box;
};

// An obstacle box with its solid and bounding box.
struct Obstacle
{
  const ObstacleBox * box;
  Solid solid;
  Aabb aabb;
};

Obstacle make_obstacle(const ObstacleBox & box, const Bounds & bounds)
{
  return {&box, {box.pose, mmg::Shape::Box, box.size / 2.0}, {bounds.lo, bounds.hi}};
}

// The world's known boxes that are not `far` (a test on their bounding box), in their order. The
// known boxes never move, so a caller filters them once against the box it sweeps over the whole
// horizon instead of at every stage; the movers are filtered per stage.
template <typename Far>
std::vector<Obstacle> near_statics(const World & world, const Far & far)
{
  std::vector<Obstacle> near;
  const auto & boxes = world.statics();
  const auto & bounds = world.static_bounds();
  for (size_t i = 0; i < boxes.size(); ++i) {
    const Aabb box{bounds[i].lo, bounds[i].hi};
    if (!far(box)) near.push_back(make_obstacle(boxes[i], bounds[i]));
  }
  return near;
}

// `near` then the movers at `t_ahead` that are not `far`; `movers` keeps the movers' boxes.
template <typename Far>
void stage_obstacles(const World & world, double t_ahead, double now, const std::vector<Obstacle> & near,
                     const Far & far, std::vector<ObstacleBox> & movers, std::vector<Obstacle> & out)
{
  movers = world.movers_at(t_ahead, now);
  out.assign(near.begin(), near.end());
  for (const ObstacleBox & mover : movers) {
    const Obstacle o = make_obstacle(mover, bounds_of(mover));
    if (!far(o.aabb)) out.push_back(o);
  }
}

enum class Kind { Obstacle, Self, Floor };

struct Candidate
{
  double bound;  // lower bound, then the exact distance once evaluated
  Kind kind;
  int body;
  int other;     // obstacle index (Obstacle) or base body index (Self, once evaluated)
  bool evaluated;
  Contact contact;
};
}  // namespace

struct CollisionModel::Impl
{
  CollisionConfig config;
  std::vector<Body> bodies;
  int panel = -1;  // body index of the payload box

  void place(const Body & body, const State & x, Placed & out) const
  {
    out.link = link_pose(body.link, x);
    out.solid = {out.link * body.pose, body.shape, body.dims};
    out.box = aabb(out.solid);
  }

  Eigen::Matrix<double, 3, kNx> jacobian(const Body & body, const Placed & placed, const State & x,
                                         const Vector3d & p) const
  {
    return point_jacobian(body.link, x, placed.link.inverse() * p).J;
  }

  // The panel against its closest base primitive, pruned by lower bounds.
  void evaluate_self(const Placed * at, Candidate & candidate) const
  {
    const Body & panel_body = bodies[static_cast<size_t>(panel)];
    std::vector<std::pair<double, int>> order;
    for (size_t b = 0; b < bodies.size(); ++b)
      if (bodies[b].base) order.emplace_back(lower_bound(at[panel].solid, at[b].solid), static_cast<int>(b));
    std::sort(order.begin(), order.end());
    candidate.bound = kInf;
    for (const auto & [bound, b] : order) {
      if (bound >= candidate.bound) break;
      const Contact contact =
        exact(*panel_body.geometry, at[panel].solid, *bodies[static_cast<size_t>(b)].geometry, at[b].solid);
      if (contact.distance < candidate.bound) {
        candidate.bound = contact.distance;
        candidate.other = b;
        candidate.contact = contact;
      }
    }
    candidate.evaluated = true;
  }
};

CollisionModel::CollisionModel(const std::vector<mmg::Primitive> & primitives, CollisionConfig config)
{
  auto impl = std::make_shared<Impl>();
  impl->config = config;
  const int panel_link = link_index("panel");

  // States at which a body's dependence on the arm joints and its height are probed.
  std::vector<State> probes(3, State::Zero());
  probes[1] << 1.0, -2.0, 0.7, 0.3, -1.1, 0.9, -0.4, 1.3, 0.2;
  probes[2] << -3.0, 0.5, -2.1, -1.4, 0.6, -0.8, 1.7, -0.5, -1.2;

  bool payload_box = false;  // a box carried by the last arm joint (the tool flange's payload)
  for (const auto & primitive : primitives) {
    Body body;
    body.link = link_index(primitive.link);
    body.pose = primitive.pose;
    body.shape = primitive.shape;
    body.dims = primitive.dims;
    if (body.shape == mmg::Shape::Box) body.geometry = std::make_shared<fcl::Boxd>(2.0 * body.dims);
    else body.geometry = std::make_shared<fcl::Cylinderd>(body.dims(0), 2.0 * body.dims(1));

    // The payload box is the model's "panel" frame: rows and Jacobians use that alias.
    if (body.shape == mmg::Shape::Box) {
      State turned = probes[1];
      turned(kNx - 1) += 0.5;  // the last arm joint (wrist_3)
      payload_box = payload_box ||
                    !(link_pose(body.link, turned) * body.pose).isApprox(link_pose(body.link, probes[1]) * body.pose, 1e-9);
    }
    if (body.shape == mmg::Shape::Box && impl->panel < 0) {
      bool same = true;
      for (const auto & x : probes)
        same = same && (link_pose(body.link, x) * body.pose).isApprox(link_pose(panel_link, x), 1e-9);
      if (same) {
        body.panel = true;
        body.link = panel_link;
        body.pose = Eigen::Isometry3d::Identity();
        impl->panel = static_cast<int>(impl->bodies.size());
      }
    }
    double arm = 0.0, height = 0.0;
    for (const auto & x : probes) {
      for (int i = -1; i < 3; ++i) {
        const Vector3d local = i < 0 ? Vector3d::Zero() : Vector3d(0.1 * Vector3d::Unit(i));
        const auto j = point_jacobian(body.link, x, body.pose * local).J;
        arm = std::max(arm, j.rightCols<6>().cwiseAbs().maxCoeff());
        height = std::max(height, j.row(2).cwiseAbs().maxCoeff());
      }
    }
    body.base = arm < 1e-12;
    body.planar = height < 1e-12;
    body.floor = body.panel || primitive.link.rfind("wrist_", 0) == 0;
    body.self_name = "self/" + primitive.link;
    body.link_name = body.panel ? "panel" : primitive.link;
    body.self_arm = !body.base && !body.panel &&
                    std::find(config.self_links.begin(), config.self_links.end(), primitive.link) != config.self_links.end();
    impl->bodies.push_back(std::move(body));
  }
  // A payload box that is not the generated model's panel (another payload, or one moved on the
  // flange) would silently get no panel rows, floor rows or panel Jacobians.
  if (payload_box && impl->panel < 0)
    throw std::invalid_argument("CollisionModel: the payload box does not match the generated model's panel frame "
                                "(regenerate the model for this payload)");
  impl_ = std::move(impl);
}

std::vector<std::pair<std::string, double>> CollisionModel::self_distances(const State & x) const
{
  const Impl & m = *impl_;
  std::map<std::string, double> by_link;
  if (m.panel < 0) return {};
  Placed panel;
  m.place(m.bodies[static_cast<size_t>(m.panel)], x, panel);
  Placed other;
  for (size_t b = 0; b < m.bodies.size(); ++b) {
    const Body & body = m.bodies[b];
    if (body.base || body.panel) continue;
    m.place(body, x, other);
    const double d = exact(*m.bodies[static_cast<size_t>(m.panel)].geometry, panel.solid, *body.geometry, other.solid).distance;
    auto [it, inserted] = by_link.emplace(body.link_name, d);
    if (!inserted) it->second = std::min(it->second, d);
  }
  return {by_link.begin(), by_link.end()};
}

std::optional<NearestPair> CollisionModel::nearest_self(const State & x) const
{
  const Impl & m = *impl_;
  std::optional<NearestPair> best;
  for (const auto & [name, d] : self_distances(x)) {
    if (std::find(m.config.self_links.begin(), m.config.self_links.end(), name) == m.config.self_links.end()) continue;
    if (!best || d < best->distance) best = NearestPair{d, "panel", name};
  }
  return best;
}

bool CollisionModel::is_base(int primitive) const
{
  return primitive >= 0 && primitive < static_cast<int>(impl_->bodies.size()) &&
         impl_->bodies[static_cast<size_t>(primitive)].base;
}

const CollisionConfig & CollisionModel::config() const
{
  return impl_->config;
}

std::vector<DistanceRow> CollisionModel::rows(const std::vector<State> & nominal, double dt, double now,
                                              const World & world, CollisionStats * stats) const
{
  const auto start = std::chrono::steady_clock::now();
  const Impl & m = *impl_;
  const CollisionConfig & c = m.config;
  const size_t count = m.bodies.size();
  const size_t stages = nominal.size();
  const size_t budget = static_cast<size_t>(std::max(0, c.pairs_per_stage));

  // Broad phase: every primitive at every stage, and the box swept by the whole robot.
  std::vector<Placed> placed(stages * count);
  Aabb sweep{Vector3d::Constant(kInf), Vector3d::Constant(-kInf)};
  for (size_t k = 0; k < stages; ++k) {
    for (size_t b = 0; b < count; ++b) {
      Placed & p = placed[k * count + b];
      m.place(m.bodies[b], nominal[k], p);
      sweep.lo = sweep.lo.cwiseMin(p.box.lo);
      sweep.hi = sweep.hi.cwiseMax(p.box.hi);
    }
  }

  // Only the obstacles within activation of the swept box can make a pair: the known boxes are
  // filtered once, the movers per stage.
  const auto far = [&](const Aabb & box) { return gap(sweep, box) > c.activation_m; };
  const std::vector<Obstacle> near = near_statics(world, far);

  std::vector<DistanceRow> out;
  out.reserve(stages * std::min<size_t>(budget, 64));
  std::vector<Candidate> candidates;
  std::vector<ObstacleBox> movers;
  std::vector<Obstacle> obstacles;
  int active = 0, dropped = 0, self_pairs = 0;
  for (size_t k = 0; k < stages; ++k) {
    const State & x = nominal[k];
    const Placed * at = &placed[k * count];
    stage_obstacles(world, static_cast<double>(k) * dt, now, near, far, movers, obstacles);
    candidates.clear();

    // Candidate pairs whose lower bound is within the activation distance.
    for (size_t o = 0; o < obstacles.size(); ++o) {
      const Aabb & box = obstacles[o].aabb;
      for (size_t b = 0; b < count; ++b) {
        if (m.bodies[b].planar && vertically_apart(at[b].box, box)) continue;  // unreachable by driving
        if (gap(at[b].box, box) > c.activation_m) continue;
        const double bound = lower_bound(at[b].solid, obstacles[o].solid);
        if (bound <= c.activation_m)
          candidates.push_back({bound, Kind::Obstacle, static_cast<int>(b), static_cast<int>(o), false, {}});
      }
    }
    if (m.panel >= 0) {
      double bound = kInf;
      for (size_t b = 0; b < count; ++b)
        if (m.bodies[b].base) bound = std::min(bound, lower_bound(at[m.panel].solid, at[b].solid));
      if (bound <= c.activation_m) candidates.push_back({bound, Kind::Self, m.panel, -1, false, {}});
    }
    for (size_t b = 0; b < count; ++b) {
      if (!m.bodies[b].floor || at[b].box.lo.z() - c.floor_top_m > c.activation_m) continue;
      const Vector3d p = support_point(at[b].solid, -Vector3d::UnitZ());  // lowest point
      const double distance = p.z() - c.floor_top_m;
      candidates.push_back({distance, Kind::Floor, static_cast<int>(b), -1, true,
                            {distance, Vector3d::UnitZ(), p, Vector3d(p.x(), p.y(), c.floor_top_m)}});
    }

    auto evaluate = [&](Candidate & cand) {
      if (cand.evaluated) return;
      if (cand.kind == Kind::Self) {
        m.evaluate_self(at, cand);
        return;
      }
      const Solid & solid = obstacles[static_cast<size_t>(cand.other)].solid;
      const fcl::Boxd shape(2.0 * solid.dims);
      cand.contact = exact(*m.bodies[static_cast<size_t>(cand.body)].geometry, at[cand.body].solid, shape, solid);
      cand.bound = cand.contact.distance;
      cand.evaluated = true;
    };

    // Exact distances in order of the lower bounds until the budget's closest pairs are known.
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate & a, const Candidate & b) { return a.bound < b.bound; });
    std::priority_queue<double> closest;  // the budget smallest exact distances within activation
    size_t next = 0;
    for (; next < candidates.size(); ++next) {
      if (budget == 0 || (closest.size() == budget && closest.top() <= candidates[next].bound)) break;
      evaluate(candidates[next]);
      if (candidates[next].bound > c.activation_m) continue;
      closest.push(candidates[next].bound);
      if (closest.size() > budget) closest.pop();
    }
    // The others only need to be known within or beyond activation, for the dropped count:
    // an upper bound within activation settles it, otherwise the exact distance.
    int unevaluated_within = 0;
    for (size_t i = next; i < candidates.size(); ++i) {
      Candidate & cand = candidates[i];
      if (cand.kind == Kind::Obstacle &&
          surely_within(at[cand.body].solid, obstacles[static_cast<size_t>(cand.other)].solid, c.activation_m)) {
        ++unevaluated_within;
        continue;
      }
      evaluate(cand);
    }

    std::vector<const Candidate *> within;
    for (const auto & cand : candidates)
      if (cand.evaluated && cand.bound <= c.activation_m) within.push_back(&cand);
    std::sort(within.begin(), within.end(),
              [](const Candidate * a, const Candidate * b) { return a->bound < b->bound; });
    const size_t keep = std::min(budget, within.size());
    active += static_cast<int>(keep);
    dropped += static_cast<int>(within.size() - keep) + unevaluated_within;

    for (size_t i = 0; i < keep; ++i) {
      const Candidate & cand = *within[i];
      const Body & body = m.bodies[static_cast<size_t>(cand.body)];
      DistanceRow row;
      row.stage = static_cast<int>(k);
      row.link = body.link;
      row.primitive = cand.body;
      row.distance = cand.contact.distance;
      row.base = cand.kind == Kind::Obstacle && body.base;
      row.margin = row.base ? c.base_margin_m : c.margin_m;
      Eigen::Matrix<double, 3, kNx> j = m.jacobian(body, at[cand.body], x, cand.contact.p_robot);
      switch (cand.kind) {
        case Kind::Obstacle:
          row.obstacle = obstacles[static_cast<size_t>(cand.other)].box->name;
          break;
        case Kind::Self: {
          const Body & base = m.bodies[static_cast<size_t>(cand.other)];
          row.obstacle = base.self_name;
          j -= m.jacobian(base, at[cand.other], x, cand.contact.p_other);
          break;
        }
        case Kind::Floor:
          row.obstacle = "floor";
          break;
      }
      row.gradient = cand.contact.n.transpose() * j;
      out.push_back(std::move(row));
    }

    // The panel against the arm links (collision.self_rows): the closest primitive of each listed link within
    // activation, the self_pairs_per_stage closest links, on their own budget.
    if (c.self_rows && m.panel >= 0 && c.self_pairs_per_stage > 0) {
      const Body & panel_body = m.bodies[static_cast<size_t>(m.panel)];
      std::map<std::string, std::pair<int, Contact>> closest_of_link;
      for (size_t b = 0; b < count; ++b) {
        if (!m.bodies[b].self_arm) continue;
        if (lower_bound(at[m.panel].solid, at[b].solid) > c.activation_m) continue;
        const Contact contact = exact(*panel_body.geometry, at[m.panel].solid, *m.bodies[b].geometry, at[b].solid);
        if (contact.distance > c.activation_m) continue;
        auto it = closest_of_link.find(m.bodies[b].link_name);
        if (it == closest_of_link.end() || contact.distance < it->second.second.distance)
          closest_of_link[m.bodies[b].link_name] = {static_cast<int>(b), contact};
      }
      std::vector<std::pair<int, Contact>> arm(closest_of_link.size());
      std::transform(closest_of_link.begin(), closest_of_link.end(), arm.begin(), [](const auto & e) { return e.second; });
      std::sort(arm.begin(), arm.end(), [](const auto & a, const auto & b) { return a.second.distance < b.second.distance; });
      const size_t keep_arm = std::min(arm.size(), static_cast<size_t>(c.self_pairs_per_stage));
      for (size_t i = 0; i < keep_arm; ++i) {
        const auto & [b, contact] = arm[i];
        DistanceRow row;
        row.stage = static_cast<int>(k);
        row.link = panel_body.link;
        row.primitive = m.panel;
        row.distance = contact.distance;
        row.margin = c.margin_m;
        row.obstacle = m.bodies[static_cast<size_t>(b)].self_name;
        Eigen::Matrix<double, 3, kNx> j = m.jacobian(panel_body, at[m.panel], x, contact.p_robot);
        j -= m.jacobian(m.bodies[static_cast<size_t>(b)], at[b], x, contact.p_other);
        row.gradient = contact.n.transpose() * j;
        out.push_back(std::move(row));
      }
      self_pairs += static_cast<int>(keep_arm);
      active += static_cast<int>(keep_arm);
      dropped += static_cast<int>(arm.size() - keep_arm);
    }
  }

  if (stats) {
    stats->self_pairs = self_pairs;
    stats->active_pairs = active;
    stats->dropped_pairs = dropped;
    stats->compute_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  }
  return out;
}

std::optional<NearestPair> CollisionModel::nearest(const State & x, const std::vector<ObstacleBox> & obstacles) const
{
  const Impl & m = *impl_;
  std::optional<NearestPair> best;
  Placed placed;
  for (const ObstacleBox & obstacle : obstacles) {
    const Solid solid{obstacle.pose, mmg::Shape::Box, obstacle.size / 2.0};
    const Aabb solid_box = aabb(solid);
    const fcl::Boxd shape(obstacle.size);
    for (const Body & body : m.bodies) {
      m.place(body, x, placed);
      if (best) {
        const double apart = gap(placed.box, solid_box);  // unsigned: settles separated pairs only
        if ((apart > 0.0 && apart >= best->distance) || lower_bound(placed.solid, solid) >= best->distance) continue;
      }
      const double distance = exact(*body.geometry, placed.solid, shape, solid).distance;
      if (!best || distance < best->distance) best = NearestPair{distance, body.link_name, obstacle.name};
    }
  }
  return best;
}

double CollisionModel::pair_distance(const State & x, const DistanceRow & row, const World & world, double now) const
{
  const Impl & m = *impl_;
  const CollisionConfig & c = m.config;
  const int count = static_cast<int>(m.bodies.size());
  // The row's own primitive, or every primitive of its link.
  auto robot = [&](int b) {
    return row.primitive >= 0 ? b == row.primitive : m.bodies[static_cast<size_t>(b)].link == row.link;
  };
  double best = kInf;
  if (row.obstacle == "floor") {
    Placed placed;
    for (int b = 0; b < count; ++b) {
      if (!robot(b) || !m.bodies[static_cast<size_t>(b)].floor) continue;
      m.place(m.bodies[static_cast<size_t>(b)], x, placed);
      best = std::min(best, support_point(placed.solid, -Vector3d::UnitZ()).z() - c.floor_top_m);
    }
    return best;
  }
  if (row.obstacle.rfind("self/", 0) == 0) {
    if (m.panel < 0) return kInf;
    // The panel against an arm link (collision.self_rows): its primitives of that link.
    bool arm = false;
    Placed panel, other;
    m.place(m.bodies[static_cast<size_t>(m.panel)], x, panel);
    for (int b = 0; b < count; ++b) {
      const Body & body = m.bodies[static_cast<size_t>(b)];
      if (!body.self_arm || body.self_name != row.obstacle) continue;
      arm = true;
      m.place(body, x, other);
      best = std::min(best, exact(*m.bodies[static_cast<size_t>(m.panel)].geometry, panel.solid, *body.geometry,
                                  other.solid).distance);
    }
    if (arm) return best;
    // The panel against its closest base primitive, as the row was made.
    std::vector<Placed> at(static_cast<size_t>(count));
    for (int b = 0; b < count; ++b) m.place(m.bodies[static_cast<size_t>(b)], x, at[static_cast<size_t>(b)]);
    Candidate candidate{kInf, Kind::Self, m.panel, -1, false, {}};
    m.evaluate_self(at.data(), candidate);
    return candidate.bound;
  }
  // The obstacles of world.at(0, now), without copying the known boxes.
  auto against = [&](const ObstacleBox & obstacle) {
    if (obstacle.name != row.obstacle) return;
    const Solid solid{obstacle.pose, mmg::Shape::Box, obstacle.size / 2.0};
    const fcl::Boxd shape(obstacle.size);
    Placed placed;
    for (int b = 0; b < count; ++b) {
      if (!robot(b)) continue;
      const Body & body = m.bodies[static_cast<size_t>(b)];
      m.place(body, x, placed);
      best = std::min(best, exact(*body.geometry, placed.solid, shape, solid).distance);
    }
  };
  for (const ObstacleBox & obstacle : world.statics()) against(obstacle);
  for (const ObstacleBox & obstacle : world.movers_at(0.0, now)) against(obstacle);
  return best;
}

double CollisionModel::clearance_with_joints(const std::vector<State> & nominal,
                                             const Eigen::Matrix<double, 6, 1> & joints, double dt, double now,
                                             const World & world) const
{
  const Impl & m = *impl_;
  const double activation = m.config.activation_m;
  double best = activation;

  // The arm and panel primitives at every stage, and the box they sweep.
  std::vector<Placed> placed;
  Aabb sweep{Vector3d::Constant(kInf), Vector3d::Constant(-kInf)};
  for (const State & nominal_x : nominal) {
    State x = nominal_x;
    x.tail<6>() = joints;
    for (const Body & body : m.bodies) {
      if (body.base) continue;
      m.place(body, x, placed.emplace_back());
      sweep.lo = sweep.lo.cwiseMin(placed.back().box.lo);
      sweep.hi = sweep.hi.cwiseMax(placed.back().box.hi);
    }
  }
  // An obstacle whose box is apart from the sweep by activation is at least as far apart from
  // each primitive's box, which settles it below as it never lowers `best` (at most activation).
  const auto far = [&](const Aabb & box) {
    const double apart = gap(sweep, box);
    return apart > 0.0 && apart >= activation;
  };
  const std::vector<Obstacle> near = near_statics(world, far);

  std::vector<ObstacleBox> movers;
  std::vector<Obstacle> obstacles;
  const Placed * at = placed.data();
  for (size_t k = 0; k < nominal.size(); ++k) {
    stage_obstacles(world, static_cast<double>(k) * dt, now, near, far, movers, obstacles);
    for (const Body & body : m.bodies) {
      if (body.base) continue;
      const Placed & p = *at++;
      for (const Obstacle & o : obstacles) {
        const double apart = gap(p.box, o.aabb);  // unsigned: settles separated pairs only
        if ((apart > 0.0 && apart >= best) || lower_bound(p.solid, o.solid) >= best) continue;
        const fcl::Boxd shape(o.box->size);
        best = std::min(best, exact(*body.geometry, p.solid, shape, o.solid).distance);
      }
    }
  }
  return best;
}
std::vector<double> CollisionModel::clearance_per_stage(const std::vector<State> & nominal,
                                                       const Eigen::Matrix<double, 6, 1> & joints, double dt,
                                                       double now, const World & world) const
{
  const Impl & m = *impl_;
  const double activation = m.config.activation_m;
  std::vector<double> per_stage(nominal.size(), activation);

  // The arm and panel primitives at every stage, and the box they sweep.
  std::vector<Placed> placed;
  Aabb sweep{Vector3d::Constant(kInf), Vector3d::Constant(-kInf)};
  for (const State & nominal_x : nominal) {
    State x = nominal_x;
    x.tail<6>() = joints;
    for (const Body & body : m.bodies) {
      if (body.base) continue;
      m.place(body, x, placed.emplace_back());
      sweep.lo = sweep.lo.cwiseMin(placed.back().box.lo);
      sweep.hi = sweep.hi.cwiseMax(placed.back().box.hi);
    }
  }
  // An obstacle whose box is apart from the sweep by activation is at least as far apart from
  // each primitive's box, which settles it below as it never lowers `best` (at most activation).
  const auto far = [&](const Aabb & box) {
    const double apart = gap(sweep, box);
    return apart > 0.0 && apart >= activation;
  };
  const std::vector<Obstacle> near = near_statics(world, far);

  std::vector<ObstacleBox> movers;
  std::vector<Obstacle> obstacles;
  const Placed * at = placed.data();
  for (size_t k = 0; k < nominal.size(); ++k) {
    stage_obstacles(world, static_cast<double>(k) * dt, now, near, far, movers, obstacles);
    double & best = per_stage[k];
    for (const Body & body : m.bodies) {
      if (body.base) continue;
      const Placed & p = *at++;
      for (const Obstacle & o : obstacles) {
        const double apart = gap(p.box, o.aabb);  // unsigned: settles separated pairs only
        if ((apart > 0.0 && apart >= best) || lower_bound(p.solid, o.solid) >= best) continue;
        const fcl::Boxd shape(o.box->size);
        best = std::min(best, exact(*body.geometry, p.solid, shape, o.solid).distance);
      }
    }
  }
  return per_stage;
}
}  // namespace mobile_manipulator_mpc
