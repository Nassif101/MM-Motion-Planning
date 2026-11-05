#include "mobile_manipulator_plant/static_bvh.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace mobile_manipulator_plant
{
namespace
{
constexpr int kLeafSize = 4;
constexpr double kMinVal = 1e-15;  // MuJoCo's mjMINVAL

// MuJoCo 3.17.0 engine_ray.c, ported operation for operation so a hit equals mj_ray's.
void ray_map(const double pos[3], const double mat[9], const double pnt[3], const double vec[3], double lpnt[3], double lvec[3])
{
  const double dif[3] = {pnt[0] - pos[0], pnt[1] - pos[1], pnt[2] - pos[2]};
  lpnt[0] = mat[0] * dif[0] + mat[3] * dif[1] + mat[6] * dif[2];
  lpnt[1] = mat[1] * dif[0] + mat[4] * dif[1] + mat[7] * dif[2];
  lpnt[2] = mat[2] * dif[0] + mat[5] * dif[1] + mat[8] * dif[2];
  lvec[0] = mat[0] * vec[0] + mat[3] * vec[1] + mat[6] * vec[2];
  lvec[1] = mat[1] * vec[0] + mat[4] * vec[1] + mat[7] * vec[2];
  lvec[2] = mat[2] * vec[0] + mat[5] * vec[1] + mat[8] * vec[2];
}

double ray_quad(double a, double b, double c)
{
  double det = b * b - a * c;
  if (det < 0 || a < kMinVal) return -1;
  det = std::sqrt(det);
  const double x0 = (-b - det) / a, x1 = (-b + det) / a;
  if (x0 >= 0) return x0;
  if (x1 >= 0) return x1;
  return -1;
}

double ray_sphere(const double pos[3], double dist_sqr, const double pnt[3], const double vec[3])
{
  const double dif[3] = {pnt[0] - pos[0], pnt[1] - pos[1], pnt[2] - pos[2]};
  const double a = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
  const double b = vec[0] * dif[0] + vec[1] * dif[1] + vec[2] * dif[2];
  const double c = dif[0] * dif[0] + dif[1] * dif[1] + dif[2] * dif[2] - dist_sqr;
  return ray_quad(a, b, c);
}

double ray_box(const double pos[3], const double mat[9], const double size[3], const double pnt[3], const double vec[3])
{
  const double ssz = size[0] * size[0] + size[1] * size[1] + size[2] * size[2];
  if (ray_sphere(pos, ssz, pnt, vec) < 0) return -1;
  const int iface[3][2] = {{1, 2}, {0, 2}, {0, 1}};
  double lpnt[3], lvec[3];
  ray_map(pos, mat, pnt, vec, lpnt, lvec);
  double x = -1;
  for (int i = 0; i < 3; i++) {
    if (std::abs(lvec[i]) > kMinVal) {
      for (int side = -1; side <= 1; side += 2) {
        const double sol = (side * size[i] - lpnt[i]) / lvec[i];
        if (sol >= 0) {
          const double p0 = lpnt[iface[i][0]] + sol * lvec[iface[i][0]];
          const double p1 = lpnt[iface[i][1]] + sol * lvec[iface[i][1]];
          if (std::abs(p0) <= size[iface[i][0]] && std::abs(p1) <= size[iface[i][1]]) {
            if (x < 0 || sol < x) x = sol;
          }
        }
      }
    }
  }
  return x;
}

double ray_plane(const double pos[3], const double mat[9], const double size[3], const double pnt[3], const double vec[3])
{
  double lpnt[3], lvec[3];
  ray_map(pos, mat, pnt, vec, lpnt, lvec);
  if (lvec[2] > -kMinVal) return -1;
  const double x = -lpnt[2] / lvec[2];
  if (x < 0) return -1;
  const double p0 = lpnt[0] + x * lvec[0];
  const double p1 = lpnt[1] + x * lvec[1];
  if ((size[0] <= 0 || std::abs(p0) <= size[0]) && (size[1] <= 0 || std::abs(p1) <= size[1])) return x;
  return -1;
}

// The entry and exit distances of the ray through an axis-aligned box (tmin may be negative when the origin is
// inside); false when it misses.
bool slab(const Eigen::Vector3d & lo, const Eigen::Vector3d & hi, const Eigen::Vector3d & o, const Eigen::Vector3d & inv,
          double & tmin, double & tmax)
{
  tmin = -std::numeric_limits<double>::infinity();
  tmax = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    double a = (lo(i) - o(i)) * inv(i), b = (hi(i) - o(i)) * inv(i);
    if (std::isnan(a) || std::isnan(b)) {  // a zero direction component with the origin on the slab plane
      if (o(i) < lo(i) || o(i) > hi(i)) return false;
      continue;
    }
    if (a > b) std::swap(a, b);
    tmin = std::max(tmin, a);
    tmax = std::min(tmax, b);
  }
  return tmin <= tmax && tmax >= 0.0;
}
}  // namespace

StaticBvh::StaticBvh(const mjModel * m, const mjData * d)
{
  for (int g = 0; g < m->ngeom; ++g) {
    if (m->body_weldid[m->geom_bodyid[g]] != 0) continue;  // MuJoCo's "static": welded to the world
    // mj_ray skips invisible geoms; so does the tree
    if ((m->geom_matid[g] < 0 && m->geom_rgba[4 * g + 3] == 0) || (m->geom_matid[g] >= 0 && m->mat_rgba[4 * m->geom_matid[g] + 3] == 0))
      continue;
    Box b;
    b.geom = g;
    for (int i = 0; i < 3; ++i) {
      b.pos[i] = d->geom_xpos[3 * g + i];
      b.size[i] = m->geom_size[3 * g + i];
    }
    for (int i = 0; i < 9; ++i) b.mat[i] = d->geom_xmat[9 * g + i];
    if (m->geom_type[g] == mjGEOM_PLANE) {
      planes_.push_back(b);
      continue;
    }
    if (m->geom_type[g] != mjGEOM_BOX) {
      const char * name = mj_id2name(m, mjOBJ_GEOM, g);
      throw std::runtime_error("StaticBvh: static geom " + std::string(name ? name : std::to_string(g)) + " is neither a box nor a plane");
    }
    // axis-aligned bounds of the oriented box: centre +- |R| half sizes
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(b.mat);
    const Eigen::Vector3d half = R.cwiseAbs() * Eigen::Map<const Eigen::Vector3d>(b.size);
    b.lo = Eigen::Map<const Eigen::Vector3d>(b.pos) - half;
    b.hi = Eigen::Map<const Eigen::Vector3d>(b.pos) + half;
    boxes_.push_back(b);
  }
  if (!boxes_.empty()) build(0, static_cast<int>(boxes_.size()));
}

int StaticBvh::build(int first, int count)
{
  const int index = static_cast<int>(nodes_.size());
  nodes_.emplace_back();
  Eigen::Vector3d lo = boxes_[first].lo, hi = boxes_[first].hi;
  for (int i = first; i < first + count; ++i) {
    lo = lo.cwiseMin(boxes_[i].lo);
    hi = hi.cwiseMax(boxes_[i].hi);
  }
  nodes_[index].lo = lo;
  nodes_[index].hi = hi;
  if (count <= kLeafSize) {
    nodes_[index].first = first;
    nodes_[index].count = count;
    return index;
  }
  // split at the median centre along the longest axis
  int axis = 0;
  (hi - lo).maxCoeff(&axis);
  const int mid = first + count / 2;
  std::nth_element(boxes_.begin() + first, boxes_.begin() + mid, boxes_.begin() + first + count,
                   [axis](const Box & a, const Box & b) { return a.lo(axis) + a.hi(axis) < b.lo(axis) + b.hi(axis); });
  const int left = build(first, mid - first);
  const int right = build(mid, first + count - mid);
  nodes_[index].left = left;
  nodes_[index].right = right;
  return index;
}

double StaticBvh::ray(const Eigen::Vector3d & origin, const Eigen::Vector3d & dir, int & geom) const
{
  double best = -1;
  geom = -1;
  const double pnt[3] = {origin.x(), origin.y(), origin.z()}, vec[3] = {dir.x(), dir.y(), dir.z()};
  auto consider = [&](const Box & b, bool plane) {
    const double t = plane ? ray_plane(b.pos, b.mat, b.size, pnt, vec) : ray_box(b.pos, b.mat, b.size, pnt, vec);
    if (t >= 0 && (best < 0 || t < best)) {
      best = t;
      geom = b.geom;
    }
  };
  for (const Box & p : planes_) consider(p, true);
  if (nodes_.empty()) return best;
  const Eigen::Vector3d inv = dir.cwiseInverse();
  int stack[64];
  int top = 0;
  stack[top++] = 0;
  while (top > 0) {
    const Node & node = nodes_[stack[--top]];
    double tmin, tmax;
    if (!slab(node.lo, node.hi, origin, inv, tmin, tmax)) continue;
    if (best >= 0 && tmin > best) continue;  // everything inside starts beyond the nearest hit so far
    if (node.left < 0) {
      for (int i = node.first; i < node.first + node.count; ++i) consider(boxes_[i], false);
      continue;
    }
    // the nearer child last onto the stack, so it is searched first
    double l0, l1, r0, r1;
    const bool hit_left = slab(nodes_[node.left].lo, nodes_[node.left].hi, origin, inv, l0, l1);
    const bool hit_right = slab(nodes_[node.right].lo, nodes_[node.right].hi, origin, inv, r0, r1);
    if (hit_left && hit_right) {
      if (l0 <= r0) {
        stack[top++] = node.right;
        stack[top++] = node.left;
      } else {
        stack[top++] = node.left;
        stack[top++] = node.right;
      }
    } else if (hit_left) {
      stack[top++] = node.left;
    } else if (hit_right) {
      stack[top++] = node.right;
    }
  }
  return best;
}
}  // namespace mobile_manipulator_plant
