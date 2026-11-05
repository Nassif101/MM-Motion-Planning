#pragma once
// A bounding volume hierarchy over the model's static geometry (MuJoCo plant spec Section 5.3, decided at CP2): the
// boxes and planes of bodies welded to the world (MuJoCo's own "static": the site, the scenario boxes, the ground),
// built once per model. Leaves test the ray exactly as MuJoCo's ray_box/ray_plane do, so a hit equals mj_ray's; the
// tree only skips geoms the ray cannot hit, or cannot hit before a nearer one.
#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <vector>

namespace mobile_manipulator_plant
{
class StaticBvh
{
public:
  // The static geoms of `m` at their poses in `d` (kinematics computed). Throws std::runtime_error for a static geom that
  // is neither a box nor a plane.
  StaticBvh(const mjModel * m, const mjData * d);

  // The nearest hit along origin + t dir (t >= 0), as mj_ray would return it among the static geoms: the distance and
  // the geom, or -1 and geom -1.
  double ray(const Eigen::Vector3d & origin, const Eigen::Vector3d & dir, int & geom) const;

  int geoms() const { return static_cast<int>(boxes_.size() + planes_.size()); }

private:
  struct Box
  {
    int geom;
    double pos[3], mat[9], size[3];
    Eigen::Vector3d lo, hi;  // its axis-aligned bounds
  };
  struct Node
  {
    Eigen::Vector3d lo, hi;
    int left = -1, right = -1, first = 0, count = 0;  // children, or a leaf's range in boxes_
  };
  int build(int first, int count);

  std::vector<Box> boxes_, planes_;
  std::vector<Node> nodes_;
};
}  // namespace mobile_manipulator_plant
