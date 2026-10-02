#include "mobile_manipulator_manipulation/octomap_box.hpp"

#include <cmath>

namespace mobile_manipulator_manipulation
{
namespace
{
// Calls visit(centre) for every occupied leaf whose cube touches the box: surface returns fill
// voxels straddling its faces.
template<typename Visit>
void for_voxels_touching(const octomap::OcTree & tree, const Box & box, Visit visit)
{
  const double pad = tree.getResolution() / 2.0;
  const Eigen::Vector3d low = box.center - box.size / 2.0, high = box.center + box.size / 2.0;
  for (auto it = tree.begin_leafs_bbx(octomap::point3d(low.x() - pad, low.y() - pad, low.z() - pad),
                                      octomap::point3d(high.x() + pad, high.y() + pad, high.z() + pad));
       it != tree.end_leafs_bbx(); ++it)
  {
    if (!tree.isNodeOccupied(*it)) continue;
    const auto c = it.getCoordinate();
    const Eigen::Vector3d centre(c.x(), c.y(), c.z());
    if (((centre - box.center).cwiseAbs().array() <= (box.size / 2.0).array() + it.getSize() / 2.0 + 1e-4).all()) {
      visit(centre);
    }
  }
}
}  // namespace

std::size_t occupied_voxels_in(const octomap::OcTree & tree, const Box & box)
{
  std::size_t count = 0;
  for_voxels_touching(tree, box, [&](const Eigen::Vector3d &) { count += 1; });
  return count;
}

double highest_voxel_in(const octomap::OcTree & tree, const Box & box)
{
  double highest = std::nan("");
  for_voxels_touching(tree, box, [&](const Eigen::Vector3d & c) {
    if (std::isnan(highest) || c.z() > highest) highest = c.z();
  });
  return highest;
}
}  // namespace mobile_manipulator_manipulation
