#pragma once
// What the lidar Octomap holds inside a known box (no ROS graph).
#include <cstddef>

#include <octomap/OcTree.h>

#include "mobile_manipulator_manipulation/scene_diff.hpp"

namespace mobile_manipulator_manipulation
{
// Occupied leaf voxels whose cube touches the axis-aligned box (same frame as the tree).
std::size_t occupied_voxels_in(const octomap::OcTree & tree, const Box & box);

// Centre height of the highest occupied voxel touching the box (NaN when none): how much of a
// tall obstacle the lidar actually covered.
double highest_voxel_in(const octomap::OcTree & tree, const Box & box);
}  // namespace mobile_manipulator_manipulation
