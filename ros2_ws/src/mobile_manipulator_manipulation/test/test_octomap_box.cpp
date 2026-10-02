#include <gtest/gtest.h>

#include <cmath>

#include <octomap/OcTree.h>

#include "mobile_manipulator_manipulation/octomap_box.hpp"

namespace mmm = mobile_manipulator_manipulation;

TEST(OctomapBox, CountsOccupiedVoxelsInsideTheBox)
{
  octomap::OcTree tree(0.05);
  // A 0.2 x 0.2 m patch of occupied voxels at z = 0.5 around (1, 2), one far away, one free.
  for (double x = 0.925; x < 1.1; x += 0.05) {
    for (double y = 1.925; y < 2.1; y += 0.05) tree.updateNode(octomap::point3d(x, y, 0.525), true);
  }
  tree.updateNode(octomap::point3d(5.0, 5.0, 0.5), true);
  tree.updateNode(octomap::point3d(1.0, 2.0, 0.2), false);
  const mmm::Box box{"box", {1.0, 2.0, 0.4}, {0.6, 0.6, 0.8}};
  EXPECT_EQ(mmm::occupied_voxels_in(tree, box), 16u);
  const mmm::Box low{"low", {1.0, 2.0, 0.1}, {0.6, 0.6, 0.2}};
  EXPECT_EQ(mmm::occupied_voxels_in(tree, low), 0u);
}

// Lidar returns from a box face fill voxels that straddle the face; a voxel whose cube
// touches the box counts even when its centre lies just outside.
TEST(OctomapBox, CountsVoxelsStraddlingAFace)
{
  octomap::OcTree tree(0.05);
  tree.updateNode(octomap::point3d(1.31, 2.0, 0.525), true);  // centre 0.01 m outside x = 1.3
  tree.updateNode(octomap::point3d(1.40, 2.0, 0.525), true);  // a full voxel outside
  const mmm::Box box{"box", {1.0, 2.0, 0.4}, {0.6, 0.6, 0.8}};
  EXPECT_EQ(mmm::occupied_voxels_in(tree, box), 1u);
}

TEST(OctomapBox, HighestVoxelShowsHowMuchOfTheBoxWasSeen)
{
  octomap::OcTree tree(0.05);
  for (double z = 0.025; z < 0.9; z += 0.05) tree.updateNode(octomap::point3d(1.0, 2.0, z), true);
  const mmm::Box tall{"tall", {1.0, 2.0, 0.8}, {0.6, 0.6, 1.6}};
  EXPECT_NEAR(mmm::highest_voxel_in(tree, tall), 0.875, 1e-6);
  const mmm::Box empty{"empty", {5.0, 5.0, 0.8}, {0.6, 0.6, 1.6}};
  EXPECT_TRUE(std::isnan(mmm::highest_voxel_in(tree, empty)));
}
