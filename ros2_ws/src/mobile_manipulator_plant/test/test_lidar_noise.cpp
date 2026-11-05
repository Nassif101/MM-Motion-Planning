// The lidar's range noise, a port of Unity's LidarRangeNoise.Sample (MuJoCo plant plan Task 5): the reference values come
// from an independent Python implementation of the C# source (Unity.Mathematics hash, Wang hash, xorshift NextFloat,
// float Box-Muller), and the draws are standard normal.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "mobile_manipulator_plant/lidar_noise.hpp"

namespace mmp = mobile_manipulator_plant;

TEST(LidarNoise, MatchesCSharpReference)
{
  EXPECT_FLOAT_EQ(mmp::lidar_range_noise(0u, 0u, 0, 0.02f), -0.019146068021655083f);
  EXPECT_FLOAT_EQ(mmp::lidar_range_noise(1u, 0u, 0, 0.02f), -0.013839151710271835f);
  EXPECT_FLOAT_EQ(mmp::lidar_range_noise(12345u, 7u, 19999, 0.02f), -0.008897094056010246f);
  EXPECT_FLOAT_EQ(mmp::lidar_range_noise(4000000000u, 123456u, 42, 0.02f), -0.024732373654842377f);
  EXPECT_FLOAT_EQ(mmp::lidar_range_noise(99u, 3u, 5000, 1.0f), -1.7784788608551025f);
}

TEST(LidarNoise, IsStandardNormal)
{
  constexpr int n = 200000;
  std::vector<double> x(n);
  for (int i = 0; i < n; ++i) x[i] = mmp::lidar_range_noise(7u, static_cast<uint32_t>(i / 20000), i % 20000, 1.0f);
  double mean = 0.0, var = 0.0;
  for (double v : x) mean += v / n;
  for (double v : x) var += (v - mean) * (v - mean) / (n - 1);
  EXPECT_NEAR(mean, 0.0, 5.0 / std::sqrt(n));
  EXPECT_NEAR(std::sqrt(var), 1.0, 0.01);
  // Kolmogorov-Smirnov against N(0, 1): D below the 1 % critical value 1.63 / sqrt(n)
  std::sort(x.begin(), x.end());
  double d = 0.0;
  for (int i = 0; i < n; ++i) {
    const double cdf = 0.5 * std::erfc(-x[i] / std::sqrt(2.0));
    d = std::max({d, std::abs(cdf - static_cast<double>(i) / n), std::abs(cdf - static_cast<double>(i + 1) / n)});
  }
  EXPECT_LT(d, 1.63 / std::sqrt(n));
}

TEST(LidarNoise, IndependentAcrossScans)
{
  // the bug it replaces in UnitySensors: the same sequence every scan
  int same = 0;
  for (int i = 0; i < 1000; ++i) same += mmp::lidar_range_noise(5u, 0u, i, 1.0f) == mmp::lidar_range_noise(5u, 1u, i, 1.0f);
  EXPECT_EQ(same, 0);
}

TEST(LidarNoise, UniformIsUniformAndIndependentOfTheNoise)
{
  constexpr int n = 100000;
  int below = 0, same_sign = 0;
  for (int i = 0; i < n; ++i) {
    const float u = mmp::lidar_uniform(9u, 0u, i);
    ASSERT_GE(u, 0.0f);
    ASSERT_LT(u, 1.0f);
    below += u < 0.3f;
    same_sign += (u < 0.5f) == (mmp::lidar_range_noise(9u, 0u, i, 1.0f) < 0.0f);
  }
  EXPECT_NEAR(below / static_cast<double>(n), 0.3, 0.01);
  EXPECT_NEAR(same_sign / static_cast<double>(n), 0.5, 0.01);
}
