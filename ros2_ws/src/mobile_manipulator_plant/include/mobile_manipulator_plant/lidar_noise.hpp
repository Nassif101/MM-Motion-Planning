#pragma once
// Gaussian lidar range noise as Unity draws it (MotionPlanningSim.Sensors.LidarRangeNoise.Sample): a pure function of
// (seed, scan, index), so every point of every scan gets an independent draw and a run repeats exactly.
#include <cstdint>

namespace mobile_manipulator_plant
{
float lidar_range_noise(uint32_t seed, uint32_t scan, int index, float sigma);
// A uniform [0, 1) draw for the same (seed, scan, index), independent of the noise (a salted seed): the dropouts.
float lidar_uniform(uint32_t seed, uint32_t scan, int index);
}  // namespace mobile_manipulator_plant
