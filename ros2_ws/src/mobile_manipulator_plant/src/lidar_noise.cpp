#include "mobile_manipulator_plant/lidar_noise.hpp"

#include <cmath>
#include <cstring>

namespace mobile_manipulator_plant
{
namespace
{
// Unity.Mathematics math.hash(uint3)
uint32_t hash3(uint32_t a, uint32_t b, uint32_t c) { return a * 0xCD266C89u + b * 0xF1852A33u + c * 0x77E35E77u + 0x863E3729u; }

// Unity.Mathematics Random.WangHash
uint32_t wang_hash(uint32_t n)
{
  n = (n ^ 61u) ^ (n >> 16);
  n *= 9u;
  n = n ^ (n >> 4);
  n *= 0x27d4eb2du;
  n = n ^ (n >> 15);
  return n;
}

// Unity.Mathematics Random (xorshift32): CreateFromIndex(i) = Random(WangHash(i + 62)), whose constructor draws once.
struct UnityRandom
{
  uint32_t state;
  explicit UnityRandom(uint32_t seed) : state(seed) { next_state(); }
  uint32_t next_state()
  {
    const uint32_t t = state;
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return t;
  }
  float next_float()
  {
    const uint32_t bits = 0x3f800000u | (next_state() >> 9);
    float f;
    std::memcpy(&f, &bits, sizeof f);
    return f - 1.0f;
  }
};

// Unity.Mathematics math.log/sqrt/cos(float) outside Burst: (float)System.Math.X(double)
float flog(float x) { return static_cast<float>(std::log(static_cast<double>(x))); }
float fsqrt(float x) { return static_cast<float>(std::sqrt(static_cast<double>(x))); }
float fcos(float x) { return static_cast<float>(std::cos(static_cast<double>(x))); }
}  // namespace

float lidar_range_noise(uint32_t seed, uint32_t scan, int index, float sigma)
{
  UnityRandom random(wang_hash(hash3(seed, scan, static_cast<uint32_t>(index)) + 62u));
  // Box-Muller; 1 - u keeps the logarithm finite
  const float u1 = 1.0f - random.next_float();
  const float u2 = random.next_float();
  const float two_pi = 2.0f * static_cast<float>(M_PI);
  return sigma * fsqrt(-2.0f * flog(u1)) * fcos(two_pi * u2);
}
float lidar_uniform(uint32_t seed, uint32_t scan, int index)
{
  UnityRandom random(wang_hash(hash3(seed ^ 0x9E3779B9u, scan, static_cast<uint32_t>(index)) + 62u));
  return random.next_float();
}
}  // namespace mobile_manipulator_plant
