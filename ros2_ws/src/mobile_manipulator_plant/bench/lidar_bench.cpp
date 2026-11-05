// The lidar's cost on the full V2 site (MuJoCo plant plan Task 5 Step 4, spec Section 5.3): 50 scans (20000 rays,
// mj_multiRay) at each of five site poses with the arm in carry, in both scan modes; mean and p99 ms per scan. The BVH
// path is needed when the p99 exceeds 25 % of the 100 ms scan period.
//   lidar_bench PROFILE_YAML SITE_MJCF PATTERN_BIN SHARE_ROOT
#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <vector>

#include "mobile_manipulator_plant/lidar.hpp"
#include "mobile_manipulator_plant/model_builder.hpp"

namespace mmp = mobile_manipulator_plant;

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr << "usage: lidar_bench PROFILE_YAML SITE_MJCF PATTERN_BIN SHARE_ROOT\n";
    return 2;
  }
  const std::string root = argv[4];
  const auto profile = mmp::load_profile(argv[1], [&](const std::string & p) { return root + "/" + p; });
  const mmp::ScanPattern pattern = mmp::load_pattern(argv[3]);
  mjModel * m = mmp::build_model(profile, argv[2], {});
  mjData * d = mj_makeData(m);
  struct Pose { const char * name; double x, y, yaw; };
  const std::vector<Pose> poses{{"open yard", -5.0, -5.5, 0.0}, {"spine", 0.0, 0.0, 0.0}, {"building bay", 8.0, 8.0, 1.57},
                                {"store door", 14.5, 9.0, 0.0}, {"forecourt", 12.0, 0.0, 0.8}};
  std::vector<double> all, all_sweep;
  for (const Pose & pose : poses) {
    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "base_x")]] = pose.x;
    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "base_y")]] = pose.y;
    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "base_yaw")]] = pose.yaw;
    for (int i = 0; i < 6; ++i) d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, profile.arm_joints[i].c_str())]] = profile.poses.at("carry")(i);
    mj_fwdPosition(m, d);
    for (const auto mode : {mmp::ScanMode::Instant, mmp::ScanMode::Sweep}) {
      mmp::Lidar lidar(profile.lidar, pattern, mode);
      lidar.attach(m, 1);
      std::vector<double> ms;
      size_t points = 0;
      for (int s = 0; s < 50; ++s) {
        // a scan: one call (instant) or 50 slices of 2 ms (sweep, the default)
        const auto t0 = std::chrono::steady_clock::now();
        for (int k = 1; k <= 50; ++k) lidar.advance(m, d, 0.1 * s + 0.002 * k, 0.002);
        const auto scan = lidar.take_scan();
        ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        points += scan ? scan->points.size() : 0;
      }
      std::sort(ms.begin(), ms.end());
      auto & bucket = mode == mmp::ScanMode::Sweep ? all_sweep : all;
      bucket.insert(bucket.end(), ms.begin(), ms.end());
      std::cout << pose.name << (mode == mmp::ScanMode::Sweep ? " (sweep)" : " (instant)") << ": mean "
                << std::accumulate(ms.begin(), ms.end(), 0.0) / ms.size() << " ms, p99 " << ms[ms.size() * 99 / 100]
                << " ms, " << points / 50 << " points per scan\n";
    }
  }
  for (auto * bucket : {&all, &all_sweep}) {
    std::sort(bucket->begin(), bucket->end());
    const double p99 = (*bucket)[bucket->size() * 99 / 100];
    std::cout << (bucket == &all ? "instant" : "sweep") << ", all poses: mean "
              << std::accumulate(bucket->begin(), bucket->end(), 0.0) / bucket->size() << " ms, p99 " << p99 << " ms ("
              << m->ngeom << " geoms); BVH needed: " << (p99 > 25.0 ? "yes" : "no") << " (limit 25 ms)\n";
  }
  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
