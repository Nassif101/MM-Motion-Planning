// The Mid-360 lidar model (MuJoCo plant plan Task 5, spec Sections 5 and 8 G4/L).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <vector>

#include "mobile_manipulator_plant/lidar.hpp"
#include "mobile_manipulator_plant/lidar_noise.hpp"
#include "mobile_manipulator_plant/model_builder.hpp"
#include "mobile_manipulator_plant/plant.hpp"

namespace mmp = mobile_manipulator_plant;

namespace
{
std::string source_share(const std::string & package) { return std::string(PACKAGES_SRC) + "/" + package; }
mmp::RobotProfile profile() { return mmp::load_profile(PROFILE_YAML, source_share); }
const mmp::ScanPattern & pattern()
{
  static const mmp::ScanPattern p = mmp::load_pattern(PATTERN_BIN);
  return p;
}

// The open yard with a wall 2 m ahead of the robot and a post to its left.
mmp::ScenarioSetup fixture()
{
  mmp::ScenarioSetup s;
  s.name = "lidar_fixture";
  s.boxes.push_back({"wall", Eigen::Vector3d(-3.0, -5.5, 1.0), Eigen::Vector3d(0.3, 4.0, 2.0), 0.3});
  s.boxes.push_back({"post", Eigen::Vector3d(-4.6, -4.2, 0.75), Eigen::Vector3d(0.2, 0.2, 1.5), 0.0});
  return s;
}

struct Model
{
  mjModel * m;
  mjData * d;
  explicit Model(const mmp::RobotProfile & p, const mmp::Vec6 & q, double x = -5.0, double y = -5.5, double yaw = 0.0,
                 const mmp::ScenarioSetup & s = fixture())
  {
    m = mmp::build_model(p, SITE_MODEL, s);
    d = mj_makeData(m);
    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "base_x")]] = x;
    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "base_y")]] = y;
    d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "base_yaw")]] = yaw;
    for (int i = 0; i < 6; ++i) d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, p.arm_joints[i].c_str())]] = q(i);
    mj_fwdPosition(m, d);
  }
  ~Model()
  {
    mj_deleteData(d);
    mj_deleteModel(m);
  }
};

mmp::LidarParams params(double sigma = 0.0)
{
  mmp::LidarParams l = profile().lidar;
  l.sigma_m = sigma;
  return l;
}

mmp::PlantScan one_scan(mmp::Lidar & lidar, const Model & model, double t0 = 0.0)
{
  for (int k = 1; k <= 50; ++k) lidar.advance(model.m, model.d, t0 + k * 0.002, 0.002);
  auto scan = lidar.take_scan();
  EXPECT_TRUE(scan.has_value());
  return scan ? *scan : mmp::PlantScan{};
}

// Exact ray against an oriented box (slab method in the box frame): the entry distance, or infinity.
double ray_box(const Eigen::Vector3d & o, const Eigen::Vector3d & dir, const Eigen::Vector3d & c, const Eigen::Matrix3d & R,
               const Eigen::Vector3d & half)
{
  const Eigen::Vector3d lo = R.transpose() * (o - c), ld = R.transpose() * dir;
  double t0 = -std::numeric_limits<double>::infinity(), t1 = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    if (std::abs(ld(i)) < 1e-15) {
      if (std::abs(lo(i)) > half(i)) return std::numeric_limits<double>::infinity();
      continue;
    }
    double a = (-half(i) - lo(i)) / ld(i), b = (half(i) - lo(i)) / ld(i);
    if (a > b) std::swap(a, b);
    t0 = std::max(t0, a);
    t1 = std::min(t1, b);
  }
  return t0 <= t1 && t0 > 0.0 ? t0 : std::numeric_limits<double>::infinity();
}
}  // namespace

TEST(Lidar, PatternMatchesUnityAsset)
{
  const auto & p = pattern();
  ASSERT_EQ(p.directions.size(), 800000u);
  // the asset's last vector (Unity frame), the CSV's row 800000 (-146.931, 40.061)
  const double last[3] = {-0.35118103, 0.7653597, -0.53934836};
  EXPECT_NEAR(p.directions.back().x(), last[2], 1e-6);
  EXPECT_NEAR(p.directions.back().y(), -last[0], 1e-6);
  EXPECT_NEAR(p.directions.back().z(), last[1], 1e-6);
  // Mid-360.asset's first vectors (Unity frame) mapped to ROS: (z, -x, y)
  const double unity[3][3] = {{0.6133357, 0.7897483, 0.010812953}, {0.65365124, 0.7567098, 0.0114096105}, {0.6892743, 0.7244005, 0.012031436}};
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(p.directions[i].x(), unity[i][2], 1e-6) << i;
    EXPECT_NEAR(p.directions[i].y(), -unity[i][0], 1e-6) << i;
    EXPECT_NEAR(p.directions[i].z(), unity[i][1], 1e-6) << i;
  }
  for (const auto & dir : p.directions) ASSERT_NEAR(dir.norm(), 1.0, 1e-12);
}

// G4: every ray of a scan against the exact nearest world hit (static boxes and the ground plane). A ray with a point:
// its range equals the exact one within 1e-6 m plus one float ulp of the stored point (MuJoCo's distance is double,
// PlantScan keeps float points), same geom; on the robot or panel, nothing in the world is nearer. A ray without a point:
// the exact world hit lies outside (0.1, 70) m, or the robot occludes it below the minimum range.
TEST(Lidar, ExactRangesWithoutNoise)
{
  const auto p = profile();
  const Model model(p, p.poses.at("carry"));
  mmp::Lidar lidar(params(), pattern(), mmp::ScanMode::Instant);
  lidar.attach(model.m, 1);
  const mmp::PlantScan scan = one_scan(lidar, model);
  const int site = mj_name2id(model.m, mjOBJ_SITE, "livox_frame");
  const Eigen::Vector3d origin = Eigen::Map<const Eigen::Vector3d>(model.d->site_xpos + 3 * site);
  const Eigen::Matrix3d R = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(model.d->site_xmat + 9 * site);
  const int ground = mj_name2id(model.m, mjOBJ_GEOM, "ground");
  auto exact = [&](const Eigen::Vector3d & dir, int & geom) {
    double best = dir.z() < 0.0 ? -origin.z() / dir.z() : std::numeric_limits<double>::infinity();
    geom = ground;
    for (int g = 0; g < model.m->ngeom; ++g) {
      if (model.m->geom_type[g] != mjGEOM_BOX || model.m->geom_bodyid[g] != 0) continue;
      const double t = ray_box(origin, dir, Eigen::Map<const Eigen::Vector3d>(model.d->geom_xpos + 3 * g),
                               Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(model.d->geom_xmat + 9 * g),
                               Eigen::Map<const Eigen::Vector3d>(model.m->geom_size + 3 * g));
      if (t < best) best = t, geom = g;
    }
    return best;
  };
  std::map<int, size_t> point_of;
  for (size_t i = 0; i < scan.points.size(); ++i) point_of[scan.ray[i]] = i;
  ASSERT_EQ(scan.rays, 20000);
  int world = 0, robot = 0, empty = 0;
  double worst = 0.0;
  for (int r = 0; r < scan.rays; ++r) {
    const Eigen::Vector3d dir = R * pattern().directions[r];  // scan 0: the window starts at the pattern's start
    int best_geom = -1;
    const double best = exact(dir, best_geom);
    const auto it = point_of.find(r);
    if (it == point_of.end()) {
      ++empty;
      const bool outside = !(best > p.lidar.min_range_m && best < p.lidar.max_range_m);
      int robot_geom = -1;
      mjtNum o[3] = {origin.x(), origin.y(), origin.z()}, v[3] = {dir.x(), dir.y(), dir.z()};
      const double d = mj_ray(model.m, model.d, o, v, nullptr, 1, -1, &robot_geom, nullptr);
      const bool occluded = robot_geom >= 0 && model.m->geom_bodyid[robot_geom] != 0 && d <= p.lidar.min_range_m;
      EXPECT_TRUE(outside || occluded) << "ray " << r << ": exact world hit at " << best << " m but no point";
      continue;
    }
    const size_t i = it->second;
    const double range = scan.points[i].cast<double>().norm();
    if (scan.cls[i] == mmp::HitClass::Robot || scan.cls[i] == mmp::HitClass::Panel) {
      ++robot;
      EXPECT_GE(best, range - 1e-6) << r;  // the robot is in front of whatever the world has on that ray
      continue;
    }
    ++world;
    worst = std::max(worst, std::abs(range - best));
    EXPECT_NEAR(range, best, 1e-6 + 2.0 * std::numeric_limits<float>::epsilon() * best) << r;
    EXPECT_EQ(scan.geom[i], best_geom) << r;
  }
  std::cout << world << " world points within " << worst << " m of the exact ranges, " << robot << " self-hits, " << empty
            << " rays without a point (all outside the range or occluded)\n";
  EXPECT_GT(world, 5000);  // a carry-pose scan: about 11800 points, a fifth of them on the robot and panel
}

TEST(Lidar, WindowAdvancesPerScan)
{
  const auto p = profile();
  const Model model(p, p.poses.at("home"));
  mmp::Lidar lidar(params(), pattern(), mmp::ScanMode::Instant);
  lidar.attach(model.m, 1);
  const auto first = one_scan(lidar, model, 0.0);
  const auto second = one_scan(lidar, model, 0.1);
  EXPECT_EQ(first.scan, 0u);
  EXPECT_EQ(second.scan, 1u);
  EXPECT_EQ(second.rays, 20000);
  // point k of the second scan lies along pattern direction 20000 + ray
  const size_t k = second.points.size() / 2;
  const Eigen::Vector3d along = second.points[k].cast<double>().normalized();
  EXPECT_NEAR(along.dot(pattern().directions[20000 + second.ray[k]]), 1.0, 1e-6);
}

TEST(Lidar, SweepCastsItsSliceEachStep)
{
  const auto p = profile();
  const Model model(p, p.poses.at("home"));
  mmp::Lidar lidar(params(), pattern(), mmp::ScanMode::Sweep);
  lidar.attach(model.m, 1);
  for (int k = 1; k <= 25; ++k) lidar.advance(model.m, model.d, k * 0.002, 0.002);
  EXPECT_FALSE(lidar.take_scan().has_value());
  for (int k = 26; k <= 50; ++k) lidar.advance(model.m, model.d, k * 0.002, 0.002);
  const auto scan = lidar.take_scan();
  ASSERT_TRUE(scan.has_value());
  EXPECT_EQ(scan->rays, 20000);
  EXPECT_NEAR(scan->t_end - scan->t_start, 0.1, 1e-12);
  for (size_t i = 0; i < scan->points.size(); ++i) {  // ray r was cast in slice r / 400, at its end
    ASSERT_NEAR(scan->dt[i], (scan->ray[i] / 400 + 1) * 0.002, 1e-7) << i;  // dt is a float
  }
}

TEST(Lidar, SweepDistortsWhenMoving)
{
  const auto p = profile();
  Model model(p, p.poses.at("home"));
  mmp::Lidar sweep(params(), pattern(), mmp::ScanMode::Sweep), instant(params(), pattern(), mmp::ScanMode::Instant);
  sweep.attach(model.m, 1);
  instant.attach(model.m, 1);
  const int x = model.m->jnt_qposadr[mj_name2id(model.m, mjOBJ_JOINT, "base_x")];
  for (int k = 1; k <= 50; ++k) {  // the base drives at 1 m/s during the scan
    model.d->qpos[x] += 0.002;
    mj_fwdPosition(model.m, model.d);
    sweep.advance(model.m, model.d, k * 0.002, 0.002);
    instant.advance(model.m, model.d, k * 0.002, 0.002);
  }
  const auto a = sweep.take_scan(), b = instant.take_scan();
  ASSERT_TRUE(a && b);
  // the first slice was cast 0.098 m behind the instant scan's pose: its wall points are that much farther
  std::map<int, double> instant_range;
  for (size_t i = 0; i < b->points.size(); ++i) instant_range[b->ray[i]] = b->points[i].norm();
  int compared = 0;
  for (size_t i = 0; i < a->points.size(); ++i) {
    if (a->ray[i] >= 400 || a->geom[i] != mj_name2id(model.m, mjOBJ_GEOM, "scenario/wall") || !instant_range.count(a->ray[i])) continue;
    EXPECT_GT(a->points[i].norm(), instant_range[a->ray[i]] + 0.05);
    ++compared;
  }
  EXPECT_GT(compared, 0);
}

TEST(Lidar, LabelsByClass)
{
  const auto p = profile();
  const Model model(p, p.poses.at("carry"));
  mmp::Lidar lidar(params(), pattern(), mmp::ScanMode::Instant);
  lidar.attach(model.m, 1);
  std::map<mmp::HitClass, int> counts;
  for (int s = 0; s < 5; ++s)
    for (auto c : one_scan(lidar, model, 0.1 * s).cls) ++counts[c];
  EXPECT_GT(counts[mmp::HitClass::Ground], 0);
  EXPECT_GT(counts[mmp::HitClass::Scenario], 0);
  EXPECT_GT(counts[mmp::HitClass::Panel] + counts[mmp::HitClass::Robot], 0);  // the carried panel shadows the view
  EXPECT_EQ(counts[mmp::HitClass::Other], 0);
}

TEST(Lidar, DropsBelowMinRange)
{
  const auto p = profile();
  const Model model(p, p.poses.at("carry"));
  mmp::Lidar lidar(params(0.02), pattern(), mmp::ScanMode::Instant);
  lidar.attach(model.m, 1);
  for (int s = 0; s < 3; ++s)
    for (const auto & pt : one_scan(lidar, model, 0.1 * s).points) {
      ASSERT_GT(pt.norm(), p.lidar.min_range_m - 1e-6);
      ASSERT_LT(pt.norm(), p.lidar.max_range_m + 1e-6);
    }
}

// A sensor inside a geom: MuJoCo's ray-box returns the exit face (PhysX, Unity's engine, ignores a collider that contains
// the ray's origin and sees the world beyond). The livox site sits 6.7 cm above the chassis, inside no robot geom, so
// this only matters for an obstacle placed over the sensor; the plant then reports the box's inner faces, never a zero
// range.
TEST(Lidar, RayFromInsideGeomSeesTheExitFace)
{
  const auto p = profile();
  mmp::ScenarioSetup s;
  s.name = "boxed_in";
  // a 0.3 m box around the sensor (livox at base_link + (0.24, 0, 0.177), base_link 0.21 above the footprint)
  s.boxes.push_back({"cage", Eigen::Vector3d(-5.0 + 0.24, -5.5, 0.387), Eigen::Vector3d(0.3, 0.3, 0.3), 0.0});
  const Model model(p, p.poses.at("home"), -5.0, -5.5, 0.0, s);
  mmp::Lidar lidar(params(), pattern(), mmp::ScanMode::Instant);
  lidar.attach(model.m, 1);
  const auto scan = one_scan(lidar, model);
  const int cage = mj_name2id(model.m, mjOBJ_GEOM, "scenario/cage");
  int on_cage = 0;
  for (size_t i = 0; i < scan.points.size(); ++i) {
    ASSERT_GT(scan.points[i].norm(), p.lidar.min_range_m);
    on_cage += scan.geom[i] == cage;
    if (scan.geom[i] == cage) ASSERT_LE(scan.points[i].norm(), 0.15 * std::sqrt(3.0) + 1e-6);  // within the cage
  }
  EXPECT_GT(on_cage, 0);
  EXPECT_EQ(scan.cls[0], mmp::HitClass::Scenario);
}

TEST(Lidar, NoiseIsUnitysPerRayDraw)
{
  const auto p = profile();
  const Model model(p, p.poses.at("carry"));
  mmp::Lidar clean(params(0.0), pattern(), mmp::ScanMode::Instant), noisy(params(0.02), pattern(), mmp::ScanMode::Instant);
  clean.attach(model.m, 77);
  noisy.attach(model.m, 77);
  const auto a = one_scan(clean, model), b = one_scan(noisy, model);
  std::map<int, double> range;
  for (size_t i = 0; i < a.points.size(); ++i) range[a.ray[i]] = a.points[i].cast<double>().norm();
  int compared = 0;
  for (size_t i = 0; i < b.points.size(); ++i) {
    if (!range.count(b.ray[i])) continue;
    const double residual = b.points[i].cast<double>().norm() - range[b.ray[i]];
    ASSERT_NEAR(residual, mmp::lidar_range_noise(77u, 0u, b.ray[i], 0.02f), 2e-5) << i;  // float points
    ++compared;
  }
  EXPECT_GT(compared, 10000);
}

TEST(Lidar, WindowWrapsAroundThePattern)
{
  const auto p = profile();
  const Model model(p, p.poses.at("home"));
  mmp::Lidar lidar(params(), pattern(), mmp::ScanMode::Instant);
  lidar.attach(model.m, 1);
  mmp::PlantScan scan;
  for (int s = 0; s <= 40; ++s) scan = one_scan(lidar, model, 0.1 * s);  // 40 scans of 20000 cover the 800000 directions
  EXPECT_EQ(scan.scan, 40u);
  const size_t k = scan.points.size() / 3;
  EXPECT_NEAR(scan.points[k].cast<double>().normalized().dot(pattern().directions[scan.ray[k]]), 1.0, 1e-6);  // from 0 again
}

TEST(Lidar, NoScanIsLostBetweenReads)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Ideal);
  plant.enable_lidar(PATTERN_BIN);
  plant.reset(fixture(), {-5.0, -5.5, 0.0}, p.poses.at("carry"), 3);
  mmp::PlantCommand c;
  c.arm.joint_velocity = mmp::Vec6::Zero();
  plant.step(c, 0.5);  // five scans complete within one step
  std::vector<uint32_t> seen;
  while (auto s = plant.scan()) seen.push_back(s->scan);
  EXPECT_EQ(seen, (std::vector<uint32_t>{0, 1, 2, 3, 4}));
}

TEST(Lidar, EnabledMidRunStartsAFullScanThen)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Ideal);
  mmp::PlantCommand c;
  c.arm.joint_velocity = mmp::Vec6::Zero();
  plant.reset(fixture(), {-5.0, -5.5, 0.0}, p.poses.at("carry"), 3);
  plant.step(c, 1.0);
  plant.enable_lidar(PATTERN_BIN);
  plant.step(c, 0.1);
  const auto s = plant.scan();
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->t_start, 1.0, 1e-9);
  EXPECT_NEAR(s->t_end, 1.1, 1e-9);
  for (float dt : s->dt) ASSERT_LE(dt, 0.1f + 1e-6f);
}

TEST(Lidar, DropoutRemovesItsShare)
{
  const auto p = profile();
  const Model model(p, p.poses.at("carry"));
  auto with = params();
  mmp::Lidar full(with, pattern(), mmp::ScanMode::Instant);
  with.dropout = 0.25;
  mmp::Lidar dropped(with, pattern(), mmp::ScanMode::Instant);
  full.attach(model.m, 1);
  dropped.attach(model.m, 1);
  const double kept = one_scan(dropped, model).points.size() / static_cast<double>(one_scan(full, model).points.size());
  EXPECT_NEAR(kept, 0.75, 0.02);
}

TEST(Lidar, PlantScansAtTenHertzOnlyWhenEnabled)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Ideal);
  mmp::PlantCommand c;
  c.arm.joint_velocity = mmp::Vec6::Zero();
  plant.reset(fixture(), {-5.0, -5.5, 0.0}, p.poses.at("carry"), 3);
  plant.step(c, 0.2);
  EXPECT_FALSE(plant.scan().has_value());  // off by default
  plant.enable_lidar(PATTERN_BIN);
  plant.reset(fixture(), {-5.0, -5.5, 0.0}, p.poses.at("carry"), 3);
  int scans = 0;
  for (int k = 0; k < 50; ++k) {  // 1 s
    plant.step(c, 0.02);
    if (auto s = plant.scan()) {
      ++scans;
      EXPECT_EQ(s->rays, 20000);
    }
  }
  EXPECT_EQ(scans, 10);
}
