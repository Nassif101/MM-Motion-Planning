#include "mobile_manipulator_campaign/lidar_fixture.hpp"

#include <Eigen/Geometry>
#include <stdexcept>

#include "mobile_manipulator_plant/frames.hpp"
#include "mobile_manipulator_plant/run_log.hpp"

namespace mobile_manipulator_campaign
{
namespace mmp = mobile_manipulator_plant;
using json = nlohmann::json;

namespace
{
constexpr double kDt = 0.02;      // the B5 cycle the hold steps with
constexpr double kSettleS = 0.5;  // on its wheels after the reset, before the scans that count

bool starts_with(const char * name, const char * prefix) { return name && std::string(name).rfind(prefix, 0) == 0; }

Eigen::Isometry3d pose_of(const mjtNum * pos, const mjtNum * mat)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 3; ++r) {
    t.translation()(r) = pos[r];
    for (int c = 0; c < 3; ++c) t.linear()(r, c) = mat[3 * r + c];
  }
  return t;
}

json rows(const Eigen::MatrixXd & m)
{
  json out = json::array();
  for (int r = 0; r < m.rows(); ++r) {
    json row = json::array();
    for (int c = 0; c < m.cols(); ++c) row.push_back(m(r, c));
    out.push_back(row);
  }
  return out;
}

json primitives(const mjModel * m, const mjData * d)
{
  json out = json::array();
  for (int g = 0; g < m->ngeom; ++g) {
    const char * name = mj_id2name(m, mjOBJ_GEOM, g);
    std::string label;
    if (name && std::string(name) == "robot/panel") label = "panel";
    else if (starts_with(name, "robot/")) label = "robot";
    else if (starts_with(name, "site/")) label = "site";
    else if (starts_with(name, "scenario/")) label = "scenario";
    else continue;
    std::string kind;
    switch (m->geom_type[g]) {
      case mjGEOM_BOX: kind = "box"; break;
      case mjGEOM_CYLINDER: kind = "cylinder"; break;
      case mjGEOM_SPHERE: kind = "sphere"; break;
      default: throw std::runtime_error(std::string("lidar_fixture: no distance for the shape of geom ") + name);
    }
    const Eigen::Isometry3d t = pose_of(d->geom_xpos + 3 * g, d->geom_xmat + 9 * g);
    out.push_back({{"name", name}, {"kind", kind}, {"label", label},
                   {"pos", {t.translation().x(), t.translation().y(), t.translation().z()}},
                   {"rot", rows(t.linear())},
                   {"size", {m->geom_size[3 * g], m->geom_size[3 * g + 1], m->geom_size[3 * g + 2]}}});
  }
  return out;
}
}  // namespace

LidarFixtureResult lidar_fixture(SessionConfig config, const std::string & scenario, const mmp::Pose2 & pose, int scans,
                                 uint64_t seed)
{
  if (scans < 1) throw std::invalid_argument("lidar_fixture: scans must be positive");
  config.lidar = true;
  config.scan_decimation = 1;
  config.perturb_start = false;
  config.medium = false;
  config.log_path.clear();
  Session session(config);
  session.reset(scenario, "", pose, seed);
  mmp::Plant & plant = session.plant().plant();
  const mjModel * m = plant.model();
  const int base = mj_name2id(m, mjOBJ_BODY, "base_footprint");
  const int lidar = mj_name2id(m, mjOBJ_SITE, plant.profile().lidar.site.c_str());
  if (base < 0 || lidar < 0) throw std::runtime_error("lidar_fixture: no base_footprint body or lidar site");

  mmp::PlantCommand hold;
  hold.arm.joint_velocity = mmp::Vec6::Zero();
  auto step = [&] {
    hold.stamp = plant.true_state().t;
    plant.step(hold, kDt);
  };
  for (double t = 0.0; t < kSettleS - 1e-9; t += kDt) step();
  while (plant.scan()) {
  }
  const Eigen::Vector3d start(plant.data()->xpos + 3 * base);

  LidarFixtureResult r;
  const double limit_s = (scans + 2) / plant.profile().lidar.rate_hz;
  for (double t = 0.0; static_cast<int>(r.scans.size()) < scans && t < limit_s; t += kDt) {
    step();
    while (auto scan = plant.scan())
      if (static_cast<int>(r.scans.size()) < scans) r.scans.push_back(std::move(*scan));
  }
  if (static_cast<int>(r.scans.size()) < scans) throw std::runtime_error("lidar_fixture: too few scans in the hold");

  const mjData * d = plant.data();
  const Eigen::Isometry3d base_pose = pose_of(d->xpos + 3 * base, d->xmat + 9 * base);
  const Eigen::Isometry3d lidar_pose = pose_of(d->site_xpos + 3 * lidar, d->site_xmat + 9 * lidar);
  const auto & q = plant.true_state().q;
  r.header = {{"schema", mmp::kSchemaVersion},
              {"kind", "lidar_fixture"},
              {"scenario", scenario},
              {"pose", {pose.x, pose.y, pose.yaw}},
              {"seed", seed},
              {"mode", config.mode == mmp::PlantMode::Realistic ? "realistic" : "ideal"},
              {"profile", config.profile},
              {"scans", scans},
              {"base", rows(base_pose.matrix())},
              {"lidar_in_base", rows((base_pose.inverse() * lidar_pose).matrix())},
              {"q", {q(0), q(1), q(2), q(3), q(4), q(5)}},
              {"contacts", static_cast<int>(plant.ground_truth().contacts.size())},
              {"drift_m", (base_pose.translation() - start).head<2>().norm()},
              {"prims", primitives(m, d)}};
  return r;
}

void write_lidar_fixture(const LidarFixtureResult & result, const std::string & path)
{
  mmp::RunLogWriter log(path, result.header);
  for (const auto & scan : result.scans) {
    std::vector<float> points, classes;
    for (size_t i = 0; i < scan.points.size(); ++i) {
      points.insert(points.end(), {scan.points[i].x(), scan.points[i].y(), scan.points[i].z()});
      classes.push_back(static_cast<float>(scan.cls[i]));
    }
    log.write(mmp::encode(mmp::FrameType::Scan,
                          {{"scan", scan.scan}, {"t_start", scan.t_start}, {"t_end", scan.t_end}, {"rays", scan.rays},
                           {"layout", "points xyz in the lidar frame at each ray's time; classes"}},
                          {points, classes}));
  }
  log.close();
}
}  // namespace mobile_manipulator_campaign
