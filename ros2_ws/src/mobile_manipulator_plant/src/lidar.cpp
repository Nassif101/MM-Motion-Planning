#include "mobile_manipulator_plant/lidar.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

#include "mobile_manipulator_plant/lidar_noise.hpp"

namespace mobile_manipulator_plant
{
namespace
{
bool starts_with(const char * name, const char * prefix) { return name && std::string(name).rfind(prefix, 0) == 0; }

HitClass class_of(const char * name)
{
  if (!name) return HitClass::Other;
  const std::string n(name);
  if (n == "ground") return HitClass::Ground;
  if (n == "robot/panel") return HitClass::Panel;
  if (starts_with(name, "robot/")) return HitClass::Robot;
  if (starts_with(name, "site/")) return HitClass::Site;
  if (starts_with(name, "scenario/")) return HitClass::Scenario;
  if (starts_with(name, "mover/")) return HitClass::Mover;
  return HitClass::Other;
}
}  // namespace

ScanPattern load_pattern(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  char magic[8];
  uint32_t count = 0;
  if (!in.read(magic, 8) || std::memcmp(magic, "MMPSCAN1", 8) != 0 || !in.read(reinterpret_cast<char *>(&count), sizeof count))
    throw std::runtime_error("load_pattern: " + path + " is not a scan pattern");
  std::vector<float> data(2 * static_cast<size_t>(count));
  if (!in.read(reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(data.size() * sizeof(float))))
    throw std::runtime_error("load_pattern: " + path + " is truncated");
  ScanPattern p;
  p.directions.reserve(count);
  constexpr double deg = M_PI / 180.0;
  for (uint32_t i = 0; i < count; ++i) {
    const double a = data[2 * i] * deg, z = data[2 * i + 1] * deg;
    p.directions.emplace_back(std::sin(z) * std::cos(a), -std::sin(z) * std::sin(a), std::cos(z));
  }
  if (p.directions.empty()) throw std::runtime_error("load_pattern: " + path + " is empty");
  return p;
}

Lidar::Lidar(const LidarParams & params, ScanPattern pattern, ScanMode mode)
: params_(params), pattern_(std::move(pattern)), mode_(mode)
{
  if (params_.points_per_scan <= 0 || params_.rate_hz <= 0.0) throw std::invalid_argument("Lidar: points and rate must be positive");
}

void Lidar::attach(const mjModel * m, const mjData * d, uint64_t seed, double t)
{
  bvh_ = params_.static_bvh ? std::make_unique<StaticBvh>(m, d) : nullptr;
  site_ = mj_name2id(m, mjOBJ_SITE, params_.site.c_str());
  if (site_ < 0) throw std::runtime_error("Lidar: the model has no site " + params_.site);
  classes_.assign(m->ngeom, HitClass::Other);
  for (int g = 0; g < m->ngeom; ++g) classes_[g] = class_of(mj_id2name(m, mjOBJ_GEOM, g));
  seed32_ = static_cast<uint32_t>(seed ^ (seed >> 32));
  scan_ = 0;
  window_ = 0;
  cast_ = 0;
  t_start_ = t;
  current_ = PlantScan{};
  done_.clear();
}

void Lidar::advance(const mjModel * m, mjData * d, double t, double /*dt*/)
{
  const double period = 1.0 / params_.rate_hz;
  if (mode_ == ScanMode::Instant) {
    if (t - t_start_ >= period - 1e-9) {  // the whole scan from the pose now
      cast(m, d, 0, params_.points_per_scan, t);
      finish(t);
    }
    return;
  }
  // sweep: the rays due by t, at points_per_scan per period, from the pose now
  const double fraction = std::min(1.0, (t - t_start_) / period);
  const int due = std::min(params_.points_per_scan, static_cast<int>(std::lround(fraction * params_.points_per_scan)));
  if (due > cast_) cast(m, d, cast_, due - cast_, t);
  if (cast_ >= params_.points_per_scan) finish(t);
}

std::optional<PlantScan> Lidar::take_scan()
{
  if (done_.empty()) return std::nullopt;
  PlantScan out = std::move(done_.front());
  done_.pop_front();
  return out;
}

void Lidar::cast(const mjModel * m, mjData * d, int first, int count, double t)
{
  const Eigen::Map<const Eigen::Vector3d> origin(d->site_xpos + 3 * site_);
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(d->site_xmat + 9 * site_);
  vec_.resize(3 * static_cast<size_t>(count));
  dist_.resize(count);
  geomid_.resize(count);
  normal_.resize(3 * static_cast<size_t>(count));
  const long size = static_cast<long>(pattern_.directions.size());
  for (int k = 0; k < count; ++k) {
    const Eigen::Vector3d dir = R * pattern_.directions[(window_ + first + k) % size];
    for (int i = 0; i < 3; ++i) vec_[3 * k + i] = dir(i);
  }
  // no body excluded: the robot sees itself. With the BVH: MuJoCo's rays for the geoms not welded to the world (robot,
  // panel, movers), the tree for the static ones, the nearer hit wins; without it, MuJoCo's rays for everything.
  mj_multiRay(m, d, origin.data(), vec_.data(), nullptr, bvh_ ? 0 : 1, -1, geomid_.data(), dist_.data(), normal_.data(),
              count, params_.max_range_m);
  if (bvh_) {
    for (int k = 0; k < count; ++k) {
      int geom = -1;
      const double t = bvh_->ray(origin, Eigen::Map<const Eigen::Vector3d>(vec_.data() + 3 * k), geom);
      if (geom >= 0 && (geomid_[k] < 0 || t < dist_[k])) {
        dist_[k] = t;
        geomid_[k] = geom;
      }
    }
  }
  const float min_r = static_cast<float>(params_.min_range_m), max_r = static_cast<float>(params_.max_range_m);
  const float sigma = static_cast<float>(params_.sigma_m);
  for (int k = 0; k < count; ++k) {
    ++current_.rays;
    if (geomid_[k] < 0 || dist_[k] < 0.0) continue;  // no hit within the range
    const int ray = first + k;
    const float distance = static_cast<float>(dist_[k]);
    const float noised = sigma > 0.0f ? distance + lidar_range_noise(seed32_, scan_, ray, sigma) : distance;
    // UnitySensors' IRaycastHitsToPointsJob: the range before and after the noise inside (min, max)
    if (!(min_r < distance && distance < max_r && min_r < noised && noised < max_r)) continue;
    if (params_.dropout > 0.0 && lidar_uniform(seed32_, scan_, ray) < params_.dropout) continue;
    const Eigen::Vector3d local = pattern_.directions[(window_ + ray) % size];
    current_.points.push_back((local * static_cast<double>(noised)).cast<float>());
    current_.dt.push_back(static_cast<float>(t - t_start_));
    current_.ray.push_back(ray);
    current_.geom.push_back(geomid_[k]);
    current_.cls.push_back(classes_[geomid_[k]]);
  }
  cast_ = first + count;
}

void Lidar::finish(double t)
{
  current_.scan = scan_;
  current_.t_start = t_start_;
  current_.t_end = t;
  done_.push_back(std::move(current_));
  current_ = PlantScan{};
  ++scan_;
  window_ = (window_ + params_.points_per_scan) % static_cast<long>(pattern_.directions.size());
  cast_ = 0;
  t_start_ = t;
}
}  // namespace mobile_manipulator_plant
