#include "mobile_manipulator_routing/primitives.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

namespace mobile_manipulator_routing
{
namespace
{
using Json = nlohmann::json;
constexpr std::uintmax_t kMaxFileBytes = 8u * 1024u * 1024u;
constexpr std::size_t kMaxHeadings = 256, kMaxMotions = 65536, kMaxPoses = 1000000;
constexpr double kHeadingTolerance = 1e-6, kEndpointToleranceM = 1e-4;
constexpr double kOriginToleranceM = 1e-9;

[[noreturn]] void invalid(const std::string & reason)
{
  throw std::invalid_argument("lattice primitive data: " + reason);
}

double number(const Json & value, const char * field)
{
  if (!value.is_number()) invalid(std::string(field) + " must be numeric");
  const double result = value.get<double>();
  if (!std::isfinite(result)) invalid(std::string(field) + " must be finite");
  return result;
}

std::size_t integer(const Json & value, const char * field)
{
  if (!value.is_number_integer()) invalid(std::string(field) + " must be an integer");
  if (value.is_number_unsigned()) {
    const auto result = value.get<std::uint64_t>();
    if (result > std::numeric_limits<std::size_t>::max()) invalid(std::string(field) + " is too large");
    return static_cast<std::size_t>(result);
  }
  const auto result = value.get<std::int64_t>();
  if (result < 0 || static_cast<std::uint64_t>(result) > std::numeric_limits<std::size_t>::max())
    invalid(std::string(field) + " must be nonnegative and in range");
  return static_cast<std::size_t>(result);
}

int endpoint_cell(double value, double resolution)
{
  const double cells = value / resolution;
  const double rounded = std::round(cells);
  if (!std::isfinite(cells) || rounded < std::numeric_limits<int>::min() ||
      rounded > std::numeric_limits<int>::max() ||
      std::abs(value - rounded * resolution) > kEndpointToleranceM)
    invalid("trajectory endpoint must lie on the declared metric lattice");
  return static_cast<int>(rounded);
}

double length_tolerance(double length, std::size_t samples)
{
  // Arc lengths exceed sampled chords; the source positions are rounded to five decimal places.
  return std::max(1e-4, 0.02 * length + 2e-5 * static_cast<double>(samples));
}

PrimitiveSet parse(const Json & data, double required_resolution)
{
  if (!data.is_object()) invalid("root must be an object");
  const auto & version = data.at("version");
  const bool supported_version = (version.is_string() && version.get<std::string>() == "1.0") ||
    (version.is_number() && number(version, "version") == 1.0);
  if (!supported_version) invalid("only format version 1.0 is supported");

  const auto & metadata = data.at("lattice_metadata");
  if (!metadata.is_object() || metadata.at("motion_model") != "diff")
    invalid("motion_model must be diff");
  PrimitiveSet result;
  result.resolution = number(metadata.at("grid_resolution"), "grid_resolution");
  result.turning_radius = number(metadata.at("turning_radius"), "turning_radius");
  if (!(result.resolution > 0.0) || !(result.turning_radius > 0.0) ||
      std::abs(result.resolution - required_resolution) > 1e-9)
    invalid("positive resolution/turning radius and matching map resolution are required");

  const std::size_t count = integer(metadata.at("num_of_headings"), "num_of_headings");
  const auto & angles = metadata.at("heading_angles");
  if (count == 0 || count > kMaxHeadings || !angles.is_array() || angles.size() != count)
    invalid("heading count/array mismatch or limit exceeded");
  result.headings.reserve(count);
  for (const auto & value : angles) {
    const double angle = number(value, "heading angle");
    if (angle < 0.0 || angle >= 2.0 * kPi) invalid("heading angles must be in [0, 2*pi)");
    for (double previous : result.headings)
      if (std::abs(wrap_angle(angle - previous)) < 1e-10) invalid("duplicate heading angle");
    result.headings.push_back(angle);
  }

  const auto & trajectories = data.at("primitives");
  const auto trajectory_count = integer(metadata.at("number_of_trajectories"), "number_of_trajectories");
  if (!trajectories.is_array() || trajectories.empty() || trajectories.size() > kMaxMotions ||
      trajectories.size() != trajectory_count)
    invalid("trajectory count/array mismatch or limit exceeded");

  // Check the total before allocating the parsed pose arrays. Each primitive gets an explicit start.
  std::size_t total_poses = trajectories.size();
  for (const auto & trajectory : trajectories) {
    const auto & poses = trajectory.at("poses");
    if (!poses.is_array() || poses.empty() || poses.size() > kMaxPoses - total_poses)
      invalid("pose array is empty or total pose limit exceeded");
    total_poses += poses.size();
  }

  result.motions.reserve(trajectories.size());
  result.successors.resize(count);
  std::set<std::size_t> ids;
  for (const auto & trajectory : trajectories) {
    MotionPrimitive motion;
    motion.id = integer(trajectory.at("trajectory_id"), "trajectory_id");
    motion.start_heading = integer(trajectory.at("start_angle_index"), "start_angle_index");
    motion.end_heading = integer(trajectory.at("end_angle_index"), "end_angle_index");
    if (!ids.insert(motion.id).second) invalid("duplicate trajectory_id");
    if (motion.start_heading >= count || motion.end_heading >= count) invalid("heading index out of range");

    const double declared_length = number(trajectory.at("trajectory_length"), "trajectory_length");
    if (declared_length < 0.0) invalid("negative trajectory length");
    for (const char * field : {"arc_length", "straight_length", "trajectory_radius"}) {
      if (trajectory.contains(field) && number(trajectory.at(field), field) < 0.0)
        invalid(std::string(field) + " must be nonnegative");
    }
    if (trajectory.contains("left_turn") && !trajectory.at("left_turn").is_boolean())
      invalid("left_turn must be boolean");
    if (trajectory.contains("arc_length") && trajectory.contains("straight_length")) {
      const double parts = number(trajectory.at("arc_length"), "arc_length") +
        number(trajectory.at("straight_length"), "straight_length");
      if (!std::isfinite(parts) || std::abs(parts - declared_length) >
          length_tolerance(declared_length, trajectory.at("poses").size()))
        invalid("trajectory length disagrees with arc and straight lengths");
    }

    const auto & source_poses = trajectory.at("poses");
    motion.poses.reserve(source_poses.size() + 1);
    motion.poses.push_back({0.0, 0.0, result.headings[motion.start_heading]});
    double measured_length = 0.0;
    bool positions_at_origin = true;
    for (const auto & value : source_poses) {
      if (!value.is_array() || value.size() != 3) invalid("each pose must contain x, y, yaw");
      Pose2 pose{number(value[0], "pose x"), number(value[1], "pose y"), number(value[2], "pose yaw")};
      const Pose2 & previous = motion.poses.back();
      const double turn = wrap_angle(pose.yaw - previous.yaw);
      pose.yaw = previous.yaw + turn;  // preserve continuous angular travel through the wrap
      const double dx = pose.x - previous.x, dy = pose.y - previous.y;
      const double segment = std::hypot(dx, dy);
      const double middle_yaw = previous.yaw + 0.5 * turn;
      if (!std::isfinite(segment) || !std::isfinite(pose.yaw)) invalid("pose differences overflow");
      if (dx * std::cos(middle_yaw) + dy * std::sin(middle_yaw) < -1e-5)
        invalid("reverse translation is not supported");
      measured_length += segment;
      motion.angular_travel += std::abs(turn);
      if (!std::isfinite(measured_length) || !std::isfinite(motion.angular_travel))
        invalid("trajectory extent overflows");
      positions_at_origin = positions_at_origin && std::hypot(pose.x, pose.y) <= kOriginToleranceM;
      // An already explicit exact start need not appear twice. Never shift positions by the first sample.
      if (segment > kOriginToleranceM || std::abs(turn) > 1e-12 || motion.poses.size() > 1)
        motion.poses.push_back(pose);
    }
    if (motion.poses.size() < 2) invalid("trajectory has no motion");
    const Pose2 & end = motion.poses.back();
    if (std::abs(wrap_angle(end.yaw - result.headings[motion.end_heading])) > kHeadingTolerance)
      invalid("final pose heading disagrees with end_angle_index");
    motion.dx = endpoint_cell(end.x, result.resolution);
    motion.dy = endpoint_cell(end.y, result.resolution);
    if (std::abs(measured_length - declared_length) > length_tolerance(declared_length, source_poses.size()))
      invalid("trajectory length disagrees with sampled motion");
    motion.length_m = std::max(measured_length, declared_length);
    motion.pure_rotation = positions_at_origin;
    motion.turning = motion.angular_travel > 1e-10;
    if (motion.pure_rotation && !motion.turning) invalid("stationary zero-cost motion is not supported");
    result.motions.push_back(std::move(motion));
  }

  // Stable IDs, not JSON order, define successor traversal and its deterministic tie behavior.
  std::sort(result.motions.begin(), result.motions.end(), [](const auto & a, const auto & b) { return a.id < b.id; });
  for (std::size_t i = 0; i < result.motions.size(); ++i)
    result.successors[result.motions[i].start_heading].push_back(i);
  for (const auto & successors : result.successors)
    if (successors.empty()) invalid("every declared heading must have at least one successor");
  return result;
}
}  // namespace

PrimitiveSet PrimitiveSet::load(const std::filesystem::path & path, double required_resolution)
{
  if (!std::isfinite(required_resolution) || !(required_resolution > 0.0))
    invalid("required map resolution must be positive and finite");
  std::error_code error;
  const auto size = std::filesystem::file_size(path, error);
  if (error) throw std::runtime_error("cannot stat lattice primitive file");
  if (size == 0 || size > kMaxFileBytes) invalid("file is empty or exceeds 8 MiB");
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot open lattice primitive file");
  std::string bytes(static_cast<std::size_t>(size), '\0');
  if (!stream.read(bytes.data(), static_cast<std::streamsize>(size)) || stream.peek() != std::char_traits<char>::eof())
    throw std::runtime_error("lattice primitive file changed or could not be read completely");
  try {
    return parse(Json::parse(bytes), required_resolution);
  } catch (const Json::exception & error) {
    invalid(std::string("malformed JSON/schema: ") + error.what());
  }
}

std::size_t PrimitiveSet::nearest_heading(double yaw) const
{
  if (headings.empty() || !std::isfinite(yaw)) throw std::invalid_argument("nearest_heading needs a finite yaw and headings");
  std::size_t best = 0;
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < headings.size(); ++i) {
    if (!std::isfinite(headings[i])) throw std::invalid_argument("nonfinite heading");
    const double difference = std::abs(wrap_angle(yaw - headings[i]));
    if (difference < nearest) { nearest = difference; best = i; }
  }
  return best;
}

std::size_t PrimitiveSet::storage_bytes() const
{
  std::size_t bytes = sizeof(*this) + headings.capacity() * sizeof(double) +
    motions.capacity() * sizeof(MotionPrimitive) + successors.capacity() * sizeof(std::vector<std::size_t>);
  for (const auto & motion : motions) bytes += motion.poses.capacity() * sizeof(Pose2);
  for (const auto & next : successors) bytes += next.capacity() * sizeof(std::size_t);
  return bytes;
}
}  // namespace mobile_manipulator_routing
