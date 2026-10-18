// Offline search for a bent-arm carry pose (a candidate only; nothing is registered here).
//
// The panel stays level and keeps home's yaw: with shoulder_pan = wrist_3 = 0 the y-axis joints
// shoulder_lift, elbow and wrist_2 form a planar chain, and wrist_1 (about the forearm's z axis)
// can only keep the panel level and at home's yaw at 0 (any other wrist_1 tilts or turns it; the
// grid samples it to show that). wrist_2 is therefore not free: for each (shoulder_lift, elbow,
// wrist_1) sample it is solved so the panel normal is vertical, and the sample is kept only when
// the panel's roll, pitch and yaw are all within --level-tol of home's.
//
// Constraints per sample: joints inside the URDF limits intersected with the operating
// envelope's wholebody_arm.joint_bounds with a --joint-margin; the panel at least --clearance
// from every base and arm primitive (wrist_3, the flange the panel is mounted on, excluded) and
// its bottom above the floor rule; the arm primitives at least --clearance from the base and
// from their non-adjacent arm links; the ground-projected hull (every primitive and the panel,
// plus the profiles' perimeter allowance) no wider or longer than home's.
//
// --mirror-of NAME (Phase 4B-A spec Section 3) instead solves the mirror image of a named pose across the
// base's vertical x-z plane: the joint vector whose panel ("panel" frame of the whole-body model) has the
// named pose's centre with y negated and its axes mirrored (compared up to sign: the payload box is
// symmetric), by damped least squares from the seeds [-q1, q2, q3, q4, +-q5, +-q6], within the joint limits
// less the joint margin; it prints the residuals and the usual metrics of the named pose and the solution
// (the footprint check is not applied: a mirrored pose has the mirrored footprint).
//
// Usage: ros2 run mobile_manipulator_mpc carry_pose_search [--step R] [--top-min M] [--top-max M]
//        [--min-elbow R] [--csv FILE] [--eval q1,q2,q3,q4,q5,q6 ...] [--mirror-of NAME]
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/narrowphase/distance.h>
#include <nlohmann/json.hpp>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/polygon_ops.hpp"
#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/model.hpp"

namespace
{
namespace mmg = mobile_manipulator_geometry;
namespace mmm = mobile_manipulator_mpc;
using Joints = Eigen::Matrix<double, 6, 1>;
using Eigen::Vector3d;

constexpr double kGravity = 9.81;
constexpr double kPerimeterAllowance = 0.02;  // footprint_profiles.yaml perimeter_allowance_m

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

const std::set<std::string> kBaseLinks = {"base_link", "front_left_wheel_link", "front_right_wheel_link",
                                          "rear_left_wheel_link", "rear_right_wheel_link", "arm_mount_link",
                                          "shoulder_pan_link"};
// Arm chain after shoulder_pan, shoulder to flange.
const std::vector<std::string> kArmLinks = {"upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link",
                                            "wrist_3_link"};

struct Metrics
{
  Joints q;
  bool feasible = false;
  std::string reject;
  double tilt = 0, yaw = 0;                      // panel tilt from level and yaw (rad)
  double panel_x = 0, panel_z = 0, top = 0, bottom = 0;
  double joint_margin = 0;
  double panel_clearance = 0;                    // panel to the closest arm primitive (wrist_3 excluded)
  std::string panel_closest;
  double panel_base_clearance = 0;               // panel to the base, arm mount and shoulder_pan
  double self_clearance = 0;                     // arm primitives to the base primitives
  std::string self_closest;
  double arm_clearance = 0;                      // arm primitives to their non-adjacent arm links
  std::string arm_closest;
  double fp_xmin = 0, fp_xmax = 0, fp_ymin = 0, fp_ymax = 0, fp_area = 0;
  double manip = 0;
  double lever_shoulder = 0, lever_elbow = 0;    // signed horizontal distance of the panel COM (m)
  double tau_shoulder = 0, tau_elbow = 0, tau_wrist2 = 0;  // full static gravity torque (N m), arm + panel
  double dist_vc = 0, time_vc = 0, time_home = 0;  // joint distance and synchronized time
};

struct Solid
{
  std::string name;
  Eigen::Isometry3d pose;
  std::shared_ptr<fcl::CollisionGeometryd> geometry;
};

class Evaluator
{
public:
  Evaluator()
  {
    payload_json_ = nlohmann::ordered_json::parse(read_file(share("mobile_manipulator_control") +
                                                            "/config/qualified_payload.json"));
    urdf_ = read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf");
    payload_ = mmg::payload_from_json(payload_json_);
    projector_ = std::make_unique<mmg::FootprintProjector>(urdf_, payload_);
    limits_ = mmm::load_limits(share("mobile_manipulator_navigation") + "/config/nav_operating_envelope.yaml", urdf_);
    mass_ = payload_json_.at("payload").at("mass_kg").get<double>();
    floor_ = payload_json_.at("acceptance").at("min_panel_ground_clearance_m").get<double>();
    const auto & poses = payload_json_.at("poses_rad");
    for (const auto & name : {"home", "vertical_carry", "level_extension"}) {
      Joints q;
      for (int i = 0; i < 6; ++i) q(i) = poses.at(name).at(i).get<double>();
      named_[name] = q;
    }
    const auto & prims = projector_->primitives();
    for (size_t i = 0; i < prims.size(); ++i) {
      const auto & p = prims[i];
      Entry e;
      e.link = p.link;
      e.panel = i + 1 == prims.size();  // load_primitives appends the payload last
      e.local = p.pose;
      if (p.shape == mmg::Shape::Box) e.geometry = std::make_shared<fcl::Boxd>(2.0 * p.dims);
      else e.geometry = std::make_shared<fcl::Cylinderd>(p.dims(0), 2.0 * p.dims(1));
      entries_.push_back(e);
    }
    home_ = evaluate(named_.at("home"));
  }

  const Joints & named(const std::string & name) const { return named_.at(name); }
  const mmm::Limits & limits() const { return limits_; }
  const Metrics & home() const { return home_; }

  mmg::JointMap joint_map(const Joints & q) const
  {
    mmg::JointMap m;
    for (int i = 0; i < 6; ++i) m[mmm::kArmJoints[i]] = q(i);
    return m;
  }

  // wrist_2 that makes the panel normal vertical for the other joints (wrist_2 axis is y).
  double level_wrist_2(Joints q) const
  {
    q(4) = 0.0;
    const Eigen::Matrix3d r = projector_->link_pose("wrist_2_link", joint_map(q)).linear();
    const Vector3d up = r.transpose() * Vector3d::UnitZ();  // world up in the wrist_2 frame
    return std::atan2(up.x(), up.z());                        // Ry(w2) * z = (sin w2, 0, cos w2)
  }

  // Synchronized point-to-point time with trapezoidal profiles at the envelope's arm limits.
  double move_time(const Joints & a, const Joints & b) const
  {
    double t = 0.0;
    for (int i = 0; i < 6; ++i) {
      const double d = std::abs(a(i) - b(i)), v = limits_.qd_max(i), acc = limits_.qdd_max(i);
      t = std::max(t, d >= v * v / acc ? d / v + v / acc : 2.0 * std::sqrt(d / acc));
    }
    return t;
  }

  Metrics evaluate(const Joints & q, bool full = true) const
  {
    Metrics m;
    m.q = q;
    const auto jm = joint_map(q);
    m.joint_margin = 1e9;
    for (int i = 0; i < 6; ++i)
      m.joint_margin = std::min({m.joint_margin, q(i) - limits_.q_min(i), limits_.q_max(i) - q(i)});
    const Eigen::Isometry3d panel = projector_->panel_pose(jm);
    const Vector3d normal = panel.linear().col(2);
    m.tilt = std::acos(std::clamp(normal.z(), -1.0, 1.0));
    m.yaw = mmg::rpy_of(panel.linear())[2];
    m.panel_x = panel.translation().x();
    m.panel_z = panel.translation().z();
    const Vector3d half = 0.5 * payload_.size;
    m.top = -1e9;
    m.bottom = 1e9;
    for (int c = 0; c < 8; ++c) {
      const Vector3d corner = panel * Vector3d((c & 1 ? 1 : -1) * half.x(), (c & 2 ? 1 : -1) * half.y(),
                                               (c & 4 ? 1 : -1) * half.z());
      m.top = std::max(m.top, corner.z());
      m.bottom = std::min(m.bottom, corner.z());
    }
    if (!full) return m;

    // Collision distances.
    std::map<std::string, Eigen::Isometry3d> link_poses;
    std::vector<Solid> solids;
    Solid panel_solid;
    for (const auto & e : entries_) {
      auto it = link_poses.find(e.link);
      if (it == link_poses.end()) it = link_poses.emplace(e.link, projector_->link_pose(e.link, jm)).first;
      Solid s{e.panel ? "panel" : e.link, it->second * e.local, e.geometry};
      if (e.panel) panel_solid = s;
      else solids.push_back(s);
    }
    auto distance = [](const Solid & a, const Solid & b) {
      const fcl::DistanceRequestd request(false, false, 0.0, 0.0, 1e-8, fcl::GST_LIBCCD);
      fcl::DistanceResultd result;
      fcl::distance(a.geometry.get(), a.pose, b.geometry.get(), b.pose, request, result);
      return result.min_distance;  // <= 0 when touching or penetrating
    };
    m.panel_clearance = m.panel_base_clearance = 1e9;
    for (const auto & s : solids) {
      if (s.name == "wrist_3_link") continue;  // the flange the panel is mounted on
      const double d = distance(panel_solid, s);
      if (kBaseLinks.count(s.name)) m.panel_base_clearance = std::min(m.panel_base_clearance, d);
      else if (d < m.panel_clearance) m.panel_clearance = d, m.panel_closest = s.name;
    }
    m.self_clearance = m.arm_clearance = 1e9;
    auto arm_index = [](const std::string & n) {
      const auto it = std::find(kArmLinks.begin(), kArmLinks.end(), n);
      return it == kArmLinks.end() ? -1 : static_cast<int>(it - kArmLinks.begin());
    };
    for (size_t i = 0; i < solids.size(); ++i) {
      const int ai = arm_index(solids[i].name);
      if (ai < 0) continue;
      for (size_t j = 0; j < solids.size(); ++j) {
        const auto & other = solids[j].name;
        const int aj = arm_index(other);
        if (aj >= 0 && (aj <= ai + 1)) continue;  // each non-adjacent arm pair once
        if (aj < 0 && !kBaseLinks.count(other)) continue;
        if (ai == 0 && other == "shoulder_pan_link") continue;  // adjacent to the upper arm
        const double d = distance(solids[i], solids[j]);
        if (aj >= 0) {
          if (d < m.arm_clearance) m.arm_clearance = d, m.arm_closest = solids[i].name + "/" + other;
        } else if (d < m.self_clearance) {
          m.self_clearance = d, m.self_closest = solids[i].name + "/" + other;
        }
      }
    }

    // Ground footprint: hull of every primitive and the panel, plus the perimeter allowance.
    const auto hull = mmg::convex_hull(projector_->projected_points(jm));
    m.fp_xmin = m.fp_ymin = 1e9;
    m.fp_xmax = m.fp_ymax = -1e9;
    for (const auto & p : hull) {
      m.fp_xmin = std::min(m.fp_xmin, p[0] - kPerimeterAllowance);
      m.fp_xmax = std::max(m.fp_xmax, p[0] + kPerimeterAllowance);
      m.fp_ymin = std::min(m.fp_ymin, p[1] - kPerimeterAllowance);
      m.fp_ymax = std::max(m.fp_ymax, p[1] + kPerimeterAllowance);
    }
    m.fp_area = mmg::area(hull);

    mmm::State x = mmm::State::Zero();
    x.tail<6>() = q;
    m.manip = mmm::manipulability(x);

    // Panel lever about the shoulder_lift and elbow axes (horizontal, perpendicular to the axis;
    // positive when the COM is on the axis's +x side for shoulder_pan = 0).
    const Vector3d com = projector_->link_pose(payload_.link, jm) * payload_.center;
    auto lever = [&](const std::string & child) {
      const Eigen::Isometry3d frame = projector_->link_pose(child, jm);
      const Vector3d axis = frame.linear() * Vector3d::UnitY();
      Vector3d r = com - frame.translation();
      r -= r.dot(axis) * axis;
      r.z() = 0.0;
      return r.dot(Vector3d(axis.y(), -axis.x(), 0.0).normalized());  // the horizontal normal to the axis
    };
    m.lever_shoulder = lever("upper_arm_link");
    m.lever_elbow = lever("forearm_link");

    // Static gravity torque about a joint: the arm links' masses below it plus the panel.
    auto torque = [&](const std::string & child) {
      const Eigen::Isometry3d frame = projector_->link_pose(child, jm);
      const auto joint = projector_->model().getLink(child)->parent_joint;
      const Vector3d axis = frame.linear() * Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
      const Vector3d force_dir(0.0, 0.0, -kGravity);
      double tau = axis.dot((com - frame.translation()).cross(mass_ * force_dir));
      std::vector<std::string> stack = {child};
      while (!stack.empty()) {
        const auto link = projector_->model().getLink(stack.back());
        stack.pop_back();
        if (link->inertial) {
          const auto & o = link->inertial->origin.position;
          const Vector3d c = projector_->link_pose(link->name, jm) * Vector3d(o.x, o.y, o.z);
          tau += axis.dot((c - frame.translation()).cross(link->inertial->mass * force_dir));
        }
        for (const auto & child_link : link->child_links) stack.push_back(child_link->name);
      }
      return tau;
    };
    m.tau_shoulder = torque("upper_arm_link");
    m.tau_elbow = torque("forearm_link");
    m.tau_wrist2 = torque("wrist_2_link");

    m.dist_vc = (q - named_.at("vertical_carry")).norm();
    m.time_vc = move_time(q, named_.at("vertical_carry"));
    m.time_home = move_time(q, named_.at("home"));
    return m;
  }

  double floor() const { return floor_; }

private:
  struct Entry
  {
    std::string link;
    bool panel;
    Eigen::Isometry3d local;
    std::shared_ptr<fcl::CollisionGeometryd> geometry;
  };
  nlohmann::ordered_json payload_json_;
  std::string urdf_;
  mmg::Payload payload_;
  std::unique_ptr<mmg::FootprintProjector> projector_;
  mmm::Limits limits_;
  double mass_ = 0, floor_ = 0;
  std::map<std::string, Joints> named_;
  std::vector<Entry> entries_;
  Metrics home_;
};

struct Options
{
  double step = 0.02;
  double level_tol = 0.01;
  double joint_margin = 0.2;
  double clearance = 0.05;
  double top_min = 1.20, top_max = 1.35;
  double min_elbow = 0.8;
  std::string csv;
  std::vector<Joints> eval;
  std::string mirror_of;
};

// The panel's centre and axes (columns) in base_footprint with the arm at q.
Eigen::Isometry3d panel_in_base(const Joints & q)
{
  mmm::State x = mmm::State::Zero();
  x.tail<6>() = q;
  return mmm::link_pose(mmm::link_index("panel"), x);
}

struct MirrorSolution
{
  Joints q;
  double position_residual = 0, axis_residual = 0;  // m; rad (largest axis angle up to sign)
  bool converged = false;
};

// Damped least squares for the mirrored panel pose of `source` from `seed`: the residual stacks the
// centre error and, per axis, the error to the mirrored axis with the sign closest at the seed.
MirrorSolution solve_mirror(const Joints & source, Joints seed, const mmm::Limits & L, double joint_margin)
{
  const Eigen::Isometry3d target = panel_in_base(source);
  const Eigen::Matrix3d M = Eigen::Vector3d(1.0, -1.0, 1.0).asDiagonal();
  const Eigen::Vector3d p_star = M * target.translation();
  Eigen::Matrix3d a_star = M * target.linear();  // mirrored axes (columns); signs fixed below
  const Eigen::Matrix3d a0 = panel_in_base(seed).linear();
  for (int i = 0; i < 3; ++i)
    if (a0.col(i).dot(a_star.col(i)) < 0.0) a_star.col(i) = -a_star.col(i);
  const Joints lo = L.q_min.array() + joint_margin, hi = L.q_max.array() - joint_margin;
  auto residual = [&](const Joints & q) {
    const Eigen::Isometry3d pose = panel_in_base(q);
    Eigen::Matrix<double, 12, 1> r;
    r.head<3>() = pose.translation() - p_star;
    for (int i = 0; i < 3; ++i) r.segment<3>(3 + 3 * i) = pose.linear().col(i) - a_star.col(i);
    return r;
  };
  Joints q = seed.cwiseMax(lo).cwiseMin(hi);
  constexpr double kLambda = 0.05, kStep = 1e-6;
  for (int it = 0; it < 200; ++it) {
    const auto r = residual(q);
    if (r.norm() < 1e-9) break;
    Eigen::Matrix<double, 12, 6> J;
    for (int j = 0; j < 6; ++j) {
      Joints dq = q;
      dq(j) += kStep;
      J.col(j) = (residual(dq) - r) / kStep;
    }
    const Eigen::Matrix<double, 6, 6> H = J.transpose() * J + kLambda * kLambda * Eigen::Matrix<double, 6, 6>::Identity();
    q = (q - H.ldlt().solve(J.transpose() * r)).cwiseMax(lo).cwiseMin(hi);
  }
  MirrorSolution s;
  s.q = q;
  const Eigen::Isometry3d pose = panel_in_base(q);
  s.position_residual = (pose.translation() - p_star).norm();
  for (int i = 0; i < 3; ++i)
    s.axis_residual = std::max(s.axis_residual, std::acos(std::clamp(std::abs(pose.linear().col(i).dot(a_star.col(i))), -1.0, 1.0)));
  s.converged = s.position_residual < 1e-3 && s.axis_residual < 1e-3;
  return s;
}

// Rejection reason for the hard constraints ("" when feasible).
std::string check(const Metrics & m, const Metrics & home, const Options & o, double floor)
{
  if (m.joint_margin < o.joint_margin) return "joint margin";
  if (m.tilt > o.level_tol || std::abs(m.yaw - home.yaw) > o.level_tol) return "not level";
  if (m.bottom < floor) return "floor";
  if (std::min(m.panel_clearance, m.panel_base_clearance) < o.clearance) return "panel clearance";
  if (m.self_clearance < o.clearance) return "arm-base clearance";
  if (m.arm_clearance <= 0.0) return "arm self-collision";
  const double tol = 1e-3;
  if (m.fp_xmax - m.fp_xmin > home.fp_xmax - home.fp_xmin + tol || m.fp_ymax - m.fp_ymin > home.fp_ymax - home.fp_ymin + tol)
    return "footprint";
  return "";
}

// Ranking cost (lower is better), each term normalized by a scale the user can read: shoulder
// lever per 0.05 m, the synchronized move time to vertical_carry per 1 s, and the panel top's
// distance from the target height per 0.05 m; manipulability per 0.01 m^3 earns.
double cost(const Metrics & m, double top_target)
{
  return std::abs(m.lever_shoulder) / 0.05 + m.time_vc / 1.0 + std::abs(m.top - top_target) / 0.05 - m.manip / 0.01;
}

// One column per pose, one row per metric.
void print_table(const std::vector<std::pair<std::string, Metrics>> & poses)
{
  auto row = [&](const char * label, auto format) {
    std::printf("%-34s", label);
    for (const auto & [name, m] : poses) std::printf(" %-30s", format(m).c_str());
    std::printf("\n");
  };
  auto f = [](const char * fmt, auto... v) {
    char buffer[96];
    std::snprintf(buffer, sizeof buffer, fmt, v...);
    return std::string(buffer);
  };
  std::printf("%-34s", "");
  for (const auto & [name, m] : poses) std::printf(" %-30s", name.c_str());
  std::printf("\n");
  for (int i = 0; i < 6; ++i)
    row(mmm::kArmJoints[i], [&](const Metrics & m) { return f("%.3f", m.q(i)); });
  row("panel top / centre z (m)", [&](const Metrics & m) { return f("%.3f / %.3f", m.top, m.panel_z); });
  row("panel bottom (m)", [&](const Metrics & m) { return f("%.3f", m.bottom); });
  row("panel centre x (m)", [&](const Metrics & m) { return f("%.3f", m.panel_x); });
  row("panel tilt / yaw (rad)", [&](const Metrics & m) { return f("%.4f / %.4f", m.tilt, m.yaw); });
  row("min joint margin (rad)", [&](const Metrics & m) { return f("%.3f", m.joint_margin); });
  row("panel-arm clearance (m)", [&](const Metrics & m) { return f("%.3f %s", m.panel_clearance, m.panel_closest.c_str()); });
  row("panel-base clearance (m)", [&](const Metrics & m) { return f("%.3f", m.panel_base_clearance); });
  row("arm-base clearance (m)", [&](const Metrics & m) { return f("%.3f %s", m.self_clearance, m.self_closest.c_str()); });
  row("arm-arm clearance (m)", [&](const Metrics & m) { return f("%.3f %s", m.arm_clearance, m.arm_closest.c_str()); });
  row("footprint x (m, +0.02)", [&](const Metrics & m) { return f("%.3f..%.3f", m.fp_xmin, m.fp_xmax); });
  row("footprint y (m, +0.02)", [&](const Metrics & m) { return f("%.3f..%.3f", m.fp_ymin, m.fp_ymax); });
  row("footprint size (m)", [&](const Metrics & m) { return f("%.3f x %.3f", m.fp_xmax - m.fp_xmin, m.fp_ymax - m.fp_ymin); });
  row("manipulability (m^3)", [&](const Metrics & m) { return f("%.4f", m.manip); });
  row("panel lever shoulder/elbow (m)", [&](const Metrics & m) { return f("%.3f / %.3f", m.lever_shoulder, m.lever_elbow); });
  row("panel moment shoulder/elbow (Nm)", [&](const Metrics & m) {
    return f("%.1f / %.1f", 3.0 * kGravity * m.lever_shoulder, 3.0 * kGravity * m.lever_elbow); });
  row("gravity torque sh/el/w2 (Nm)", [&](const Metrics & m) {
    return f("%.1f / %.1f / %.1f", m.tau_shoulder, m.tau_elbow, m.tau_wrist2); });
  row("|q - vertical_carry| (rad)", [&](const Metrics & m) { return f("%.3f", m.dist_vc); });
  row("move time to vc / home (s)", [&](const Metrics & m) { return f("%.2f / %.2f", m.time_vc, m.time_home); });
}

Joints parse_joints(const std::string & text)
{
  Joints q;
  std::stringstream stream(text);
  std::string item;
  for (int i = 0; i < 6; ++i) {
    if (!std::getline(stream, item, ',')) throw std::invalid_argument("--eval needs six comma-separated values");
    q(i) = std::stod(item);
  }
  return q;
}
}  // namespace

int main(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() {
      if (i + 1 >= argc) throw std::invalid_argument(a + " needs a value");
      return std::string(argv[++i]);
    };
    if (a == "--step") o.step = std::stod(next());
    else if (a == "--top-min") o.top_min = std::stod(next());
    else if (a == "--top-max") o.top_max = std::stod(next());
    else if (a == "--min-elbow") o.min_elbow = std::stod(next());
    else if (a == "--csv") o.csv = next();
    else if (a == "--eval") o.eval.push_back(parse_joints(next()));
    else if (a == "--mirror-of") o.mirror_of = next();
    else throw std::invalid_argument("unknown argument " + a);
  }

  const Evaluator ev;
  const auto & L = ev.limits();
  std::vector<std::pair<std::string, Metrics>> references;
  for (const auto & name : {"home", "vertical_carry", "level_extension"}) references.emplace_back(name, ev.evaluate(ev.named(name)));
  for (const auto & q : o.eval) {
    const Metrics m = ev.evaluate(q);
    const std::string why = check(m, ev.home(), o, ev.floor());
    references.emplace_back("eval: " + (why.empty() ? std::string("feasible") : why), m);
  }
  if (!o.mirror_of.empty()) {
    const Joints source = ev.named(o.mirror_of);
    std::optional<MirrorSolution> best;
    for (const double s5 : {1.0, -1.0}) {
      for (const double s6 : {1.0, -1.0}) {
        Joints seed = source;
        seed(0) = -source(0);
        seed(4) = s5 * source(4);
        seed(5) = s6 * source(5);
        const MirrorSolution s = solve_mirror(source, seed, L, o.joint_margin);
        std::printf("seed [%.3f %.3f %.3f %.3f %.3f %.3f]: q [%.4f %.4f %.4f %.4f %.4f %.4f], residual %.2e m, %.2e rad%s\n",
                    seed(0), seed(1), seed(2), seed(3), seed(4), seed(5), s.q(0), s.q(1), s.q(2), s.q(3), s.q(4), s.q(5),
                    s.position_residual, s.axis_residual, s.converged ? "" : " (not converged)");
        if (s.converged && (!best || s.position_residual + s.axis_residual < best->position_residual + best->axis_residual))
          best = s;
      }
    }
    if (!best) {
      std::printf("No mirror of %s within the joint limits less %.2f rad\n", o.mirror_of.c_str(), o.joint_margin);
      return 2;
    }
    const Metrics m = ev.evaluate(best->q);
    // The level and footprint checks are carry-pose rules; a mirrored pose keeps the others.
    std::string why;
    if (m.joint_margin < o.joint_margin - 1e-6) why = "joint margin";  // the solver may end on its own clamp
    else if (m.bottom < ev.floor()) why = "floor";
    else if (std::min(m.panel_clearance, m.panel_base_clearance) < o.clearance) why = "panel clearance";
    else if (m.self_clearance < o.clearance) why = "arm-base clearance";
    else if (m.arm_clearance <= 0.0) why = "arm self-collision";
    references = {{o.mirror_of, ev.evaluate(source)}, {"mirror: " + (why.empty() ? std::string("feasible") : why), m}};
    std::printf("Mirror of %s (residual %.2e m, %.2e rad):\n", o.mirror_of.c_str(), best->position_residual,
                best->axis_residual);
    print_table(references);
    return why.empty() ? 0 : 2;
  }
  std::printf("Reference poses (panel moment: payload %.1f kg only; gravity torque: arm links + payload):\n", 3.0);
  print_table(references);
  if (!o.eval.empty()) return 0;

  // Grid over shoulder_lift, elbow, wrist_1; wrist_2 solved for a level panel.
  std::map<std::string, int> rejected;
  std::vector<Metrics> feasible;
  long samples = 0;
  const double w1_step = std::max(o.step, 0.25);
  std::vector<double> wrist_1 = {0.0};
  for (double w1 = w1_step; w1 <= L.q_max(3) - o.joint_margin + 1e-9; w1 += w1_step) wrist_1.push_back(w1), wrist_1.push_back(-w1);
  for (double sl = L.q_min(1) + o.joint_margin; sl <= L.q_max(1) - o.joint_margin + 1e-9; sl += o.step) {
    for (double el = L.q_min(2) + o.joint_margin; el <= L.q_max(2) - o.joint_margin + 1e-9; el += o.step) {
      for (const double w1 : wrist_1) {
        ++samples;
        Joints q;
        q << 0.0, sl, el, w1, 0.0, 0.0;
        q(4) = ev.level_wrist_2(q);
        // Cheap filters first.
        Metrics m = ev.evaluate(q, false);
        if (std::abs(el) < o.min_elbow) { ++rejected["elbow not bent"]; continue; }
        if (m.tilt > o.level_tol || std::abs(m.yaw - ev.home().yaw) > o.level_tol) { ++rejected["not level"]; continue; }
        if (m.top < o.top_min || m.top > o.top_max) { ++rejected["panel top outside band"]; continue; }
        if (m.joint_margin < o.joint_margin) { ++rejected["joint margin"]; continue; }
        m = ev.evaluate(q);
        const std::string why = check(m, ev.home(), o, ev.floor());
        if (!why.empty()) { ++rejected[why]; continue; }
        feasible.push_back(m);
      }
    }
  }
  std::printf("\n%ld samples (step %.3f rad, wrist_1 step %.2f rad); %zu feasible; rejected:", samples, o.step, w1_step,
              feasible.size());
  for (const auto & [why, n] : rejected) std::printf(" %s %d;", why.c_str(), n);
  std::printf("\n");
  if (!o.csv.empty()) {
    std::ofstream out(o.csv);
    out << "q1,q2,q3,q4,q5,q6,top,panel_x,tilt,joint_margin,panel_clearance,panel_base_clearance,self_clearance,"
           "arm_clearance,fp_xmin,fp_xmax,fp_ymin,fp_ymax,manip,lever_shoulder,lever_elbow,tau_shoulder,tau_elbow,"
           "tau_wrist2,dist_vc,time_vc,time_home\n";
    for (const auto & m : feasible) {
      for (int i = 0; i < 6; ++i) out << m.q(i) << ",";
      out << m.top << "," << m.panel_x << "," << m.tilt << "," << m.joint_margin << "," << m.panel_clearance << ","
          << m.panel_base_clearance << "," << m.self_clearance << "," << m.arm_clearance << "," << m.fp_xmin << ","
          << m.fp_xmax << "," << m.fp_ymin << "," << m.fp_ymax << "," << m.manip << "," << m.lever_shoulder << ","
          << m.lever_elbow << "," << m.tau_shoulder << "," << m.tau_elbow << "," << m.tau_wrist2 << "," << m.dist_vc
          << "," << m.time_vc << "," << m.time_home << "\n";
    }
  }
  if (feasible.empty()) return 1;

  // One candidate per panel height (low, middle, high in the band): the best grid sample for
  // that height, refined by a pattern search on shoulder_lift and elbow (wrist_2 re-solved).
  auto refine = [&](Metrics best, double target) {
    for (double h = o.step / 2; h >= 1e-3; h /= 2) {
      bool improved = true;
      while (improved) {
        improved = false;
        for (const auto & d : {Eigen::Vector2d(1, 0), Eigen::Vector2d(-1, 0), Eigen::Vector2d(0, 1), Eigen::Vector2d(0, -1)}) {
          Joints q = best.q;
          q(1) += h * d(0);
          q(2) += h * d(1);
          q(4) = ev.level_wrist_2(q);
          const Metrics m = ev.evaluate(q);
          if (std::abs(q(2)) < o.min_elbow || m.top < o.top_min || m.top > o.top_max) continue;
          if (!check(m, ev.home(), o, ev.floor()).empty()) continue;
          if (cost(m, target) < cost(best, target) - 1e-9) best = m, improved = true;
        }
      }
    }
    Joints q = best.q.unaryExpr([](double v) { return std::round(v * 1000.0) / 1000.0; });
    q(4) = std::round(ev.level_wrist_2(q) * 1000.0) / 1000.0;
    return ev.evaluate(q);
  };
  std::vector<std::pair<std::string, Metrics>> candidates;
  const double band = o.top_max - o.top_min;
  for (const auto & [label, target] : {std::pair<std::string, double>{"low", o.top_min + 0.2 * band},
                                       {"middle", o.top_min + 0.5 * band}, {"high", o.top_min + 0.8 * band}}) {
    const auto seed = std::min_element(feasible.begin(), feasible.end(), [&, t = target](const Metrics & a, const Metrics & b) {
      return cost(a, t) < cost(b, t);
    });
    const Metrics m = refine(*seed, target);
    const std::string why = check(m, ev.home(), o, ev.floor());
    char name[96];
    std::snprintf(name, sizeof name, "%s (top %.3f) cost %.2f%s%s", label.c_str(), target, cost(m, target),
                  why.empty() ? "" : " ", why.c_str());
    candidates.emplace_back(name, m);
  }
  // The mirrored branch (upper arm leaning back, elbow behind the mount) at the middle height.
  std::vector<Metrics> mirrored;
  for (const auto & m : feasible) if (m.q(1) < 0.0) mirrored.push_back(m);
  if (!mirrored.empty()) {
    const double target = o.top_min + 0.5 * band;
    const auto seed = std::min_element(mirrored.begin(), mirrored.end(),
                                       [&](const Metrics & a, const Metrics & b) { return cost(a, target) < cost(b, target); });
    const Metrics m = refine(*seed, target);
    char name[96];
    std::snprintf(name, sizeof name, "mirrored middle cost %.2f", cost(m, target));
    candidates.emplace_back(name, m);
  }
  std::printf("\nCandidates (rounded to 1 mrad; cost = |lever_sh|/0.05 m + t_vc/1 s + |top - target|/0.05 m - manip/0.01):\n");
  print_table(candidates);
  return 0;
}
