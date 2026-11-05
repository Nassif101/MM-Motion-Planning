#include "mobile_manipulator_plant/plant.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "mobile_manipulator_plant/model_builder.hpp"

namespace mobile_manipulator_plant
{
namespace
{
bool starts_with(const char * name, const char * prefix) { return name && std::string(name).rfind(prefix, 0) == 0; }

int joint_id(const mjModel * m, const std::string & name)
{
  const int id = mj_name2id(m, mjOBJ_JOINT, name.c_str());
  if (id < 0) throw std::runtime_error("plant: the model has no joint " + name);
  return id;
}
}  // namespace

Plant::Plant(RobotProfile profile, std::string site_mjcf, PlantMode mode)
: profile_(std::move(profile)), site_mjcf_(std::move(site_mjcf)), mode_(mode), ideal_breakaway_(profile_.ideal_breakaway)
{
}

Plant::~Plant()
{
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
}

void Plant::reset(const ScenarioSetup & scenario, const Pose2 & start, const Vec6 & q0, uint64_t)
{
  mjModel * m = build_model(profile_, site_mjcf_, scenario);
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
  m_ = m;
  d_ = mj_makeData(m_);

  const char * base_joints[3] = {"base_x", "base_y", "base_yaw"};
  for (int i = 0; i < 3; ++i) {
    const int j = joint_id(m_, base_joints[i]);
    qadr_base_[i] = m_->jnt_qposadr[j];
    vadr_base_[i] = m_->jnt_dofadr[j];
  }
  for (int i = 0; i < 6; ++i) {
    const int j = joint_id(m_, profile_.arm_joints[i]);
    qadr_arm_[i] = m_->jnt_qposadr[j];
    vadr_arm_[i] = m_->jnt_dofadr[j];
    q_lo_(i) = m_->jnt_range[2 * j];
    q_hi_(i) = m_->jnt_range[2 * j + 1];
  }
  panel_body_ = mj_name2id(m_, mjOBJ_BODY, "panel");
  robot_geoms_.clear();
  world_geoms_.clear();
  for (int g = 0; g < m_->ngeom; ++g) {
    const char * name = mj_id2name(m_, mjOBJ_GEOM, g);
    if (starts_with(name, "robot/")) robot_geoms_.push_back(g);
    else if (starts_with(name, "site/") || starts_with(name, "scenario/") || starts_with(name, "mover/")) world_geoms_.push_back(g);
  }
  movers_.clear();
  for (const MoverSpec & spec : scenario.movers) {
    const int body = mj_name2id(m_, mjOBJ_BODY, ("mover/" + spec.name).c_str());
    movers_.push_back({MoverMotion(spec), m_->body_mocapid[body], mj_name2id(m_, mjOBJ_GEOM, ("mover/" + spec.name).c_str()), spec.size.z()});
  }

  d_->qpos[qadr_base_[0]] = start.x;
  d_->qpos[qadr_base_[1]] = start.y;
  d_->qpos[qadr_base_[2]] = start.yaw;
  for (int i = 0; i < 6; ++i) d_->qpos[qadr_arm_[i]] = std::clamp(q0(i), q_lo_(i), q_hi_(i));
  executed_.setZero();
  steps_ = 0;
  d_->time = 0.0;
  mj_fwdPosition(m_, d_);
  read_state();
}

const PlantState & Plant::step(const PlantCommand & command, double dt)
{
  if (!m_) throw std::logic_error("plant: step before reset");
  const double n = dt / kPhysicsStepS;
  const long substeps = std::lround(n);
  if (substeps < 1 || std::abs(n - static_cast<double>(substeps)) * kPhysicsStepS > 1e-9)
    throw std::invalid_argument("plant: dt " + std::to_string(dt) + " is not a multiple of the 2 ms physics step");
  if (mode_ != PlantMode::Ideal) throw std::logic_error("plant: the realistic mode is not implemented yet");
  if (!command.arm.joint_velocity) throw std::invalid_argument("plant: the ideal mode takes joint velocities");
  for (long k = 0; k < substeps; ++k) substep(command);
  read_state();
  return state_;
}

void Plant::substep(const PlantCommand & command)
{
  const double h = kPhysicsStepS;
  // base: the commanded twist, or nothing from rest below the breakaway (the kinematic plant's static friction)
  const bool from_rest = base_at_rest();
  Eigen::Vector2d u(command.v, command.omega);
  if (ideal_breakaway_ && from_rest && std::abs(u(0)) < profile_.breakaway_linear_mps &&
      std::abs(u(1)) < profile_.breakaway_yaw_radps)
    u.setZero();
  executed_ = u;
  double & x = d_->qpos[qadr_base_[0]];
  double & y = d_->qpos[qadr_base_[1]];
  double & yaw = d_->qpos[qadr_base_[2]];
  // the exact arc of a constant (v, omega)
  if (std::abs(u(1)) > 1e-12) {
    const double r = u(0) / u(1), yaw1 = yaw + u(1) * h;
    x += r * (std::sin(yaw1) - std::sin(yaw));
    y -= r * (std::cos(yaw1) - std::cos(yaw));
    yaw = yaw1;
  } else {
    x += u(0) * h * std::cos(yaw);
    y += u(0) * h * std::sin(yaw);
  }
  d_->qvel[vadr_base_[0]] = u(0) * std::cos(yaw);
  d_->qvel[vadr_base_[1]] = u(0) * std::sin(yaw);
  d_->qvel[vadr_base_[2]] = u(1);
  // arm: integrate the joint velocities, stopped at the joint limits
  const Vec6 & qd = *command.arm.joint_velocity;
  for (int i = 0; i < 6; ++i) {
    double & q = d_->qpos[qadr_arm_[i]];
    const double next = std::clamp(q + qd(i) * h, q_lo_(i), q_hi_(i));
    d_->qvel[vadr_arm_[i]] = (next - q) / h;
    q = next;
  }
  ++steps_;
  d_->time = steps_ * h;
  update_movers(d_->time);
  mj_fwdPosition(m_, d_);  // kinematics and collision detection: contacts are logged, nothing is pushed
}

void Plant::update_movers(double t)
{
  if (movers_.empty()) return;
  const Eigen::Vector2d robot(d_->qpos[qadr_base_[0]], d_->qpos[qadr_base_[1]]);
  mj_kinematics(m_, d_);  // the robot's geoms where the base and arm now are
  for (Mover & mover : movers_) {
    const Eigen::Vector2d previous = mover.motion.position();
    const Eigen::Vector2d next = mover.motion.next(t, robot);
    if (next == previous) {
      mover.motion.commit(t, next);
      continue;
    }
    // the mover's box at its next position, held while any robot geom is within the stand-off (Unity's
    // ScenarioMover.RobotWithin: an overlap of the box grown by the stand-off)
    double * pos = d_->mocap_pos + 3 * mover.mocap;
    pos[0] = next.x();
    pos[1] = next.y();
    pos[2] = mover.height / 2.0;
    mj_kinematics(m_, d_);
    bool blocked = false;
    for (int g : robot_geoms_) {
      if (mj_geomDistance(m_, d_, mover.geom, g, kMoverStandOffM, nullptr) < kMoverStandOffM) {
        blocked = true;
        break;
      }
    }
    if (blocked) {
      pos[0] = previous.x();
      pos[1] = previous.y();
      mover.motion.hold(t);
    } else {
      mover.motion.commit(t, next);
    }
  }
}

bool Plant::base_at_rest() const
{
  return std::abs(executed_(0)) < profile_.rest_linear_mps && std::abs(executed_(1)) < profile_.rest_yaw_radps;
}

void Plant::read_state()
{
  state_.t = d_->time;
  state_.base = {d_->qpos[qadr_base_[0]], d_->qpos[qadr_base_[1]], d_->qpos[qadr_base_[2]]};
  state_.v = executed_(0);
  state_.omega = executed_(1);
  for (int i = 0; i < 6; ++i) {
    state_.q(i) = d_->qpos[qadr_arm_[i]];
    state_.qd(i) = d_->qvel[vadr_arm_[i]];
  }
  if (panel_body_ >= 0) {
    state_.panel = Eigen::Isometry3d::Identity();
    state_.panel.translation() = Eigen::Map<const Eigen::Vector3d>(d_->xpos + 3 * panel_body_);
    state_.panel.linear() =
      Eigen::Quaterniond(d_->xquat[4 * panel_body_], d_->xquat[4 * panel_body_ + 1], d_->xquat[4 * panel_body_ + 2],
                         d_->xquat[4 * panel_body_ + 3])
        .toRotationMatrix();
  }
}

GroundTruth Plant::ground_truth() const
{
  GroundTruth truth;
  // broad phase: bounding spheres within the reach of the closest pair found so far (at most 2 m)
  constexpr double kReach = 2.0;
  auto * d = const_cast<mjData *>(d_);  // mj_geomDistance reads positions only
  for (int r : robot_geoms_) {
    const Eigen::Map<const Eigen::Vector3d> pr(d_->geom_xpos + 3 * r);
    for (int w : world_geoms_) {
      const Eigen::Map<const Eigen::Vector3d> pw(d_->geom_xpos + 3 * w);
      const double cap = std::min(kReach, truth.min_clearance_m);
      if ((pr - pw).norm() - m_->geom_rbound[r] - m_->geom_rbound[w] > cap) continue;
      mjtNum fromto[6];
      const double dist = mj_geomDistance(m_, d, r, w, cap, fromto);
      if (dist < truth.min_clearance_m) {
        truth.min_clearance_m = dist;
        truth.robot_geom = mj_id2name(m_, mjOBJ_GEOM, r);
        truth.other_geom = mj_id2name(m_, mjOBJ_GEOM, w);
        truth.witness_robot = Eigen::Vector3d(fromto[0], fromto[1], fromto[2]);
        truth.witness_other = Eigen::Vector3d(fromto[3], fromto[4], fromto[5]);
      }
    }
  }
  for (int c = 0; c < d_->ncon; ++c) {
    const mjContact & con = d_->contact[c];
    const char * a = mj_id2name(m_, mjOBJ_GEOM, con.geom[0]);
    const char * b = mj_id2name(m_, mjOBJ_GEOM, con.geom[1]);
    const bool ar = starts_with(a, "robot/"), br = starts_with(b, "robot/");
    if (!ar && !br) continue;
    Contact contact;
    contact.robot_geom = ar ? a : b;
    contact.other_geom = ar ? (b ? b : "") : (a ? a : "");
    contact.depth = -con.dist;
    contact.pos = Eigen::Vector3d(con.pos[0], con.pos[1], con.pos[2]);
    truth.contacts.push_back(contact);
  }
  return truth;
}

std::vector<std::pair<std::string, Eigen::Vector2d>> Plant::mover_positions() const
{
  std::vector<std::pair<std::string, Eigen::Vector2d>> out;
  for (const Mover & mover : movers_) out.emplace_back(mover.motion.spec().name, mover.motion.position());
  return out;
}

void Plant::move_body(const std::string & name, const Eigen::Vector3d & center, double yaw)
{
  const int body = mj_name2id(m_, mjOBJ_BODY, name.c_str());
  if (body < 0 || m_->body_mocapid[body] < 0) throw std::invalid_argument("plant: no mocap body " + name);
  const int id = m_->body_mocapid[body];
  for (int i = 0; i < 3; ++i) d_->mocap_pos[3 * id + i] = center(i);
  d_->mocap_quat[4 * id] = std::cos(yaw / 2.0);
  d_->mocap_quat[4 * id + 1] = 0.0;
  d_->mocap_quat[4 * id + 2] = 0.0;
  d_->mocap_quat[4 * id + 3] = std::sin(yaw / 2.0);
  mj_fwdPosition(m_, d_);
}
}  // namespace mobile_manipulator_plant
