#include "mobile_manipulator_plant/plant.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
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

void Plant::reset(const ScenarioSetup & scenario, const Pose2 & start, const Vec6 & q0, uint64_t seed)
{
  // build and index the new model before touching the plant's members: a throw leaves the previous run intact
  std::unique_ptr<mjModel, void (*)(mjModel *)> m(build_model(profile_, site_mjcf_, scenario), mj_deleteModel);
  int qadr_base[3], vadr_base[3], qadr_arm[6], vadr_arm[6];
  Vec6 q_lo, q_hi;
  const char * base_joints[3] = {"base_x", "base_y", "base_yaw"};
  for (int i = 0; i < 3; ++i) {
    const int j = joint_id(m.get(), base_joints[i]);
    qadr_base[i] = m->jnt_qposadr[j];
    vadr_base[i] = m->jnt_dofadr[j];
  }
  for (int i = 0; i < 6; ++i) {
    const int j = joint_id(m.get(), profile_.arm_joints[i]);
    qadr_arm[i] = m->jnt_qposadr[j];
    vadr_arm[i] = m->jnt_dofadr[j];
    q_lo(i) = m->jnt_range[2 * j];
    q_hi(i) = m->jnt_range[2 * j + 1];
  }
  std::vector<int> robot_geoms, world_geoms;
  for (int g = 0; g < m->ngeom; ++g) {
    const char * name = mj_id2name(m.get(), mjOBJ_GEOM, g);
    if (starts_with(name, "robot/")) robot_geoms.push_back(g);
    else if (starts_with(name, "site/") || starts_with(name, "scenario/") || starts_with(name, "mover/")) world_geoms.push_back(g);
  }
  std::vector<Mover> movers;
  for (const MoverSpec & spec : scenario.movers) {
    const int body = mj_name2id(m.get(), mjOBJ_BODY, ("mover/" + spec.name).c_str());
    movers.push_back({MoverMotion(spec), m->body_mocapid[body], mj_name2id(m.get(), mjOBJ_GEOM, ("mover/" + spec.name).c_str()), spec.size.z()});
  }
  mjData * d = mj_makeData(m.get());
  if (!d) throw std::runtime_error("plant: mj_makeData failed");

  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
  m_ = m.release();
  d_ = d;
  std::copy(qadr_base, qadr_base + 3, qadr_base_);
  std::copy(vadr_base, vadr_base + 3, vadr_base_);
  std::copy(qadr_arm, qadr_arm + 6, qadr_arm_);
  std::copy(vadr_arm, vadr_arm + 6, vadr_arm_);
  q_lo_ = q_lo;
  q_hi_ = q_hi;
  panel_body_ = mj_name2id(m_, mjOBJ_BODY, "panel");
  robot_geoms_ = std::move(robot_geoms);
  world_geoms_ = std::move(world_geoms);
  movers_ = std::move(movers);

  d_->qpos[qadr_base_[0]] = start.x;
  d_->qpos[qadr_base_[1]] = start.y;
  d_->qpos[qadr_base_[2]] = start.yaw;
  for (int i = 0; i < 6; ++i) d_->qpos[qadr_arm_[i]] = std::clamp(q0(i), q_lo_(i), q_hi_(i));
  executed_.setZero();
  steps_ = 0;
  seed_ = seed;
  d_->time = 0.0;
  if (mode_ == PlantMode::Realistic) {
    const char * drives[3] = {"drive/base_x", "drive/base_y", "drive/base_yaw"};
    for (int i = 0; i < 3; ++i) drive_act_[i] = mj_name2id(m_, mjOBJ_ACTUATOR, drives[i]);
    BaseDriveParams bp;
    bp.limits = profile_.base_limits;
    bp.wheel_radius_m = profile_.wheel_radius_m;
    bp.effective_track_m = profile_.effective_track_m;
    bp.wheel_speed_max = profile_.wheel_speed_max;
    bp.breakaway = profile_.base_breakaway;
    bp.breakaway_linear = profile_.breakaway_linear_mps;
    bp.breakaway_yaw = profile_.breakaway_yaw_radps;
    bp.rest_linear = profile_.rest_linear_mps;
    bp.rest_yaw = profile_.rest_yaw_radps;
    bp.tau_v = profile_.base_lag_s(0);
    bp.tau_w = profile_.base_lag_s(1);
    bp.k_v = profile_.base_slip(0);
    bp.k_w = profile_.base_slip(1);
    drive_ = std::make_unique<BaseDrive>(bp);
    servo_ = std::make_unique<ArmServo>(m_, profile_, profile_.gravity_ff, profile_.motion_ff);
    jtc_ = std::make_unique<JtcSampler>(profile_.jtc_rate_hz, profile_.ff_accel_limit);
    commands_ = DelayQueue<PlantCommand>(profile_.command_delay_s);
    states_ = DelayQueue<PlantState>(profile_.state_delay_s);
    for (int i = 0; i < 6; ++i) q_ref_(i) = d_->qpos[qadr_arm_[i]];
    jtc_->reset(q_ref_);
    mj_fwdPosition(m_, d_);
    servo_->apply(d_, {q_ref_, Vec6::Zero(), Vec6::Zero()});  // hold the start pose (with its gravity feedforward)
    for (int i = 0; i < 3; ++i) d_->ctrl[drive_act_[i]] = 0.0;
  }
  mj_fwdPosition(m_, d_);
  if (lidar_) lidar_->attach(m_, d_, seed_);
  read_state();
}

const PlantState & Plant::step(const PlantCommand & command, double dt)
{
  if (!m_) throw std::logic_error("plant: step before reset");
  const double n = dt / kPhysicsStepS;
  const long substeps = std::lround(n);
  if (substeps < 1 || std::abs(n - static_cast<double>(substeps)) * kPhysicsStepS > 1e-9)
    throw std::invalid_argument("plant: dt " + std::to_string(dt) + " is not a multiple of the 2 ms physics step");
  if (mode_ == PlantMode::Realistic) {
    commands_.push(d_->time, command);  // it arrives after the command delay
    for (long k = 0; k < substeps; ++k) realistic_substep();
    read_state();
    return state_;
  }
  if (!command.arm.joint_velocity) throw std::invalid_argument("plant: the ideal mode takes joint velocities");
  // the base: the commanded twist, or nothing from rest below the breakaway, judged once per command as the
  // kinematic plant does (KinematicPlant::step)
  Eigen::Vector2d u(command.v, command.omega);
  if (ideal_breakaway_ && base_at_rest() && std::abs(u(0)) < profile_.breakaway_linear_mps &&
      std::abs(u(1)) < profile_.breakaway_yaw_radps)
    u.setZero();
  for (long k = 0; k < substeps; ++k) substep(u, *command.arm.joint_velocity);
  read_state();
  return state_;
}

void Plant::substep(const Eigen::Vector2d & u, const Vec6 & qd)
{
  const double h = kPhysicsStepS;
  executed_ = u;
  double & x = d_->qpos[qadr_base_[0]];
  double & y = d_->qpos[qadr_base_[1]];
  double & yaw = d_->qpos[qadr_base_[2]];
  // the exact arc of a constant (v, omega): the chord 2 sin(omega h / 2) / omega along the mid heading
  const double half = 0.5 * u(1) * h;
  const double chord = std::abs(half) > 1e-9 ? u(0) * std::sin(half) / (0.5 * u(1)) : u(0) * h;
  x += chord * std::cos(yaw + half);
  y += chord * std::sin(yaw + half);
  yaw += u(1) * h;
  d_->qvel[vadr_base_[0]] = u(0) * std::cos(yaw);
  d_->qvel[vadr_base_[1]] = u(0) * std::sin(yaw);
  d_->qvel[vadr_base_[2]] = u(1);
  // arm: integrate the joint velocities, stopped at the joint limits (the commanded velocity unless clamped)
  for (int i = 0; i < 6; ++i) {
    double & q = d_->qpos[qadr_arm_[i]];
    const double free = q + qd(i) * h;
    const double next = std::clamp(free, q_lo_(i), q_hi_(i));
    d_->qvel[vadr_arm_[i]] = next == free ? qd(i) : (next - q) / h;
    q = next;
  }
  ++steps_;
  d_->time = steps_ * h;
  finish_substep();
}

void Plant::realistic_substep()
{
  const double h = kPhysicsStepS, t = d_->time;
  double arrived = 0.0;
  const std::optional<PlantCommand> command = commands_.at(t, &arrived);
  // the base: Unity's limiter on the command that has arrived (none yet: stand), applied as a world-frame target of the
  // planar velocity servos (zero lateral velocity in the body frame)
  const Eigen::Vector2d wanted = command ? Eigen::Vector2d(command->v, command->omega) : Eigen::Vector2d::Zero();
  const Eigen::Vector2d u = drive_->target(wanted, command ? t - arrived : 0.0, h);
  const double yaw = d_->qpos[qadr_base_[2]];
  d_->ctrl[drive_act_[0]] = u(0) * std::cos(yaw);
  d_->ctrl[drive_act_[1]] = u(0) * std::sin(yaw);
  d_->ctrl[drive_act_[2]] = u(1);
  // the arm: joint velocities integrated into the reference, or the stream as the JTC samples it
  ArmTarget target{q_ref_, Vec6::Zero(), Vec6::Zero()};
  if (command && command->arm.joint_velocity) {
    const Vec6 & qd = *command->arm.joint_velocity;
    q_ref_ = (q_ref_ + h * qd).cwiseMax(q_lo_).cwiseMin(q_hi_);
    target = {q_ref_, qd, Vec6::Zero()};
  } else if (command) {
    target = jtc_->sample(command->arm, t);
    q_ref_ = target.q;
  }
  servo_->apply(d_, target);
  mj_step(m_, d_);
  ++steps_;
  d_->time = steps_ * h;
  finish_substep();
}

void Plant::finish_substep()
{
  update_movers(d_->time);
  mj_fwdPosition(m_, d_);  // current kinematics and contacts (ideal: logged only; realistic: mj_step pushed already)
  if (lidar_) lidar_->advance(m_, d_, d_->time, kPhysicsStepS);
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
  // ideal: the twist executed last; realistic: the measured one
  const Eigen::Vector2d twist = mode_ == PlantMode::Realistic ? Eigen::Vector2d(truth_.v, truth_.omega) : executed_;
  return std::abs(twist(0)) < profile_.rest_linear_mps && std::abs(twist(1)) < profile_.rest_yaw_radps;
}

void Plant::read_state()
{
  PlantState & s = truth_;
  s.t = d_->time;
  s.base = {d_->qpos[qadr_base_[0]], d_->qpos[qadr_base_[1]], d_->qpos[qadr_base_[2]]};
  if (mode_ == PlantMode::Realistic) {  // the measured twist in the body frame
    const double c = std::cos(s.base.yaw), sn = std::sin(s.base.yaw);
    s.v = c * d_->qvel[vadr_base_[0]] + sn * d_->qvel[vadr_base_[1]];
    s.omega = d_->qvel[vadr_base_[2]];
  } else {
    s.v = executed_(0);
    s.omega = executed_(1);
  }
  for (int i = 0; i < 6; ++i) {
    s.q(i) = d_->qpos[qadr_arm_[i]];
    s.qd(i) = d_->qvel[vadr_arm_[i]];
  }
  if (panel_body_ >= 0) {
    s.panel = Eigen::Isometry3d::Identity();
    s.panel.translation() = Eigen::Map<const Eigen::Vector3d>(d_->xpos + 3 * panel_body_);
    s.panel.linear() =
      Eigen::Quaterniond(d_->xquat[4 * panel_body_], d_->xquat[4 * panel_body_ + 1], d_->xquat[4 * panel_body_ + 2],
                         d_->xquat[4 * panel_body_ + 3])
        .toRotationMatrix();
  }
  if (mode_ == PlantMode::Realistic) {  // reported after the state delay (the first state until one is due)
    states_.push(s.t, s);
    const auto delayed = states_.at(s.t);
    state_ = delayed ? *delayed : s;
  } else {
    state_ = s;
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
      if (dist < cap && dist < truth.min_clearance_m) {  // dist == cap: farther than the cap, no witness
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

void Plant::enable_lidar(const std::string & pattern_path)
{
  ScanPattern pattern = load_pattern(pattern_path.empty() ? profile_.lidar_pattern_path : pattern_path);
  lidar_ = std::make_unique<Lidar>(profile_.lidar, std::move(pattern),
                                   profile_.lidar.scan_mode == "instant" ? ScanMode::Instant : ScanMode::Sweep);
  if (m_) lidar_->attach(m_, d_, seed_, d_->time);  // mid-run: the first scan covers the next period
}

std::optional<PlantScan> Plant::scan() { return lidar_ ? lidar_->take_scan() : std::nullopt; }

std::vector<Plant::MoverStatus> Plant::mover_status() const
{
  std::vector<MoverStatus> out;
  for (const Mover & mover : movers_)
    out.push_back({mover.motion.spec().name, mover.motion.position(), mover.motion.walked(), mover.motion.blocked_s(),
                   mover.motion.trigger_time().has_value(), mover.motion.finished()});
  return out;
}

double Plant::mover_robot_distance(const std::string & name) const
{
  double best = std::numeric_limits<double>::infinity();
  auto * d = const_cast<mjData *>(d_);
  for (const Mover & mover : movers_) {
    if (mover.motion.spec().name != name) continue;
    for (int g : robot_geoms_) best = std::min(best, mj_geomDistance(m_, d, mover.geom, g, 10.0, nullptr));
  }
  return best;
}

void Plant::move_body(const std::string & name, const Eigen::Vector3d & center, double yaw)
{
  const int body = mj_name2id(m_, mjOBJ_BODY, name.c_str());
  if (body < 0 || m_->body_mocapid[body] < 0) throw std::invalid_argument("plant: no mocap body " + name);
  if (starts_with(name.c_str(), "mover/")) throw std::invalid_argument("plant: " + name + " moves by its scenario motion");
  const int id = m_->body_mocapid[body];
  for (int i = 0; i < 3; ++i) d_->mocap_pos[3 * id + i] = center(i);
  d_->mocap_quat[4 * id] = std::cos(yaw / 2.0);
  d_->mocap_quat[4 * id + 1] = 0.0;
  d_->mocap_quat[4 * id + 2] = 0.0;
  d_->mocap_quat[4 * id + 3] = std::sin(yaw / 2.0);
  mj_fwdPosition(m_, d_);
}
}  // namespace mobile_manipulator_plant
