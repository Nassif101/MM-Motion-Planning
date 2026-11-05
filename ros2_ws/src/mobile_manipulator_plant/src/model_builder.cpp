#include "mobile_manipulator_plant/model_builder.hpp"

#include <cmath>
#include <cstdio>
#include <vector>
#include <memory>
#include <stdexcept>

namespace mobile_manipulator_plant
{
namespace
{
// Drops MuJoCo warnings containing one of `expected` while alive; others go to stderr as before.
class QuietWarnings
{
public:
  explicit QuietWarnings(std::vector<std::string> expected) : previous_(mju_user_warning)
  {
    expected_() = std::move(expected);
    mju_user_warning = &filter;
  }
  ~QuietWarnings() { mju_user_warning = previous_; }

private:
  static std::vector<std::string> & expected_()
  {
    static thread_local std::vector<std::string> e;
    return e;
  }
  static void filter(const char * message)
  {
    for (const auto & e : expected_())
      if (std::string(message).find(e) != std::string::npos) return;
    std::fprintf(stderr, "MuJoCo warning: %s\n", message);
  }
  void (*previous_)(const char *);
};

struct SpecDeleter
{
  void operator()(mjSpec * s) const { mj_deleteSpec(s); }
};
using SpecPtr = std::unique_ptr<mjSpec, SpecDeleter>;

SpecPtr parse(const std::string & path)
{
  char error[1000] = "";
  mjSpec * s = mj_parseXML(path.c_str(), nullptr, error, sizeof(error));
  if (!s) throw std::runtime_error(path + ": " + error);
  return SpecPtr(s);
}

void set_box(mjsGeom * g, const Eigen::Vector3d & center, const Eigen::Vector3d & size, double yaw)
{
  g->type = mjGEOM_BOX;
  for (int i = 0; i < 3; ++i) {
    g->pos[i] = center(i);
    g->size[i] = size(i) / 2.0;
  }
  g->quat[0] = std::cos(yaw / 2.0);
  g->quat[1] = 0.0;
  g->quat[2] = 0.0;
  g->quat[3] = std::sin(yaw / 2.0);
  g->contype = 1;
  g->conaffinity = 2;
}

// The site geom `name` equals the box (centre, half size, yaw quaternion up to sign) within 1e-6.
bool same_box(const mjsGeom * g, const SceneBox & box)
{
  for (int i = 0; i < 3; ++i)
    if (std::abs(g->pos[i] - box.center(i)) > 1e-6 || std::abs(g->size[i] - box.size(i) / 2.0) > 1e-6) return false;
  const double dot = g->quat[0] * std::cos(box.yaw / 2.0) + g->quat[3] * std::sin(box.yaw / 2.0);
  const double norm = std::sqrt(g->quat[0] * g->quat[0] + g->quat[1] * g->quat[1] + g->quat[2] * g->quat[2] + g->quat[3] * g->quat[3]);
  return std::abs(std::abs(dot) / norm - 1.0) < 1e-9;
}
int id_of(const mjModel * m, mjtObj type, const std::string & name)
{
  const int id = mj_name2id(m, type, name.c_str());
  if (id < 0) throw std::runtime_error("build_model: the robot model has no " + name);
  return id;
}

// The profile is the one source of the gains and passive joint terms: robot.xml's values are overwritten, so a
// calibrated profile takes effect without regenerating the model.
void apply_profile(mjModel * m, const RobotProfile & p)
{
  for (int i = 0; i < 6; ++i) {
    const int a = id_of(m, mjOBJ_ACTUATOR, "servo/" + p.arm_joints[i]);
    m->actuator_gainprm[a * mjNGAIN] = p.kp(i);
    m->actuator_biasprm[a * mjNBIAS + 1] = -p.kp(i);
    m->actuator_biasprm[a * mjNBIAS + 2] = -p.kd(i);
    const int dof = m->jnt_dofadr[id_of(m, mjOBJ_JOINT, p.arm_joints[i])];
    m->dof_armature[dof] = p.armature(i);
    m->dof_damping[dof] = p.damping(i);
    m->dof_frictionloss[dof] = p.frictionloss(i);
  }
  const char * drives[3] = {"drive/base_x", "drive/base_y", "drive/base_yaw"};
  for (int i = 0; i < 3; ++i) {
    const int a = id_of(m, mjOBJ_ACTUATOR, drives[i]);
    m->actuator_gainprm[a * mjNGAIN] = p.base_servo_kv(i);
    m->actuator_biasprm[a * mjNBIAS + 2] = -p.base_servo_kv(i);
    // the wheels' traction: x and y each up to the drive force (a box around the friction circle), yaw the torque
    const double limit = i < 2 ? p.drive_force_limit_n : p.drive_torque_limit_nm;
    m->actuator_forcelimited[a] = 1;
    m->actuator_forcerange[2 * a] = -limit;
    m->actuator_forcerange[2 * a + 1] = limit;
  }
  // the armature enters the constant terms computed at qpos0 (inverse weights of the constraint solver)
  mjData * d = mj_makeData(m);
  mj_setConst(m, d);
  mj_deleteData(d);
}
}  // namespace

mjModel * build_model(const RobotProfile & profile, const std::string & site_mjcf, const ScenarioSetup & scenario)
{
  SpecPtr site = parse(site_mjcf);
  SpecPtr robot = parse(profile.model_path);
  mjsBody * world = mjs_findBody(site.get(), "world");
  mjsBody * base = mjs_findBody(robot.get(), "base_footprint");
  if (!world || !base) throw std::runtime_error("build_model: no world body in the site or no base_footprint in the robot");
  mjsFrame * frame = mjs_addFrame(world, nullptr);  // a body attaches under a frame
  // the robot keeps its own names (they are unique beside the site's): MuJoCo warns about the empty prefix
  const QuietWarnings quiet({"empty prefix"});
  if (!mjs_attach(frame->element, base->element, "", ""))
    throw std::runtime_error(std::string("build_model: attaching the robot failed: ") + mjs_getError(site.get()));

  for (const SceneBox & box : scenario.boxes) {
    if (mjsElement * existing = mjs_findElement(site.get(), mjOBJ_GEOM, ("site/" + box.name).c_str())) {
      if (!same_box(mjs_asGeom(existing), box))
        throw std::runtime_error("build_model: scenario " + scenario.name + " box " + box.name + " differs from the site box");
      continue;
    }
    mjsGeom * g = mjs_addGeom(world, nullptr);
    mjs_setName(g->element, ("scenario/" + box.name).c_str());
    set_box(g, box.center, box.size, box.yaw);
    g->group = 3;
    g->rgba[0] = 0.75f, g->rgba[1] = 0.55f, g->rgba[2] = 0.3f, g->rgba[3] = 1.0f;
  }
  for (const MoverSpec & mover : scenario.movers) {
    mjsBody * b = mjs_addBody(world, nullptr);
    mjs_setName(b->element, ("mover/" + mover.name).c_str());
    b->mocap = 1;
    b->pos[0] = mover.start.x();
    b->pos[1] = mover.start.y();
    b->pos[2] = mover.size.z() / 2.0;
    mjsGeom * g = mjs_addGeom(b, nullptr);
    mjs_setName(g->element, ("mover/" + mover.name).c_str());
    set_box(g, Eigen::Vector3d::Zero(), mover.size, 0.0);
    g->group = 4;
    g->rgba[0] = 0.9f, g->rgba[1] = 0.8f, g->rgba[2] = 0.1f, g->rgba[3] = 1.0f;
  }

  site->option.timestep = kPhysicsStepS;
  site->option.integrator = mjINT_IMPLICITFAST;
  std::unique_ptr<mjModel, void (*)(mjModel *)> m(mj_compile(site.get(), nullptr), mj_deleteModel);
  if (!m) throw std::runtime_error(std::string("build_model: ") + mjs_getError(site.get()));
  apply_profile(m.get(), profile);
  return m.release();
}
}  // namespace mobile_manipulator_plant
