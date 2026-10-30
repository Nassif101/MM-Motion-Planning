// medium_bench (Phase 5A plan Task 9, spec Section 6.1): the medium planner open-loop on recorded states.
//
//   medium_bench open --bag DIR --scenario NAME --out FILE.json [options]
//   medium_bench synthetic --scenario NAME --out FILE.json [options]
//
// open: a B5 recording (its bag directory); every --spacing-s of the recorded cycles one snapshot (the recorded state,
// its odometry and streamed arm velocity, the FollowPath route then active). Each snapshot is planned twice: cold (a
// fresh planner, full multi-start) and warm (one planner carried along the snapshots, as the node would, with its
// periodic full multi-starts). synthetic: the scenario's start-to-goal straight line, snapshots every --spacing-m on
// it at the start pose's arm (the cells without a recording). Options: --backend hpipm|osqp|daqp, --tolerance X,
// --segments N, --horizon-s T, --collision-dt D, --torque-dt D, --joints 4|6, --config FILE (medium.yaml), --cap-s S
// (default 10: the bench measures uncapped work), --oracle (open: also an "oracle" record per snapshot, the SCP seeded
// with the motion B5 then executed, fitted to the recorded states over the horizon: Task 9 CP, whether the infeasible
// plans are the seeds' or the formulation's). One JSON object: the options and one record per planned cycle.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_medium/planner.hpp"
#include "mobile_manipulator_mpc/replay_bag.hpp"
#include "mobile_manipulator_mpc/scene.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace
{
namespace mmd = mobile_manipulator_medium;
namespace mmm = mobile_manipulator_mpc;
namespace mmn = mobile_manipulator_navigation;
using Json = mmn::Json;

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

struct Options
{
  std::string mode, bag, scenario, out, config, backend;
  double tolerance = -1, horizon_s = -1, collision_dt = -1, torque_dt = -1, cap_s = 10.0, spacing_s = 1.0, spacing_m = 0.5;
  int segments = -1, joints = -1;
  bool oracle = false;
};

Options parse(int argc, char ** argv)
{
  if (argc < 2) throw std::invalid_argument("usage: medium_bench open|synthetic --scenario NAME --out FILE [options]");
  Options o;
  o.mode = argv[1];
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() {
      if (i + 1 >= argc) throw std::invalid_argument(a + " needs a value");
      return std::string(argv[++i]);
    };
    if (a == "--bag") o.bag = next();
    else if (a == "--scenario") o.scenario = next();
    else if (a == "--out") o.out = next();
    else if (a == "--config") o.config = next();
    else if (a == "--backend") o.backend = next();
    else if (a == "--tolerance") o.tolerance = std::stod(next());
    else if (a == "--segments") o.segments = std::stoi(next());
    else if (a == "--horizon-s") o.horizon_s = std::stod(next());
    else if (a == "--collision-dt") o.collision_dt = std::stod(next());
    else if (a == "--torque-dt") o.torque_dt = std::stod(next());
    else if (a == "--joints") o.joints = std::stoi(next());
    else if (a == "--cap-s") o.cap_s = std::stod(next());
    else if (a == "--oracle") o.oracle = true;
    else if (a == "--spacing-s") o.spacing_s = std::stod(next());
    else if (a == "--spacing-m") o.spacing_m = std::stod(next());
    else throw std::invalid_argument("unknown option " + a);
  }
  if ((o.mode != "open" && o.mode != "synthetic") || o.scenario.empty() || o.out.empty() || (o.mode == "open" && o.bag.empty()))
    throw std::invalid_argument("medium_bench open needs --bag, both modes --scenario and --out");
  return o;
}

mmd::MediumConfig config_of(const Options & o)
{
  mmd::MediumConfig c = o.config.empty() ? mmd::load_medium_config(share("mobile_manipulator_medium") + "/config/medium.yaml")
                                         : mmd::load_medium_config(o.config);
  if (!o.backend.empty()) c.qp_backend = o.backend;
  if (o.tolerance > 0) c.qp_tolerance = o.tolerance;
  if (o.segments > 0) c.bezier.segments = o.segments;
  if (o.horizon_s > 0) c.bezier.horizon_s = o.horizon_s;
  if (o.collision_dt > 0) c.collision_dt = o.collision_dt;
  if (o.torque_dt > 0) c.torque_dt = o.torque_dt;
  if (o.joints == 6)
    c.arm_joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  else if (o.joints != -1 && o.joints != 4)
    throw std::invalid_argument("--joints must be 4 or 6");
  c.solve_cap_s = o.cap_s;
  return c;
}

class Bench
{
public:
  explicit Bench(const Options & o) : config_(config_of(o))
  {
    urdf_ = read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf");
    payload_ = Json::parse(read_file(share("mobile_manipulator_control") + "/config/qualified_payload.json"));
    primitives_ = mobile_manipulator_geometry::load_primitives(urdf_, mobile_manipulator_geometry::payload_from_json(payload_));
    limits_ = mmm::load_limits(share("mobile_manipulator_navigation") + "/config/nav_operating_envelope.yaml", urdf_);
    statics_ = std::make_shared<const mmm::GeneratedStaticTorque>();
    for (const auto & item : payload_.at("poses_rad").items()) {
      mmd::Joints q;
      for (int i = 0; i < 6; ++i) q(i) = item.value().at(i).get<double>();
      poses_[item.key()] = q;
    }
    scene_ = mmm::load_scene(share("mobile_manipulator_navigation"), "", o.scenario, "");
    world_.set_static(scene_.boxes);
  }

  mmd::MediumPlanner planner() const
  {
    mmd::MediumProblem problem(config_, limits_, primitives_, mmm::CollisionConfig{}, statics_, poses_.at("carry"));
    return mmd::MediumPlanner(std::move(problem), mmd::make_dense_solver(config_.qp_backend, mmd::DenseQpOptions{config_.qp_tolerance}),
                              poses_, primitives_.back().dims);
  }

  const mmd::MediumConfig & config() const { return config_; }
  const mmm::World & world() const { return world_; }
  const mmm::Scene & scene() const { return scene_; }
  const std::map<std::string, mmd::Joints> & poses() const { return poses_; }

private:
  mmd::MediumConfig config_;
  std::string urdf_;
  Json payload_;
  std::vector<mobile_manipulator_geometry::Primitive> primitives_;
  mmm::Limits limits_;
  std::shared_ptr<const mmm::StaticTorque> statics_;
  std::map<std::string, mmd::Joints> poses_;
  mmm::Scene scene_;
  mmm::World world_;
};

Json record(const std::string & kind, double t, const mmd::PlanOutput & out, const mmd::MediumPlanner & p,
            const mmm::World & world)
{
  Json r;
  r["kind"] = kind;
  r["t"] = t;
  r["total_s"] = out.solve_s;
  r["linearize_s"] = out.linearize_s;
  r["qp_s"] = out.qp_s;
  r["merit_s"] = out.merit_s;
  r["collision_s"] = out.linearize_parts.collision_s;
  r["torque_s"] = out.linearize_parts.torque_s;
  r["cost_s"] = out.linearize_parts.cost_s;
  r["assemble_s"] = out.linearize_parts.assemble_s;
  int iterations = 0, converged = 0, feasible = 0;
  for (const auto & s : out.seeds) {
    iterations += s.iterations;
    converged += s.converged;
    feasible += s.feasible;
  }
  r["iterations"] = iterations;
  r["seeds"] = static_cast<int>(out.seeds.size());
  r["converged_seeds"] = converged;
  r["feasible_seeds"] = feasible;
  r["full_multistart"] = out.full_multistart;
  r["capped"] = out.capped;
  r["status"] = out.status;
  r["seed"] = out.seed;
  r["homotopy"] = out.homotopy;
  r["homotopy_switched"] = out.homotopy_switched;
  r["plan_length_m"] = out.samples.empty() ? 0.0 : out.samples.back().s;  // the plan's path length over the horizon
  if (const mmd::Trajectory * t_last = p.last_trajectory()) {  // the published plan's violations by kind
    const mmd::Violations v = p.problem().violations(*t_last, world, t);
    r["plan"] = {{"collision_m", v.collision_m}, {"torque", v.torque}, {"speed_mps", v.speed_mps}, {"hull", v.hull}};
  }
  return r;
}

// The violations of standing at `x` (the start line at rest): what no plan from this state can avoid at its first samples.
Json start_violations(const mmd::MediumPlanner & p, const mmm::State & x, const mmm::World & world)
{
  const auto & problem = p.problem();
  const mmd::Trajectory still = problem.from_parameters(x, Eigen::Matrix<double, 9, 1>::Zero(),
                                                        Eigen::MatrixXd::Zero(problem.coordinates(), problem.basis().free_parameters()));
  const mmd::Violations v = problem.violations(still, world, 0.0);
  return {{"collision_m", v.collision_m}, {"torque", v.torque}};
}

// The measured forward speed's violation of 0 <= v <= v_max (the plan's first sample carries it).
double start_speed_violation(const mmd::MediumPlanner & p, const mmm::State & x, const Eigen::Matrix<double, 9, 1> & rate)
{
  const double v = rate(0) * std::cos(x(2)) + rate(1) * std::sin(x(2));
  return std::max({0.0, -v, v - p.problem().limits().v_max});
}

Json violations_json(const mmd::Violations & v)
{
  return {{"collision_m", v.collision_m}, {"torque", v.torque}, {"speed_mps", v.speed_mps}, {"hull", v.hull}};
}

Eigen::Matrix<double, 9, 1> rate_of(const mmm::RecordedCycle & c);

// The SCP from the recorded motion of cycles[i..] over the horizon (none when the recording ends before it): the fit's
// largest error (base position m, planned joint rad), and per variant of the seed the seed's violations and merit and
// the SCP's result. Variants keep one property of the recorded motion and take the rest from a pose seed's
// construction (the route at the reference speed, the arm held): full, base (recorded base, arm held), arm (route
// base, recorded arm), speed (the route at the recorded progress along it, arm held), path (the recorded path at the
// reference speed, arm held).
Json oracle(const mmm::Recording & rec, size_t i, const mmd::MediumPlanner & cold, const mmm::Route & route,
            const mmn::Pose2 & goal, const Bench & bench)
{
  const auto & problem = cold.problem();
  const mmm::RecordedCycle & c = rec.cycles[i];
  const double horizon = problem.basis().horizon_s();
  size_t last = i;
  while (last + 1 < rec.cycles.size() && rec.cycles[last + 1].stamp - c.stamp <= horizon + 1e-9) ++last;
  if (rec.cycles[last].stamp - c.stamp < horizon - 0.1) return nullptr;
  auto recorded = [&](double t) {  // the recorded state nearest now + t
    size_t j = i;
    while (j < last && rec.cycles[j + 1].stamp - c.stamp <= t) ++j;
    if (j < last && (rec.cycles[j + 1].stamp - c.stamp - t) < (t - (rec.cycles[j].stamp - c.stamp))) ++j;
    return rec.cycles[j].x;
  };
  const int K = problem.samples();
  std::vector<double> times(K);
  for (int k = 0; k < K; ++k) times[k] = problem.sample_time(k);
  const Eigen::Matrix<double, 9, 1> rate = rate_of(c);
  mmd::RouteTask task;
  task.route = &route;
  task.goal_s = route.length();
  task.goal_yaw = goal[2];
  task.s0 = route.project_range(c.x.head<2>(), 0.0, route.length()).s;
  const double v = problem.config().speed_fraction * problem.limits().v_max;
  auto smoothstep = [](double u) { u = std::clamp(u, 0.0, 1.0); return u * u * (3.0 - 2.0 * u); };
  // the pose seed's base at route arc length s and time t
  auto route_base = [&](mmm::State & x, double s, double t) {
    const Eigen::Vector2d p = route.at(s);
    const double blend = smoothstep(t / 1.0);  // the planner's kSeedYawBlendS
    x(0) = (1.0 - blend) * c.x(0) + blend * p.x();
    x(1) = (1.0 - blend) * c.x(1) + blend * p.y();
    x(2) = c.x(2) + blend * std::remainder(route.heading_at(s) - c.x(2), 2.0 * M_PI);
  };
  // the recorded path's arc length, for the path variant
  std::vector<double> arc{0.0};
  for (size_t j = i + 1; j <= last; ++j) arc.push_back(arc.back() + (rec.cycles[j].x.head<2>() - rec.cycles[j - 1].x.head<2>()).norm());
  const std::vector<std::string> variants = {"full", "base", "arm", "speed", "path"};
  Json out = {{"t", c.stamp}, {"recorded_arc_m", arc.back()}, {"reference_arc_m", v * horizon}};
  double s_progress = task.s0;
  std::map<std::string, std::vector<mmm::State>> states;
  for (int k = 0; k < K; ++k) {
    const double t = times[k];
    const mmm::State r = recorded(t);
    mmm::State held = r;
    held.tail<6>() = c.x.tail<6>();
    states["full"].push_back(r);
    states["base"].push_back(held);
    mmm::State arm = r;
    route_base(arm, std::min(task.goal_s, task.s0 + v * t), t);
    states["arm"].push_back(arm);
    s_progress = std::max(s_progress, route.project_range(r.head<2>(), s_progress, std::min(route.length(), s_progress + 1.0)).s);
    mmm::State speed = held;
    route_base(speed, s_progress, t);
    states["speed"].push_back(speed);
    const double want = v * t;
    size_t a = 0;
    while (a + 1 < arc.size() && arc[a + 1] <= want) ++a;
    mmm::State path = rec.cycles[i + a].x;
    path.tail<6>() = c.x.tail<6>();
    states["path"].push_back(path);
  }
  for (const auto & name : variants) {
    const mmd::Trajectory seed = problem.fit(c.x, rate, times, states[name]);
    if (name == "full") {
      double base_err = 0.0, joint_err = 0.0;
      for (int k = 0; k < K; ++k) {
        const mmm::State x = problem.state_at(seed, k);
        base_err = std::max(base_err, (x.head<2>() - states[name][k].head<2>()).norm());
        joint_err = std::max(joint_err, (x.tail<6>() - states[name][k].tail<6>()).cwiseAbs().maxCoeff());
      }
      out["fit_base_m"] = base_err;
      out["fit_joint_rad"] = joint_err;
    }
    mmd::Violations at_seed;
    const double seed_merit = problem.merit(seed, task, bench.world(), c.stamp, &at_seed);
    const auto solver = mmd::make_dense_solver(problem.config().qp_backend, mmd::DenseQpOptions{problem.config().qp_tolerance});
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                               std::chrono::duration<double>(problem.config().solve_cap_s));
    const mmd::ScpResult r = mmd::solve_scp(problem, *solver, seed, task, bench.world(), c.stamp, deadline);
    out[name] = {{"seed_merit", seed_merit}, {"seed", violations_json(at_seed)},
                 {"seed_feasible", mmd::feasible(at_seed, problem.config())}, {"merit", r.merit},
                 {"plan", violations_json(r.violations)}, {"feasible", r.feasible}, {"converged", r.converged},
                 {"iterations", r.iterations}, {"solve_s", r.solve_s}};
  }
  return out;
}

Eigen::Matrix<double, 9, 1> rate_of(const mmm::RecordedCycle & c)
{
  Eigen::Matrix<double, 9, 1> r = Eigen::Matrix<double, 9, 1>::Zero();
  r(0) = c.odom_v * std::cos(c.x(2));
  r(1) = c.odom_v * std::sin(c.x(2));
  r(2) = c.odom_omega;
  r.tail<6>() = c.arm_v;
  return r;
}

Json run_open(const Options & o, Bench & bench)
{
  std::array<std::string, 6> joints;
  for (int i = 0; i < 6; ++i) joints[i] = mmm::kArmJoints[i];
  const mmm::Recording rec = mmm::read_recording(o.bag, joints);
  Json records = Json::array();
  mmd::MediumPlanner warm = bench.planner();
  size_t goal = 0;
  int route_id = 0;
  double next = -1e300;
  for (size_t i = 0; i < rec.cycles.size(); ++i) {
    const auto & c = rec.cycles[i];
    if (c.stamp < next) continue;
    next = c.stamp + o.spacing_s;
    // the FollowPath goal active at this cycle
    while (goal + 1 < rec.goals.size() && rec.goals[goal + 1].bag_time <= c.bag_time) ++goal;
    if (rec.goals.empty() || rec.goals[goal].bag_time > c.bag_time || rec.goals[goal].points.size() < 2) continue;
    const mmm::Route route(rec.goals[goal].points);
    if (static_cast<int>(goal) + 1 != route_id) {
      route_id = static_cast<int>(goal) + 1;
      warm.set_route(route, rec.goals[goal].goal, static_cast<uint64_t>(route_id));
    }
    mmd::MediumPlanner cold = bench.planner();
    cold.set_route(route, rec.goals[goal].goal, 1);
    records.push_back(record("cold", c.stamp, cold.plan(c.stamp, c.x, rate_of(c), bench.world()), cold, bench.world()));
    records.back()["start"] = start_violations(cold, c.x, bench.world());
    records.back()["start"]["speed_mps"] = start_speed_violation(cold, c.x, rate_of(c));
    records.back()["remaining_m"] = route.length() - route.project_range(c.x.head<2>(), 0.0, route.length()).s;
    if (o.oracle) {
      // the same route over the whole horizon only
      size_t last = i;
      while (last + 1 < rec.cycles.size() && rec.cycles[last + 1].stamp - c.stamp <= cold.problem().basis().horizon_s()) ++last;
      const bool same_route = goal + 1 >= rec.goals.size() || rec.goals[goal + 1].bag_time > rec.cycles[last].bag_time;
      if (same_route) records.back()["oracle"] = oracle(rec, i, cold, route, rec.goals[goal].goal, bench);
    }
    records.push_back(record("warm", c.stamp, warm.plan(c.stamp, c.x, rate_of(c), bench.world()), warm, bench.world()));
  }
  return records;
}

Json run_synthetic(const Options & o, Bench & bench)
{
  const auto & spec = *bench.scene().scenario;
  const auto start = spec.at("start"), goal = spec.at("goal");
  const Eigen::Vector2d a(start.at(0).get<double>(), start.at(1).get<double>()), b(goal.at(0).get<double>(), goal.at(1).get<double>());
  const mmm::Route route({a, b});
  const std::string arm = spec.value("arm_pose", std::string("carry"));
  const mmd::Joints q = bench.poses().at(arm);
  const double yaw = std::atan2(b.y() - a.y(), b.x() - a.x());
  Json records = Json::array();
  mmd::MediumPlanner warm = bench.planner();
  warm.set_route(route, {b.x(), b.y(), goal.at(2).get<double>()}, 1);
  double t = 0.0;
  for (double s = 0.0; s < route.length() - 0.5; s += o.spacing_m, t += o.spacing_m / 0.24) {
    const Eigen::Vector2d p = route.at(s);
    mmm::State x;
    x << p.x(), p.y(), yaw, q;
    const Eigen::Matrix<double, 9, 1> rate = Eigen::Matrix<double, 9, 1>::Zero();
    mmd::MediumPlanner cold = bench.planner();
    cold.set_route(route, {b.x(), b.y(), goal.at(2).get<double>()}, 1);
    records.push_back(record("cold", t, cold.plan(t, x, rate, bench.world()), cold, bench.world()));
    records.back()["start"] = start_violations(cold, x, bench.world());
    records.push_back(record("warm", t, warm.plan(t, x, rate, bench.world()), warm, bench.world()));
  }
  return records;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options o = parse(argc, argv);
    Bench bench(o);
    Json out;
    const auto & c = bench.config();
    out["options"] = {{"mode", o.mode}, {"bag", o.bag}, {"scenario", o.scenario}, {"backend", c.qp_backend},
                      {"tolerance", c.qp_tolerance}, {"segments", c.bezier.segments}, {"horizon_s", c.bezier.horizon_s},
                      {"collision_dt", c.collision_dt}, {"torque_dt", c.torque_dt},
                      {"joints", static_cast<int>(c.arm_joints.size())}, {"cap_s", c.solve_cap_s}};
    out["records"] = o.mode == "open" ? run_open(o, bench) : run_synthetic(o, bench);
    std::ofstream(o.out) << out.dump(1) << "\n";
    std::cout << out["records"].size() << " records -> " << o.out << "\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "medium_bench: " << e.what() << "\n";
    return 1;
  }
}
