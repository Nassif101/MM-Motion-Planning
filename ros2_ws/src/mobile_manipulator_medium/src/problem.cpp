#include "mobile_manipulator_medium/problem.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mobile_manipulator_medium
{
namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kRegularization = 1e-6;  // on the parameter block of H: strictly positive definite (DAQP)
constexpr double kFixedRowGradient = 1e-12;
constexpr double kHullTolerance = 1e-6;   // the QP backends' accuracy on an active bound

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// A row of the cost or of a constraint over the QP variables.
using Row = Eigen::RowVectorXd;

// 0.5 w (r0 + J dp)^2 into H and g.
void add_residual(Eigen::MatrixXd & H, Eigen::VectorXd & g, double w, double r0, const Row & J)
{
  H.noalias() += w * J.transpose() * J;
  g.noalias() += w * r0 * J.transpose();
}

bool is_obstacle_row(const mmm::DistanceRow & row)
{
  return row.obstacle != "floor" && row.obstacle.rfind("self/", 0) != 0;
}

// The route's unit tangent at s: the chord over +-kTangentChordM, as B5's (it turns continuously through corners).
Eigen::Vector2d tangent(const mmm::Route & route, double s)
{
  const Eigen::Vector2d chord = route.at(s + mmm::kTangentChordM) - route.at(s - mmm::kTangentChordM);
  if (chord.norm() > 1e-9) return chord.normalized();
  const double h = route.heading_at(s);
  return {std::cos(h), std::sin(h)};
}

// The goal heading's weight factor: 0 farther than kGoalHeadingM from the goal, rising linearly to 1 at it.
double goal_closeness(double goal_s, double s) { return std::clamp(1.0 - (goal_s - s) / mmm::kGoalHeadingM, 0.0, 1.0); }
}  // namespace

struct MediumProblem::Evaluation
{
  std::map<std::string, double> terms{{"progress", 0.0}, {"contour", 0.0}, {"goal_heading", 0.0}, {"clearance", 0.0},
                                      {"comfort", 0.0}, {"smooth", 0.0}, {"skid", 0.0}};
  Violations violations;
};

std::vector<int> joint_indices(const std::vector<std::string> & arm_joints)
{
  std::vector<int> out;
  for (const auto & name : arm_joints) {
    const auto it = std::find(mmm::kArmJoints.begin(), mmm::kArmJoints.end(), name);
    if (it == mmm::kArmJoints.end()) throw std::invalid_argument("MediumProblem: unknown arm joint " + name);
    const int j = static_cast<int>(it - mmm::kArmJoints.begin());
    if (std::find(out.begin(), out.end(), j) != out.end())
      throw std::invalid_argument("MediumProblem: arm joint " + name + " listed twice");
    out.push_back(j);
  }
  return out;
}

MediumProblem::MediumProblem(MediumConfig config, mmm::Limits limits,
                             const std::vector<mobile_manipulator_geometry::Primitive> & primitives,
                             mmm::CollisionConfig collision, std::shared_ptr<const mmm::StaticTorqueModel> statics,
                             Joints q_comfort)
: config_(std::move(config)),
  limits_(std::move(limits)),
  collision_([&] {
    collision.margin_m = config_.arm_margin_m;
    collision.base_margin_m = config_.base_margin_m;
    if (collision.activation_m < config_.clearance_target_m)
      throw std::invalid_argument("MediumProblem: the collision activation distance is below clearance_target_m");
    return mmm::CollisionModel(primitives, collision);
  }()),
  statics_(std::move(statics)),
  q_comfort_(q_comfort),
  basis_(std::make_unique<PiecewiseBezier>(config_.bezier)),
  joints_(joint_indices(config_.arm_joints))
{
  if (!(config_.collision_dt > 0.0) || !(config_.torque_dt > 0.0))
    throw std::invalid_argument("MediumProblem: sample times must be positive");
  if (!statics_) throw std::invalid_argument("MediumProblem: no static torque model");
  const double T = basis_->horizon_s();
  const int K = static_cast<int>(std::round(T / config_.collision_dt));
  for (int k = 0; k <= K; ++k) times_.push_back(std::min(T, k * config_.collision_dt));
  const int n = basis_->control_points();
  for (int r = 0; r <= 2; ++r) {
    Eigen::MatrixXd S(samples(), n);
    for (int k = 0; k < samples(); ++k) S.row(k) = basis_->basis(times_[k], r);
    s_.push_back(S);
    sm_.push_back(S * basis_->map());
    const Eigen::MatrixXd D = basis_->derivative_control_points(r);
    const auto rows = basis_->hull_rows(r);
    Eigen::MatrixXd hull(rows.size(), n);
    for (size_t i = 0; i < rows.size(); ++i) hull.row(i) = D.row(rows[i]);
    hull_.push_back(hull);
  }
  gram_ = basis_->map().transpose() * basis_->acceleration_gram() * basis_->map();
  const int stride = std::max(1, static_cast<int>(std::round(config_.torque_dt / config_.collision_dt)));
  for (int k = stride; k < samples(); k += stride) torque_samples_.push_back(k);
  if (torque_samples_.empty() || torque_samples_.back() != samples() - 1) torque_samples_.push_back(samples() - 1);
}

Trajectory MediumProblem::from_parameters(const mmm::State & start, const Eigen::Matrix<double, 9, 1> & start_rate,
                                          const Eigen::MatrixXd & params) const
{
  Trajectory t;
  t.q_rest = start.tail<6>();
  t.control_points.resize(coordinates(), basis_->control_points());
  for (int c = 0; c < coordinates(); ++c) {
    const int i = c < 3 ? c : 3 + joints_[c - 3];
    t.control_points.row(c) = (basis_->map() * params.row(c).transpose() + basis_->linear(start(i), start_rate(i))).transpose();
  }
  return t;
}

Eigen::MatrixXd MediumProblem::parameters(const Trajectory & t) const
{
  // The deviation from the trajectory's own start line lies in the map's range (orthonormal columns): map^T recovers it.
  Eigen::MatrixXd out(coordinates(), basis_->free_parameters());
  const Eigen::RowVectorXd b0 = basis_->basis(0.0, 0), b1 = basis_->basis(0.0, 1);
  for (int c = 0; c < coordinates(); ++c) {
    const Eigen::VectorXd cp = t.control_points.row(c).transpose();
    out.row(c) = (basis_->map().transpose() * (cp - basis_->linear(b0.dot(cp), b1.dot(cp)))).transpose();
  }
  return out;
}

mmm::State MediumProblem::state_at(const Trajectory & t, int sample) const
{
  mmm::State x;
  x.tail<6>() = t.q_rest;
  const Eigen::VectorXd values = t.control_points * s_[0].row(sample).transpose();
  x.head<3>() = values.head<3>();
  for (size_t m = 0; m < joints_.size(); ++m) x(3 + joints_[m]) = values(3 + static_cast<int>(m));
  return x;
}

Eigen::Matrix<double, 9, 1> MediumProblem::rate_at(const Trajectory & t, int sample) const
{
  Eigen::Matrix<double, 9, 1> r = Eigen::Matrix<double, 9, 1>::Zero();
  const Eigen::VectorXd values = t.control_points * s_[1].row(sample).transpose();
  r.head<3>() = values.head<3>();
  for (size_t m = 0; m < joints_.size(); ++m) r(3 + joints_[m]) = values(3 + static_cast<int>(m));
  return r;
}

Trajectory MediumProblem::apply(const Trajectory & t, const Eigen::VectorXd & dp) const
{
  const int nf = basis_->free_parameters();
  if (dp.size() < coordinates() * nf) throw std::invalid_argument("MediumProblem::apply: step too short");
  Trajectory out = t;
  for (int c = 0; c < coordinates(); ++c)
    out.control_points.row(c) += (basis_->map() * dp.segment(c * nf, nf)).transpose();
  return out;
}

double MediumProblem::target_speed(double s, double goal_s) const
{
  if (s >= goal_s) return 0.0;
  const double v_nom = config_.speed_fraction * limits_.v_max;
  return std::max(limits_.breakaway.linear_mps, v_nom * std::clamp((goal_s - s) / mmm::kProgressTaperM, 0.0, 1.0));
}

MediumQp MediumProblem::linearize(const Trajectory & nominal, const RouteTask & task, const mmm::World & world,
                                  double now, double trust) const
{
  if (task.route == nullptr) throw std::invalid_argument("MediumProblem::linearize: no route");
  const int nf = basis_->free_parameters(), C = coordinates(), np = C * nf, n = np + 2, K = samples();
  const int soft = np, hull_slack = np + 1;
  const auto & w = config_.weights;
  const double dt = config_.collision_dt;
  MediumQp out;
  out.parameters = np;
  DenseQp & qp = out.qp;
  qp.H = Eigen::MatrixXd::Zero(n, n);
  qp.g = Eigen::VectorXd::Zero(n);

  std::vector<mmm::State> X(K);
  std::vector<Eigen::Matrix<double, 9, 1>> V(K);
  for (int k = 0; k < K; ++k) {
    X[k] = state_at(nominal, k);
    V[k] = rate_at(nominal, k);
  }
  // The QP row of coordinate c's r-th derivative at sample k.
  auto J = [&](int c, int r, int k) {
    Row row = Row::Zero(n);
    row.segment(c * nf, nf) = sm_[r].row(k);
    return row;
  };
  // The QP row of d(value)/d(state) . grad at sample k (state index -> coordinate; joints not planned are fixed).
  auto through_state = [&](const Eigen::Matrix<double, 1, mmm::kNx> & grad, int k) {
    Row row = Row::Zero(n);
    for (int i = 0; i < 3; ++i)
      if (grad(i) != 0.0) row.segment(i * nf, nf) += grad(i) * sm_[0].row(k);
    for (size_t m = 0; m < joints_.size(); ++m) {
      const double gi = grad(3 + joints_[m]);
      if (gi != 0.0) row.segment((3 + static_cast<int>(m)) * nf, nf) += gi * sm_[0].row(k);
    }
    return row;
  };

  // ---- cost ----
  const double window = std::max(1.0, 3.0 * limits_.v_max * dt);
  double s_hint = task.s0;
  for (int k = 1; k < K; ++k) {
    const Eigen::Vector2d p = X[k].head<2>();
    const auto proj = task.route->project(p, s_hint, window);
    s_hint = proj.s;
    const Eigen::Vector2d tan = tangent(*task.route, proj.s);
    const Eigen::Vector2d nrm(-tan.y(), tan.x());
    // progress: the speed along the route towards the reference speed (a bounded pull)
    add_residual(qp.H, qp.g, w.progress * dt, tan.dot(V[k].head<2>()) - target_speed(proj.s, task.goal_s),
                 tan.x() * J(0, 1, k) + tan.y() * J(1, 1, k));
    // contour: normal offset at the projection
    add_residual(qp.H, qp.g, w.contour * dt, nrm.dot(p - proj.point), nrm.x() * J(0, 0, k) + nrm.y() * J(1, 0, k));
    // comfort
    for (size_t m = 0; m < joints_.size(); ++m)
      add_residual(qp.H, qp.g, w.comfort * dt, X[k](3 + joints_[m]) - q_comfort_(joints_[m]),
                   J(3 + static_cast<int>(m), 0, k));
    // skid: body-frame lateral velocity
    const double c = std::cos(X[k](2)), s = std::sin(X[k](2));
    add_residual(qp.H, qp.g, w.skid * dt, -s * V[k](0) + c * V[k](1),
                 -s * J(0, 1, k) + c * J(1, 1, k) + (-c * V[k](0) - s * V[k](1)) * J(2, 0, k));
  }
  // goal heading, ramped in over the last kGoalHeadingM before the goal
  {
    const auto proj = task.route->project(X[K - 1].head<2>(), s_hint, window);
    const double closeness = goal_closeness(task.goal_s, proj.s);
    if (closeness > 0.0)
      add_residual(qp.H, qp.g, w.goal_heading * closeness, wrap(X[K - 1](2) - task.goal_yaw), J(2, 0, K - 1));
  }
  // smoothness: 0.5 w c^T G c per coordinate, in the parameters
  for (int c = 0; c < C; ++c) {
    const double wc = c < 3 ? w.smooth_base : w.smooth_arm;
    qp.H.block(c * nf, c * nf, nf, nf) += wc * gram_;
    qp.g.segment(c * nf, nf) += wc * basis_->map().transpose() * basis_->acceleration_gram() *
                                nominal.control_points.row(c).transpose();
  }

  // ---- rows ----
  std::vector<Row> rows;
  std::vector<double> lo, hi;
  auto push = [&](const Row & row, double l, double u, RowInfo info) {
    rows.push_back(row);
    lo.push_back(l);
    hi.push_back(u);
    out.rows.push_back(info);
  };
  // collision (soft, metres) and the clearance hinge
  mmm::CollisionStats stats;
  for (const auto & row : collision_.rows(X, dt, now, world, &stats)) {
    if (row.stage <= 0 || row.stage >= K) continue;
    Row grad = through_state(row.gradient, row.stage);
    if (is_obstacle_row(row) && row.distance < config_.clearance_target_m)
      add_residual(qp.H, qp.g, w.clearance * dt, config_.clearance_target_m - row.distance, -grad);
    grad(soft) = 1.0;  // margin - d <= J dp + s
    push(grad, row.margin - row.distance, kInf, RowInfo{RowKind::Collision, row.stage, row, -1});
  }
  // static torque (soft, as a fraction of the limit): |tau_j| / L_j <= 1
  for (const int k : torque_samples_) {
    const Joints q = X[k].tail<6>();
    const Joints tau = statics_->torque(q);
    const Eigen::Matrix<double, 6, 6> Jt = statics_->jacobian(q);
    for (int j = 0; j < 6; ++j) {
      const double L = config_.torque_limits_nm(j);
      if (!std::isfinite(L)) continue;
      Eigen::Matrix<double, 1, mmm::kNx> grad = Eigen::Matrix<double, 1, mmm::kNx>::Zero();
      grad.tail<6>() = Jt.row(j) / L;
      Row row = through_state(grad, k);
      if (row.head(np).norm() < kFixedRowGradient && std::abs(tau(j)) <= L) continue;  // nothing to bound
      Row up = row, down = row;
      up(soft) = -1.0;
      down(soft) = 1.0;
      push(up, -kInf, 1.0 - tau(j) / L, RowInfo{RowKind::Torque, k, {}, j});
      push(down, -1.0 - tau(j) / L, kInf, RowInfo{RowKind::Torque, k, {}, j});
    }
  }
  // forward-only base speed (soft, m/s): 0 <= v <= v_max
  for (int k = 1; k < K; ++k) {
    const double c = std::cos(X[k](2)), s = std::sin(X[k](2));
    const double v = c * V[k](0) + s * V[k](1);
    Row row = c * J(0, 1, k) + s * J(1, 1, k) + (-s * V[k](0) + c * V[k](1)) * J(2, 0, k);
    Row up = row, down = row;
    up(soft) = -1.0;
    down(soft) = 1.0;
    push(down, -v, kInf, RowInfo{RowKind::Speed, k, {}, -1});
    push(up, -kInf, limits_.v_max - v, RowInfo{RowKind::Speed, k, {}, -1});
  }
  // hull rows: hard where the nominal meets the bound, elastic (the hull slack) on a violated side
  auto hull = [&](int c, int r, double lower, double upper) {
    const Eigen::MatrixXd & D = hull_[r];
    const Eigen::VectorXd value = D * nominal.control_points.row(c).transpose();
    const Eigen::MatrixXd DM = D * basis_->map();
    for (int i = 0; i < D.rows(); ++i) {
      Row row = Row::Zero(n);
      row.segment(c * nf, nf) = DM.row(i);
      const RowInfo info{RowKind::Hull, -1, {}, c};
      if (value(i) > upper) {
        push(row, lower - value(i), kInf, info);
        Row elastic = row;
        elastic(hull_slack) = -1.0;
        push(elastic, -kInf, upper - value(i), info);
      } else if (value(i) < lower) {
        push(row, -kInf, upper - value(i), info);
        Row elastic = row;
        elastic(hull_slack) = 1.0;
        push(elastic, lower - value(i), kInf, info);
      } else {
        push(row, lower - value(i), upper - value(i), info);
      }
    }
  };
  for (size_t m = 0; m < joints_.size(); ++m) {
    const int c = 3 + static_cast<int>(m), j = joints_[m];
    hull(c, 0, limits_.q_min(j), limits_.q_max(j));
    hull(c, 1, -limits_.qd_max(j), limits_.qd_max(j));
    hull(c, 2, -limits_.qdd_max(j), limits_.qdd_max(j));
  }
  hull(2, 1, -limits_.omega_max, limits_.omega_max);
  const double accel = std::max(limits_.accel, limits_.decel);
  hull(0, 2, -accel, accel);
  hull(1, 2, -accel, accel);

  qp.A.resize(static_cast<int>(rows.size()), n);
  for (size_t i = 0; i < rows.size(); ++i) qp.A.row(static_cast<int>(i)) = rows[i];
  qp.lA = Eigen::Map<Eigen::VectorXd>(lo.data(), static_cast<int>(lo.size()));
  qp.uA = Eigen::Map<Eigen::VectorXd>(hi.data(), static_cast<int>(hi.size()));

  // ---- slacks, regularization, trust region ----
  for (const int s : {soft, hull_slack}) {
    qp.H(s, s) += w.slack_quadratic;
    qp.g(s) += w.slack;
  }
  qp.H.topLeftCorner(np, np).diagonal().array() += kRegularization;
  qp.lb = Eigen::VectorXd::Constant(n, -trust);
  qp.ub = Eigen::VectorXd::Constant(n, trust);
  for (const int s : {soft, hull_slack}) {
    qp.lb(s) = 0.0;
    qp.ub(s) = kInf;
  }
  return out;
}

MediumProblem::Evaluation MediumProblem::evaluate(const Trajectory & t, const RouteTask * task,
                                                  const mmm::World & world, double now) const
{
  const auto & w = config_.weights;
  const double dt = config_.collision_dt;
  const int K = samples();
  Evaluation e;
  std::vector<mmm::State> X(K);
  std::vector<Eigen::Matrix<double, 9, 1>> V(K);
  for (int k = 0; k < K; ++k) {
    X[k] = state_at(t, k);
    V[k] = rate_at(t, k);
  }
  // collision: the clearance hinge and the margin violation from one pass
  mmm::CollisionStats stats;
  for (const auto & row : collision_.rows(X, dt, now, world, &stats)) {
    if (row.stage <= 0 || row.stage >= K) continue;
    e.violations.collision_m = std::max(e.violations.collision_m, row.margin - row.distance);
    if (is_obstacle_row(row) && row.distance < config_.clearance_target_m) {
      const double h = config_.clearance_target_m - row.distance;
      e.terms["clearance"] += 0.5 * w.clearance * dt * h * h;
    }
  }
  for (const int k : torque_samples_) {
    const Joints tau = statics_->torque(X[k].tail<6>());
    for (int j = 0; j < 6; ++j) {
      const double L = config_.torque_limits_nm(j);
      if (std::isfinite(L)) e.violations.torque = std::max(e.violations.torque, (std::abs(tau(j)) - L) / L);
    }
  }
  for (int k = 1; k < K; ++k) {
    const double v = std::cos(X[k](2)) * V[k](0) + std::sin(X[k](2)) * V[k](1);
    e.violations.speed_mps = std::max({e.violations.speed_mps, -v, v - limits_.v_max});
  }
  e.violations.hull = hull_violation(t);
  if (task == nullptr) return e;

  if (task->route == nullptr) throw std::invalid_argument("MediumProblem: no route");
  const double window = std::max(1.0, 3.0 * limits_.v_max * dt);
  double s_hint = task->s0;
  for (int k = 1; k < K; ++k) {
    const Eigen::Vector2d p = X[k].head<2>();
    const auto proj = task->route->project(p, s_hint, window);
    s_hint = proj.s;
    const Eigen::Vector2d tan = tangent(*task->route, proj.s);
    const double speed_error = tan.dot(V[k].head<2>()) - target_speed(proj.s, task->goal_s);
    e.terms["progress"] += 0.5 * w.progress * dt * speed_error * speed_error;
    const double contour = Eigen::Vector2d(-tan.y(), tan.x()).dot(p - proj.point);
    e.terms["contour"] += 0.5 * w.contour * dt * contour * contour;
    for (const int j : joints_) {
      const double d = X[k](3 + j) - q_comfort_(j);
      e.terms["comfort"] += 0.5 * w.comfort * dt * d * d;
    }
    const double lateral = -std::sin(X[k](2)) * V[k](0) + std::cos(X[k](2)) * V[k](1);
    e.terms["skid"] += 0.5 * w.skid * dt * lateral * lateral;
  }
  const auto proj = task->route->project(X[K - 1].head<2>(), s_hint, window);
  const double closeness = goal_closeness(task->goal_s, proj.s);
  if (closeness > 0.0) {
    const double d = wrap(X[K - 1](2) - task->goal_yaw);
    e.terms["goal_heading"] += 0.5 * w.goal_heading * closeness * d * d;
  }
  for (int c = 0; c < coordinates(); ++c) {
    const double wc = c < 3 ? w.smooth_base : w.smooth_arm;
    const Eigen::VectorXd cp = t.control_points.row(c).transpose();
    e.terms["smooth"] += 0.5 * wc * cp.dot(basis_->acceleration_gram() * cp);
  }
  return e;
}

std::map<std::string, double> MediumProblem::cost_terms(const Trajectory & t, const RouteTask & task,
                                                        const mmm::World & world, double now) const
{
  return evaluate(t, &task, world, now).terms;
}

Violations MediumProblem::violations(const Trajectory & t, const mmm::World & world, double now) const
{
  return evaluate(t, nullptr, world, now).violations;
}

double MediumProblem::max_slack(const Trajectory & t, const mmm::World & world, double now) const
{
  return violations(t, world, now).soft();
}

double MediumProblem::merit(const Trajectory & t, const RouteTask & task, const mmm::World & world, double now) const
{
  const Evaluation e = evaluate(t, &task, world, now);
  double total = 0.0;
  for (const auto & [name, value] : e.terms) total += value;
  const auto & w = config_.weights;
  for (const double s : {e.violations.soft(), e.violations.hull}) total += w.slack * s + 0.5 * w.slack_quadratic * s * s;
  return total;
}

double MediumProblem::hull_violation(const Trajectory & t) const
{
  double worst = 0.0;
  auto check = [&](int c, int r, double lower, double upper) {
    const Eigen::VectorXd value = hull_[r] * t.control_points.row(c).transpose();
    if (value.size() > 0) worst = std::max({worst, lower - value.minCoeff(), value.maxCoeff() - upper});
  };
  for (size_t m = 0; m < joints_.size(); ++m) {
    const int c = 3 + static_cast<int>(m), j = joints_[m];
    check(c, 0, limits_.q_min(j), limits_.q_max(j));
    check(c, 1, -limits_.qd_max(j), limits_.qd_max(j));
    check(c, 2, -limits_.qdd_max(j), limits_.qdd_max(j));
  }
  check(2, 1, -limits_.omega_max, limits_.omega_max);
  const double accel = std::max(limits_.accel, limits_.decel);
  check(0, 2, -accel, accel);
  check(1, 2, -accel, accel);
  return worst;
}

bool MediumProblem::hull_ok(const Trajectory & t) const { return hull_violation(t) <= kHullTolerance; }
}  // namespace mobile_manipulator_medium
