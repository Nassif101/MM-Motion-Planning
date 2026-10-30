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
}  // namespace

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

std::vector<double> MediumProblem::reference_s(const RouteTask & task) const
{
  const double v_nom = config_.speed_fraction * limits_.v_max;
  std::vector<double> s(samples());
  s[0] = std::min(task.s0, task.goal_s);
  for (int k = 1; k < samples(); ++k) {
    const double taper = std::clamp((task.goal_s - s[k - 1]) / mmm::kProgressTaperM, 0.0, 1.0);
    const double v = std::max(limits_.breakaway.linear_mps, v_nom * taper);
    s[k] = std::min(task.goal_s, s[k - 1] + v * (times_[k] - times_[k - 1]));
  }
  return s;
}

namespace
{
// The cost and constraint evaluation shared by linearize() (with the QP to fill) and the exact merit (without).
struct Evaluation
{
  std::map<std::string, double> terms{{"progress", 0.0}, {"contour", 0.0}, {"goal_heading", 0.0}, {"clearance", 0.0},
                                      {"comfort", 0.0}, {"smooth", 0.0}, {"skid", 0.0}};
  double violation = 0.0;
};
}  // namespace

MediumQp MediumProblem::linearize(const Trajectory & nominal, const RouteTask & task, const mmm::World & world,
                                  double now, double trust) const
{
  if (task.route == nullptr) throw std::invalid_argument("MediumProblem::linearize: no route");
  const int nf = basis_->free_parameters(), C = coordinates(), np = C * nf, n = np + 1, K = samples();
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
  const std::vector<double> s_ref = reference_s(task);
  const double window = std::max(1.0, 3.0 * limits_.v_max * dt);
  double s_hint = task.s0;
  for (int k = 1; k < K; ++k) {
    const Eigen::Vector2d p = X[k].head<2>();
    // progress: lag along the route tangent behind the reference point
    const double h_ref = task.route->heading_at(s_ref[k]);
    const Eigen::Vector2d t_ref(std::cos(h_ref), std::sin(h_ref));
    const Eigen::Vector2d d_ref = p - task.route->at(s_ref[k]);
    add_residual(qp.H, qp.g, w.progress * dt, t_ref.dot(d_ref), t_ref.x() * J(0, 0, k) + t_ref.y() * J(1, 0, k));
    // contour: normal offset at the projection
    const auto proj = task.route->project(p, s_hint, window);
    s_hint = proj.s;
    const Eigen::Vector2d nrm(-std::sin(proj.heading), std::cos(proj.heading));
    add_residual(qp.H, qp.g, w.contour * dt, nrm.dot(p - proj.point), nrm.x() * J(0, 0, k) + nrm.y() * J(1, 0, k));
    // comfort
    for (size_t m = 0; m < joints_.size(); ++m)
      add_residual(qp.H, qp.g, w.comfort * dt, X[k](3 + joints_[m]) - q_comfort_(joints_[m]),
                   J(3 + static_cast<int>(m), 0, k));
    // skid: body-frame lateral velocity
    const double c = std::cos(X[k](2)), s = std::sin(X[k](2));
    const double lateral = -s * V[k](0) + c * V[k](1);
    add_residual(qp.H, qp.g, w.skid * dt, lateral,
                 -s * J(0, 1, k) + c * J(1, 1, k) + (-c * V[k](0) - s * V[k](1)) * J(2, 0, k));
  }
  // goal heading: the last sample within kGoalHeadingM of the goal turns to the goal yaw
  {
    const auto proj = task.route->project(X[K - 1].head<2>(), s_hint, window);
    if (task.goal_s - proj.s <= mmm::kGoalHeadingM)
      add_residual(qp.H, qp.g, w.goal_heading, wrap(X[K - 1](2) - task.goal_yaw), J(2, 0, K - 1));
  }
  // smoothness: 0.5 w c^T G c per coordinate, in the parameters
  for (int c = 0; c < C; ++c) {
    const double wc = c < 3 ? w.smooth_base : w.smooth_arm;
    qp.H.block(c * nf, c * nf, nf, nf) += wc * gram_;
    qp.g.segment(c * nf, nf) += wc * basis_->map().transpose() * basis_->acceleration_gram() *
                                nominal.control_points.row(c).transpose();
  }

  // ---- collision rows (soft) and the clearance hinge ----
  std::vector<Row> rows;
  std::vector<double> lo, hi;
  auto push = [&](const Row & row, double l, double u, RowInfo info) {
    rows.push_back(row);
    lo.push_back(l);
    hi.push_back(u);
    out.rows.push_back(info);
  };
  mmm::CollisionStats stats;
  for (const auto & row : collision_.rows(X, dt, now, world, &stats)) {
    if (row.stage <= 0 || row.stage >= K) continue;
    Row grad = through_state(row.gradient, row.stage);
    if (is_obstacle_row(row) && row.distance < config_.clearance_target_m)
      add_residual(qp.H, qp.g, w.clearance * dt, config_.clearance_target_m - row.distance, -grad);
    grad(np) = 1.0;  // margin - d <= J dp + s
    RowInfo info{RowKind::Collision, row.stage, row, -1};
    push(grad, row.margin - row.distance, kInf, info);
  }
  // ---- static torque (soft): |tau_j| <= L_j ----
  for (const int k : torque_samples_) {
    const Joints q = X[k].tail<6>();
    const Joints tau = statics_->torque(q);
    const Eigen::Matrix<double, 6, 6> Jt = statics_->jacobian(q);
    for (int j = 0; j < 6; ++j) {
      const double L = config_.torque_limits_nm(j);
      if (!std::isfinite(L)) continue;
      Eigen::Matrix<double, 1, mmm::kNx> grad = Eigen::Matrix<double, 1, mmm::kNx>::Zero();
      grad.tail<6>() = Jt.row(j);
      Row row = through_state(grad, k);
      if (row.head(np).norm() < kFixedRowGradient && std::abs(tau(j)) <= L) continue;  // nothing to bound
      Row up = row, down = row;
      up(np) = -1.0;
      down(np) = 1.0;
      push(up, -kInf, L - tau(j), RowInfo{RowKind::Torque, k, {}, j});
      push(down, -L - tau(j), kInf, RowInfo{RowKind::Torque, k, {}, j});
    }
  }
  // ---- forward-only base speed (soft): 0 <= v <= v_max ----
  for (int k = 1; k < K; ++k) {
    const double c = std::cos(X[k](2)), s = std::sin(X[k](2));
    const double v = c * V[k](0) + s * V[k](1);
    Row row = c * J(0, 1, k) + s * J(1, 1, k) + (-s * V[k](0) + c * V[k](1)) * J(2, 0, k);
    Row up = row, down = row;
    up(np) = -1.0;
    down(np) = 1.0;
    push(down, -v, kInf, RowInfo{RowKind::Speed, k, {}, -1});
    push(up, -kInf, limits_.v_max - v, RowInfo{RowKind::Speed, k, {}, -1});
  }
  // ---- hull rows (hard; a violated one is not made worse) ----
  auto hull = [&](int c, int r, double lower, double upper) {
    const Eigen::MatrixXd & D = hull_[r];
    const Eigen::VectorXd value = D * nominal.control_points.row(c).transpose();
    const Eigen::MatrixXd DM = D * basis_->map();
    for (int i = 0; i < D.rows(); ++i) {
      Row row = Row::Zero(n);
      row.segment(c * nf, nf) = DM.row(i);
      push(row, std::min(lower - value(i), 0.0), std::max(upper - value(i), 0.0), RowInfo{RowKind::Hull, -1, {}, c});
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

  // ---- slack, regularization, trust region ----
  qp.H(np, np) += w.slack_quadratic;
  qp.g(np) += w.slack;
  qp.H.topLeftCorner(np, np).diagonal().array() += kRegularization;
  qp.lb = Eigen::VectorXd::Constant(n, -trust);
  qp.ub = Eigen::VectorXd::Constant(n, trust);
  qp.lb(np) = 0.0;
  qp.ub(np) = kInf;
  return out;
}

std::map<std::string, double> MediumProblem::cost_terms(const Trajectory & t, const RouteTask & task,
                                                        const mmm::World & world, double now) const
{
  if (task.route == nullptr) throw std::invalid_argument("MediumProblem::cost_terms: no route");
  const auto & w = config_.weights;
  const double dt = config_.collision_dt;
  const int K = samples();
  Evaluation e;
  std::vector<mmm::State> X(K);
  for (int k = 0; k < K; ++k) X[k] = state_at(t, k);
  const std::vector<double> s_ref = reference_s(task);
  const double window = std::max(1.0, 3.0 * limits_.v_max * dt);
  double s_hint = task.s0;
  for (int k = 1; k < K; ++k) {
    const Eigen::Vector2d p = X[k].head<2>();
    const double h_ref = task.route->heading_at(s_ref[k]);
    const double lag = Eigen::Vector2d(std::cos(h_ref), std::sin(h_ref)).dot(p - task.route->at(s_ref[k]));
    e.terms["progress"] += 0.5 * w.progress * dt * lag * lag;
    const auto proj = task.route->project(p, s_hint, window);
    s_hint = proj.s;
    const double contour = Eigen::Vector2d(-std::sin(proj.heading), std::cos(proj.heading)).dot(p - proj.point);
    e.terms["contour"] += 0.5 * w.contour * dt * contour * contour;
    for (const int j : joints_) {
      const double d = X[k](3 + j) - q_comfort_(j);
      e.terms["comfort"] += 0.5 * w.comfort * dt * d * d;
    }
    const Eigen::Matrix<double, 9, 1> v = rate_at(t, k);
    const double lateral = -std::sin(X[k](2)) * v(0) + std::cos(X[k](2)) * v(1);
    e.terms["skid"] += 0.5 * w.skid * dt * lateral * lateral;
  }
  const auto proj = task.route->project(X[K - 1].head<2>(), s_hint, window);
  if (task.goal_s - proj.s <= mmm::kGoalHeadingM) {
    const double d = wrap(X[K - 1](2) - task.goal_yaw);
    e.terms["goal_heading"] += 0.5 * w.goal_heading * d * d;
  }
  for (int c = 0; c < coordinates(); ++c) {
    const double wc = c < 3 ? w.smooth_base : w.smooth_arm;
    const Eigen::VectorXd cp = t.control_points.row(c).transpose();
    e.terms["smooth"] += 0.5 * wc * cp.dot(basis_->acceleration_gram() * cp);
  }
  mmm::CollisionStats stats;
  for (const auto & row : collision_.rows(X, dt, now, world, &stats)) {
    if (row.stage <= 0 || row.stage >= K || !is_obstacle_row(row) || row.distance >= config_.clearance_target_m)
      continue;
    const double h = config_.clearance_target_m - row.distance;
    e.terms["clearance"] += 0.5 * w.clearance * dt * h * h;
  }
  return e.terms;
}

double MediumProblem::max_slack(const Trajectory & t, const mmm::World & world, double now) const
{
  const int K = samples();
  std::vector<mmm::State> X(K);
  for (int k = 0; k < K; ++k) X[k] = state_at(t, k);
  double worst = 0.0;
  mmm::CollisionStats stats;
  for (const auto & row : collision_.rows(X, config_.collision_dt, now, world, &stats))
    if (row.stage > 0 && row.stage < K) worst = std::max(worst, row.margin - row.distance);
  for (const int k : torque_samples_) {
    const Joints tau = statics_->torque(X[k].tail<6>());
    for (int j = 0; j < 6; ++j) {
      const double L = config_.torque_limits_nm(j);
      if (std::isfinite(L)) worst = std::max(worst, (std::abs(tau(j)) - L) / L);
    }
  }
  for (int k = 1; k < K; ++k) {
    const Eigen::Matrix<double, 9, 1> v = rate_at(t, k);
    const double speed = std::cos(X[k](2)) * v(0) + std::sin(X[k](2)) * v(1);
    worst = std::max({worst, -speed, speed - limits_.v_max});
  }
  return worst;
}

double MediumProblem::merit(const Trajectory & t, const RouteTask & task, const mmm::World & world, double now) const
{
  double total = 0.0;
  for (const auto & [name, value] : cost_terms(t, task, world, now)) total += value;
  const double s = max_slack(t, world, now);
  return total + config_.weights.slack * s + 0.5 * config_.weights.slack_quadratic * s * s;
}

bool MediumProblem::hull_ok(const Trajectory & t) const
{
  constexpr double kTol = 1e-6;  // the QP backends' accuracy on an active bound
  auto inside = [&](int c, int r, double lower, double upper) {
    const Eigen::VectorXd value = hull_[r] * t.control_points.row(c).transpose();
    return value.size() == 0 || (value.minCoeff() >= lower - kTol && value.maxCoeff() <= upper + kTol);
  };
  for (size_t m = 0; m < joints_.size(); ++m) {
    const int c = 3 + static_cast<int>(m), j = joints_[m];
    if (!inside(c, 0, limits_.q_min(j), limits_.q_max(j)) || !inside(c, 1, -limits_.qd_max(j), limits_.qd_max(j)) ||
        !inside(c, 2, -limits_.qdd_max(j), limits_.qdd_max(j)))
      return false;
  }
  const double accel = std::max(limits_.accel, limits_.decel);
  return inside(2, 1, -limits_.omega_max, limits_.omega_max) && inside(0, 2, -accel, accel) &&
         inside(1, 2, -accel, accel);
}
}  // namespace mobile_manipulator_medium
