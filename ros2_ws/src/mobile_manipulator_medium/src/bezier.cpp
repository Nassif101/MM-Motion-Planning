#include "mobile_manipulator_medium/bezier.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/SVD>

namespace mobile_manipulator_medium
{
namespace
{
double binomial(int n, int k)
{
  if (k < 0 || k > n) return 0.0;
  double r = 1.0;
  for (int i = 1; i <= k; ++i) r = r * (n - k + i) / i;
  return r;
}

double bernstein(int i, int m, double tau)
{
  if (i < 0 || i > m) return 0.0;
  return binomial(m, i) * std::pow(tau, i) * std::pow(1.0 - tau, m - i);
}

// d! / (d - r)!
double falling(int d, int r)
{
  double f = 1.0;
  for (int i = 0; i < r; ++i) f *= d - i;
  return f;
}
}  // namespace

PiecewiseBezier::PiecewiseBezier(BezierSpec spec) : spec_(spec)
{
  const int d = spec_.degree, M = spec_.segments, C = spec_.continuity;
  if (d < 2 || M < 1 || !(spec_.horizon_s > 0.0) || C < 1 || C >= d)
    throw std::invalid_argument("PiecewiseBezier: needs degree >= 2, segments >= 1, horizon_s > 0 and continuity in "
                                "[1, degree - 1]");
  n_ = M * d + 1;
  h_ = spec_.horizon_s / M;

  // The equalities: C^1..C^C at each join, then the start position and velocity (the last two rows).
  const int rows = (M - 1) * C + 2;
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(rows, n_);
  int row = 0;
  for (int k = 1; k < M; ++k)
    for (int r = 1; r <= C; ++r) E.row(row++) = segment_row(k - 1, 1.0, r) - segment_row(k, 0.0, r);
  E.row(row++) = segment_row(0, 0.0, 0);
  E.row(row++) = segment_row(0, 0.0, 1);

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto & sigma = svd.singularValues();
  const double tol = 1e-10 * std::max(1.0, sigma(0));
  int rank = 0;
  while (rank < sigma.size() && sigma(rank) > tol) ++rank;
  map_ = svd.matrixV().rightCols(n_ - rank);
  // Minimum-norm particular solutions for a unit start position and a unit start velocity.
  Eigen::VectorXd e = Eigen::VectorXd::Zero(rows);
  e(rows - 2) = 1.0;
  offset_p_ = svd.solve(e);
  e(rows - 2) = 0.0;
  e(rows - 1) = 1.0;
  offset_v_ = svd.solve(e);

  // Gram of the second derivative: per segment, Bernstein products of degree m = d - 2 integrate exactly.
  const int m = d - 2;
  Eigen::MatrixXd W(m + 1, m + 1);
  for (int i = 0; i <= m; ++i)
    for (int j = 0; j <= m; ++j) W(i, j) = h_ * binomial(m, i) * binomial(m, j) / (binomial(2 * m, i + j) * (2 * m + 1));
  const Eigen::MatrixXd D = derivative_control_points(2);
  gram_ = Eigen::MatrixXd::Zero(n_, n_);
  for (int k = 0; k < M; ++k) {
    const Eigen::MatrixXd Dk = D.middleRows(k * (m + 1), m + 1);
    gram_ += Dk.transpose() * W * Dk;
  }
}

Eigen::RowVectorXd PiecewiseBezier::segment_row(int k, double tau, int derivative) const
{
  const int d = spec_.degree, r = derivative;
  Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(n_);
  if (r > d) return row;
  const double scale = falling(d, r) / std::pow(h_, r);
  for (int i = 0; i <= d - r; ++i) {
    const double b = bernstein(i, d - r, tau) * scale;
    if (b == 0.0) continue;
    for (int j = 0; j <= r; ++j) row(k * d + i + j) += b * (((r - j) % 2) ? -1.0 : 1.0) * binomial(r, j);
  }
  return row;
}

Eigen::RowVectorXd PiecewiseBezier::basis(double t, int derivative) const
{
  if (derivative < 0) throw std::invalid_argument("PiecewiseBezier::basis: negative derivative");
  if (t < -1e-9 || t > spec_.horizon_s + 1e-9) throw std::out_of_range("PiecewiseBezier::basis: t outside the horizon");
  const int k = std::clamp(static_cast<int>(std::floor(t / h_)), 0, spec_.segments - 1);
  const double tau = std::clamp((t - k * h_) / h_, 0.0, 1.0);
  return segment_row(k, tau, derivative);
}

Eigen::MatrixXd PiecewiseBezier::derivative_control_points(int derivative) const
{
  if (derivative < 0) throw std::invalid_argument("PiecewiseBezier::derivative_control_points: negative derivative");
  const int d = spec_.degree, r = derivative, per = std::max(0, d - r + 1);
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(spec_.segments * per, n_);
  if (r > d) return D;
  const double scale = falling(d, r) / std::pow(h_, r);
  for (int k = 0; k < spec_.segments; ++k)
    for (int i = 0; i < per; ++i)
      for (int j = 0; j <= r; ++j) D(k * per + i, k * d + i + j) += scale * (((r - j) % 2) ? -1.0 : 1.0) * binomial(r, j);
  return D;
}

Eigen::VectorXd PiecewiseBezier::linear(double p0, double v0) const
{
  // A line's Bezier control points are its values at evenly spaced parameters.
  Eigen::VectorXd c(n_);
  for (int i = 0; i < n_; ++i) c(i) = p0 + v0 * h_ * i / spec_.degree;
  return c;
}

std::vector<int> PiecewiseBezier::hull_rows(int derivative) const
{
  const Eigen::MatrixXd D = derivative_control_points(derivative);
  const Eigen::MatrixXd DM = D * map_;
  const Eigen::VectorXd dp = D * offset_p_, dv = D * offset_v_;
  constexpr double kTol = 1e-9;
  std::vector<int> rows;
  for (int i = 0; i < D.rows(); ++i) {
    if (DM.row(i).norm() <= kTol) continue;  // fixed by the start state
    bool duplicate = false;
    for (const int j : rows)
      duplicate = duplicate || ((DM.row(i) - DM.row(j)).norm() <= kTol && std::abs(dp(i) - dp(j)) <= kTol &&
                                std::abs(dv(i) - dv(j)) <= kTol);
    if (!duplicate) rows.push_back(i);
  }
  return rows;
}
}  // namespace mobile_manipulator_medium
