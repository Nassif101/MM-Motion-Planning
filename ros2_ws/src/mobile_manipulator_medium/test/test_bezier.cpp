// The medium layer's trajectory basis (Phase 5A Task 5, spec Section 3.1): piecewise Bezier with continuity and the
// start state eliminated by an affine map from free parameters to control points.
#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "mobile_manipulator_medium/bezier.hpp"

namespace
{
namespace mmd = mobile_manipulator_medium;

Eigen::VectorXd random_vector(int n, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> u(-2.0, 2.0);
  Eigen::VectorXd v(n);
  for (int i = 0; i < n; ++i) v(i) = u(rng);
  return v;
}

double value(const mmd::PiecewiseBezier & b, const Eigen::VectorXd & c, double t, int derivative)
{
  return b.basis(t, derivative).dot(c);
}

TEST(Bezier, FreeParameterCount)
{
  EXPECT_EQ(mmd::PiecewiseBezier({3, 4, 8.0, 2}).control_points(), 13);
  EXPECT_EQ(mmd::PiecewiseBezier({3, 4, 8.0, 2}).free_parameters(), 5);  // 7 for a C2 cubic spline, less the start
  EXPECT_EQ(mmd::PiecewiseBezier({3, 4, 8.0, 1}).free_parameters(), 8);  // 13 - 3 C1 joins - 2
  EXPECT_EQ(mmd::PiecewiseBezier({5, 6, 8.0, 2}).free_parameters(), 31 - 10 - 2);
}

TEST(Bezier, StartStateHeld)
{
  const mmd::PiecewiseBezier b({3, 4, 8.0, 2});
  std::mt19937 rng(1);
  for (int trial = 0; trial < 20; ++trial) {
    const Eigen::VectorXd c = b.map() * random_vector(b.free_parameters(), rng) + b.offset(0.7, -0.3);
    EXPECT_NEAR(value(b, c, 0.0, 0), 0.7, 1e-12);
    EXPECT_NEAR(value(b, c, 0.0, 1), -0.3, 1e-12);
  }
}

TEST(Bezier, MapColumnsAreOrthonormal)
{
  // A trust region on the free parameters is then the same on the control points.
  const mmd::PiecewiseBezier b({3, 4, 8.0, 2});
  const Eigen::MatrixXd gram = b.map().transpose() * b.map();
  EXPECT_LT((gram - Eigen::MatrixXd::Identity(gram.rows(), gram.cols())).norm(), 1e-10);
}

TEST(Bezier, ContinuousAtTheJoins)
{
  for (const int continuity : {1, 2}) {
    const mmd::PiecewiseBezier b({3, 4, 8.0, continuity});
    std::mt19937 rng(2);
    const Eigen::VectorXd c = b.map() * random_vector(b.free_parameters(), rng) + b.offset(0.1, 0.2);
    for (int join = 1; join < 4; ++join) {
      const double t = 2.0 * join;
      for (int r = 0; r <= continuity; ++r)
        EXPECT_NEAR(value(b, c, t - 1e-9, r), value(b, c, t + 1e-9, r), 1e-6)
          << "continuity " << continuity << " join " << join << " derivative " << r;
    }
  }
}

TEST(Bezier, DerivativesMatchFiniteDifferences)
{
  const mmd::PiecewiseBezier b({3, 4, 8.0, 2});
  std::mt19937 rng(3);
  const Eigen::VectorXd c = b.map() * random_vector(b.free_parameters(), rng) + b.offset(0.0, 0.5);
  for (double t : {0.3, 1.7, 2.5, 5.1, 7.9}) {
    const double h = 1e-5;
    EXPECT_NEAR(value(b, c, t, 1), (value(b, c, t + h, 0) - value(b, c, t - h, 0)) / (2 * h), 1e-6) << t;
    EXPECT_NEAR(value(b, c, t, 2), (value(b, c, t + h, 1) - value(b, c, t - h, 1)) / (2 * h), 1e-5) << t;
  }
}

TEST(Bezier, HullBoundsContainSamples)
{
  // The convex-hull property: per segment, the curve and its derivatives lie between their control points' extremes.
  const mmd::PiecewiseBezier b({3, 4, 8.0, 2});
  std::mt19937 rng(4);
  const Eigen::VectorXd c = b.map() * random_vector(b.free_parameters(), rng) + b.offset(-0.4, 0.9);
  for (int r = 0; r <= 2; ++r) {
    const Eigen::VectorXd d = b.derivative_control_points(r) * c;
    const int per = b.spec().degree - r + 1;
    for (int k = 0; k < 4; ++k) {
      const Eigen::VectorXd seg = d.segment(k * per, per);
      for (int i = 0; i <= 250; ++i) {
        const double t = 2.0 * k + 2.0 * i / 250.0 * (1.0 - 1e-12);
        const double v = value(b, c, t, r);
        EXPECT_GE(v, seg.minCoeff() - 1e-9) << "r " << r << " t " << t;
        EXPECT_LE(v, seg.maxCoeff() + 1e-9) << "r " << r << " t " << t;
      }
    }
  }
}

TEST(Bezier, GramMatchesQuadrature)
{
  const mmd::PiecewiseBezier b({3, 4, 8.0, 2});
  std::mt19937 rng(5);
  const Eigen::VectorXd c = b.map() * random_vector(b.free_parameters(), rng) + b.offset(0.3, 0.1);
  double integral = 0.0;
  const int n = 80000;
  for (int i = 0; i < n; ++i) {
    const double t = 8.0 * (i + 0.5) / n;  // midpoint rule
    integral += std::pow(value(b, c, t, 2), 2) * 8.0 / n;
  }
  EXPECT_NEAR(c.dot(b.acceleration_gram() * c), integral, 1e-6 * std::max(1.0, integral));
}

TEST(Bezier, EndOfTheHorizonUsesTheLastSegment)
{
  const mmd::PiecewiseBezier b({3, 4, 8.0, 2});
  std::mt19937 rng(6);
  const Eigen::VectorXd c = b.map() * random_vector(b.free_parameters(), rng) + b.offset(0.0, 0.0);
  EXPECT_NEAR(value(b, c, 8.0, 0), c(c.size() - 1), 1e-12);
  EXPECT_THROW(b.basis(-0.1, 0), std::out_of_range);
  EXPECT_THROW(b.basis(8.1, 0), std::out_of_range);
}

// Task 9's grid: degree, segments, horizon, continuity.
const std::vector<mmd::BezierSpec> kGrid = {{3, 4, 8.0, 2}, {3, 6, 6.0, 2}, {3, 8, 10.0, 2}, {5, 8, 10.0, 4},
                                            {5, 6, 8.0, 2}, {4, 4, 8.0, 3}};

TEST(Bezier, DerivativeControlPointsEndWhereTheCurveDoes)
{
  // A Bezier segment interpolates its first and last control point, for the curve and every derivative.
  for (const auto & spec : kGrid) {
    const mmd::PiecewiseBezier b(spec);
    std::mt19937 rng(7);
    const Eigen::VectorXd c = b.map() * random_vector(b.free_parameters(), rng) + b.offset(0.2, -0.6);
    const double h = spec.horizon_s / spec.segments;
    for (int r = 0; r <= spec.degree; ++r) {
      const Eigen::VectorXd d = b.derivative_control_points(r) * c;
      const int per = spec.degree - r + 1;
      for (int k = 0; k < spec.segments; ++k) {
        const double scale = std::max(1.0, std::abs(d(k * per)));
        // The segment's own ends: just inside it (a join belongs to both segments only up to the continuity).
        EXPECT_NEAR(d(k * per), value(b, c, k * h + 1e-12, r), 1e-6 * scale) << "deg " << spec.degree << " r " << r;
        EXPECT_NEAR(d(k * per + per - 1), value(b, c, (k + 1) * h - 1e-12, r), 1e-6 * std::max(1.0, std::abs(d(k * per + per - 1))))
          << "deg " << spec.degree << " r " << r << " k " << k;
      }
    }
  }
}

TEST(Bezier, MapAndOffsetHoldOnTheWholeGrid)
{
  for (const auto & spec : kGrid) {
    const mmd::PiecewiseBezier b(spec);
    const int n = b.control_points();
    EXPECT_EQ(n, spec.segments * spec.degree + 1);
    EXPECT_EQ(b.free_parameters(), n - (spec.segments - 1) * spec.continuity - 2);
    const Eigen::MatrixXd gram = b.map().transpose() * b.map();
    EXPECT_LT((gram - Eigen::MatrixXd::Identity(gram.rows(), gram.cols())).norm(), 1e-10);
    // The minimum-norm offset is orthogonal to the map: p = map^T (c - offset) recovers the parameters.
    EXPECT_LT((b.map().transpose() * b.offset(1.3, -0.7)).norm(), 1e-10);
    std::mt19937 rng(8);
    const Eigen::VectorXd c = b.map() * random_vector(b.free_parameters(), rng) + b.offset(1.3, -0.7);
    EXPECT_NEAR(value(b, c, 0.0, 0), 1.3, 1e-10);
    EXPECT_NEAR(value(b, c, 0.0, 1), -0.7, 1e-10);
    const double h = spec.horizon_s / spec.segments;
    for (int join = 1; join < spec.segments; ++join)
      for (int r = 0; r <= spec.continuity; ++r)
        EXPECT_NEAR(value(b, c, join * h - 1e-10, r), value(b, c, join * h + 1e-10, r), 1e-6);
  }
}

TEST(Bezier, HullRowsAreUniqueAndDependOnTheParameters)
{
  // Cubic, 4 segments, C2: 16 / 12 / 8 rows for r = 0 / 1 / 2; the joins duplicate 3 each (C0, C1, C2); the start
  // fixes the first two position points and the first velocity point.
  const mmd::PiecewiseBezier b({3, 4, 8.0, 2});
  EXPECT_EQ(b.hull_rows(0).size(), 16u - 3u - 2u);
  EXPECT_EQ(b.hull_rows(1).size(), 12u - 3u - 1u);
  EXPECT_EQ(b.hull_rows(2).size(), 8u - 3u);
  for (int r = 0; r <= 2; ++r) {
    const Eigen::MatrixXd DM = b.derivative_control_points(r) * b.map();
    const auto rows = b.hull_rows(r);
    for (size_t i = 0; i < rows.size(); ++i) {
      EXPECT_GT(DM.row(rows[i]).norm(), 1e-9) << "r " << r;
      for (size_t j = 0; j < i; ++j) EXPECT_GT((DM.row(rows[i]) - DM.row(rows[j])).norm(), 1e-9) << "r " << r;
    }
  }
  // C1 only: the velocity points at the joins are shared, the accelerations are not.
  EXPECT_EQ(mmd::PiecewiseBezier({3, 4, 8.0, 1}).hull_rows(2).size(), 8u);
}

TEST(Bezier, NegativeDerivativeThrows)
{
  const mmd::PiecewiseBezier b({3, 4, 8.0, 2});
  EXPECT_THROW(b.basis(1.0, -1), std::invalid_argument);
  EXPECT_THROW(b.derivative_control_points(-1), std::invalid_argument);
  EXPECT_EQ(b.derivative_control_points(5).rows(), 0);
  EXPECT_TRUE(b.basis(1.0, 4).isZero());
}

TEST(Bezier, InvalidSpecThrows)
{
  EXPECT_THROW(mmd::PiecewiseBezier({2, 4, 8.0, 2}), std::invalid_argument);  // continuity must be below the degree
  EXPECT_THROW(mmd::PiecewiseBezier({3, 0, 8.0, 2}), std::invalid_argument);
  EXPECT_THROW(mmd::PiecewiseBezier({3, 4, 0.0, 2}), std::invalid_argument);
  EXPECT_THROW(mmd::PiecewiseBezier({3, 4, 8.0, 0}), std::invalid_argument);  // the start velocity needs C1
}
}  // namespace
