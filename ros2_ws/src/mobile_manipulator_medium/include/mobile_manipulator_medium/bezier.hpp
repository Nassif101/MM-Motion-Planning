#pragma once
// The medium layer's trajectory basis (Phase 5A spec Section 3.1). One scalar coordinate (x, y, yaw or one arm joint)
// over the horizon is a linear combination of control points, c(t) = basis(t, 0) . c. Continuity between segments and
// the start state (position and velocity) are linear equalities on the control points; they are eliminated by an
// affine map c = map() p + offset(p0, v0) from the free parameters p, whose columns are orthonormal, so a trust region
// on p is the same on the control points and every QP has bounds and inequalities only.
//
// TrajectoryBasis is the interface the problem uses; PiecewiseBezier is F2. A B-spline basis (F4, optional) or another
// representation implements the same interface.
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

namespace mobile_manipulator_medium
{
class TrajectoryBasis
{
public:
  virtual ~TrajectoryBasis() = default;
  virtual double horizon_s() const = 0;
  virtual int control_points() const = 0;
  virtual int free_parameters() const = 0;
  // Row vector over all control points: the `derivative`-th time derivative at t in [0, horizon_s]
  // (std::out_of_range outside; std::invalid_argument for a negative derivative; zero above the degree).
  virtual Eigen::RowVectorXd basis(double t, int derivative) const = 0;
  virtual const Eigen::MatrixXd & map() const = 0;                  // control_points x free_parameters
  virtual Eigen::VectorXd offset(double p0, double v0) const = 0;  // control points of the start state alone
  // The control points of the straight line p0 + v0 t (it meets every continuity and start equality).
  virtual Eigen::VectorXd linear(double p0, double v0) const = 0;
  // The control points of the `derivative`-th derivative curve (per segment, for convex-hull bounds) as a linear map
  // of the control points.
  virtual Eigen::MatrixXd derivative_control_points(int derivative) const = 0;
  // The rows of derivative_control_points(derivative) a hull bound needs (Task 5 review): those that depend on the free
  // parameters, each once. The rows left out are duplicates (the shared points at the joins, up to the continuity) or
  // fixed by the start state alone (the first position points and the first velocity point): a bound on those cannot
  // be met by any p when the measured start already violates it, so the problem treats the start state separately
  // (slack or skip) instead of bounding these rows.
  virtual std::vector<int> hull_rows(int derivative) const = 0;
  // G with the integral of the squared second derivative over the horizon = c^T G c.
  virtual const Eigen::MatrixXd & acceleration_gram() const = 0;
};

struct BezierSpec
{
  int degree = 3;
  int segments = 4;
  double horizon_s = 8.0;
  int continuity = 2;  // C^continuity at the joins; at least 1 (the start velocity), below the degree
};

// Uniform segments of horizon_s / segments; neighbouring segments share their end control point (C0). Throws
// std::invalid_argument for degree < 2, segments < 1, horizon_s <= 0 or continuity outside [1, degree - 1].
class PiecewiseBezier : public TrajectoryBasis
{
public:
  explicit PiecewiseBezier(BezierSpec spec);

  const BezierSpec & spec() const { return spec_; }
  double horizon_s() const override { return spec_.horizon_s; }
  int control_points() const override { return n_; }
  int free_parameters() const override { return static_cast<int>(map_.cols()); }
  Eigen::RowVectorXd basis(double t, int derivative) const override;
  const Eigen::MatrixXd & map() const override { return map_; }
  Eigen::VectorXd offset(double p0, double v0) const override { return p0 * offset_p_ + v0 * offset_v_; }
  Eigen::VectorXd linear(double p0, double v0) const override;
  // Per segment, degree - derivative + 1 rows (segment-major); none above the degree; std::invalid_argument for a
  // negative derivative.
  Eigen::MatrixXd derivative_control_points(int derivative) const override;
  std::vector<int> hull_rows(int derivative) const override;
  const Eigen::MatrixXd & acceleration_gram() const override { return gram_; }

private:
  // The derivative of segment k at local tau in [0, 1] as a row over all control points.
  Eigen::RowVectorXd segment_row(int k, double tau, int derivative) const;

  BezierSpec spec_;
  int n_;
  double h_;
  Eigen::MatrixXd map_, gram_;
  Eigen::VectorXd offset_p_, offset_v_;
};
}  // namespace mobile_manipulator_medium
