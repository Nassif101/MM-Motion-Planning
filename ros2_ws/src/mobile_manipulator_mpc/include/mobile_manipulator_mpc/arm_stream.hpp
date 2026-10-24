#pragma once
// The arm stream as the JTC samples it (the node's reference, the replay bench's previous arm command).
#include <Eigen/Core>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace mobile_manipulator_mpc
{
// Position and velocity of a sent trajectory `t` seconds after it was sent, as the JTC samples it from positions
// and velocities: a cubic Hermite spline between points; the first point before it (a first point at time 0 is
// never before), at rest at the last point after it. Joints in the trajectory's order (the model's).
// The start time (s, the node's clock) of a streamed trajectory led by `lead_s` (Phase 4B-B option A, ADR 0014): `now_s
// - lead_s`, or 0 (start on receipt) without a lead or before the clock passes it. The node's own sampling of what it
// sent keeps counting from the send time: the plan's time, where the arm is once the lead covers its delay.
inline double stream_start_seconds(double now_s, double lead_s)
{
  return lead_s > 0.0 && now_s - lead_s > 0.0 ? now_s - lead_s : 0.0;
}

// A streamed point after point 0: its time from the trajectory's start, positions and velocities.
struct StreamPoint
{
  double t;
  Eigen::Matrix<double, 6, 1> q, v;
};

// The arm stream's points after point 0 up to the plan's last stage, from the plan's arm joints q_stage (stages 0..N)
// and velocities u_stage (stages 0..N-1); trajectory time 0 lies `offset` after the plan's stage 0, and nothing is
// streamed within `min_lead_s` of it. Stage points: stage k at k dt - offset, the mean of the adjacent controls as
// velocity (the last stage's own). With dense_s > 0 (Phase 4B-C), points every dense_dt_s up to dense_s on the plan's
// own path, q_k + u_k (t - k dt) with velocity u_k, replace the stage points after the first one (which stays, so the
// first segment from point 0 keeps its length), so the JTC's splines follow the plan instead of bulging between stages.
std::vector<StreamPoint> plan_stream_points(const std::vector<Eigen::Matrix<double, 6, 1>> & q_stage,
                                            const std::vector<Eigen::Matrix<double, 6, 1>> & u_stage, double dt,
                                            double offset, double min_lead_s, double dense_s, double dense_dt_s);

void sample_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory, double t,
                       Eigen::Matrix<double, 6, 1> & q, Eigen::Matrix<double, 6, 1> & v);
}  // namespace mobile_manipulator_mpc
