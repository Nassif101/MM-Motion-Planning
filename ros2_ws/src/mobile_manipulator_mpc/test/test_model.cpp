#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_mpc/model.hpp"

namespace mmg = mobile_manipulator_geometry;
using namespace mobile_manipulator_mpc;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

const nlohmann::ordered_json & qualified_payload()
{
  static const auto json = nlohmann::ordered_json::parse(read_file(
    ament_index_cpp::get_package_share_directory("mobile_manipulator_control") +
    "/config/qualified_payload.json"));
  return json;
}

const mmg::FootprintProjector & projector()
{
  static const mmg::FootprintProjector instance(
    read_file(ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
              "/urdf/mobile_manipulator.urdf"),
    mmg::payload_from_json(qualified_payload()));
  return instance;
}

mmg::JointMap joint_map(const State & x)
{
  mmg::JointMap joints;
  const auto & order = qualified_payload()["joint_order"];
  for (int i = 0; i < 6; ++i) joints[order[i].get<std::string>()] = x(3 + i);
  return joints;
}

Eigen::Isometry3d base_pose(const State & x)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x(0), x(1), 0.0);
  pose.linear() = Eigen::AngleAxisd(x(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

State random_state(std::mt19937 & rng)
{
  std::uniform_real_distribution<double> position(-10.0, 10.0), angle(-M_PI, M_PI), joint(-2.0, 2.0);
  State x;
  x << position(rng), position(rng), angle(rng), joint(rng), joint(rng), joint(rng), joint(rng),
    joint(rng), joint(rng);
  return x;
}

Control random_control(std::mt19937 & rng)
{
  std::uniform_real_distribution<double> v(0.0, 0.3), omega(-0.4, 0.4), qdot(-0.6, 0.6);
  Control u;
  u << v(rng), omega(rng), qdot(rng), qdot(rng), qdot(rng), qdot(rng), qdot(rng), qdot(rng);
  return u;
}

State named_pose(const std::string & name)
{
  State x = State::Zero();
  const auto & values = qualified_payload()["poses_rad"][name];
  for (int i = 0; i < 6; ++i) x(3 + i) = values[i].get<double>();
  return x;
}

void expect_isometry_near(const Eigen::Isometry3d & actual, const Eigen::Isometry3d & expected,
                          double tolerance, const std::string & what)
{
  EXPECT_LT((actual.matrix() - expected.matrix()).cwiseAbs().maxCoeff(), tolerance) << what;
}

// Links that carry a collision primitive, including the payload's link.
std::set<std::string> primitive_links()
{
  std::set<std::string> links;
  for (const auto & primitive : projector().primitives()) links.insert(primitive.link);
  return links;
}
}  // namespace

TEST(Model, LinkIndexCoversPrimitiveLinksAndPanel)
{
  std::set<int> indices;
  for (const auto & link : primitive_links()) indices.insert(link_index(link));
  indices.insert(link_index("panel"));
  EXPECT_EQ(indices.size(), primitive_links().size() + 1);  // distinct
  EXPECT_TRUE(primitive_links().count("tool0"));            // the payload's link
  EXPECT_THROW(link_index("base_footprint"), std::invalid_argument);
  EXPECT_THROW(link_index("no_such_link"), std::invalid_argument);
}

TEST(Model, LinkPoseMatchesGeometryFk)
{
  std::mt19937 rng(7);
  for (int trial = 0; trial < 20; ++trial) {
    const State x = random_state(rng);
    for (const auto & link : primitive_links()) {
      expect_isometry_near(link_pose(link_index(link), x),
                           base_pose(x) * projector().link_pose(link, joint_map(x)), 1e-9,
                           link + " trial " + std::to_string(trial));
    }
  }
}

TEST(Model, PanelPoseMatchesProjector)
{
  std::mt19937 rng(11);
  for (int trial = 0; trial < 20; ++trial) {
    const State x = random_state(rng);
    expect_isometry_near(link_pose(link_index("panel"), x),
                         base_pose(x) * projector().panel_pose(joint_map(x)), 1e-9,
                         "trial " + std::to_string(trial));
  }
}

TEST(Model, StepJacobiansMatchFiniteDifferences)
{
  std::mt19937 rng(3);
  const double dt = 0.1, h = 1e-6;
  for (int trial = 0; trial < 10; ++trial) {
    const State x = random_state(rng);
    const Control u = random_control(rng);
    const Step step = discrete_step(x, u, dt);
    for (int j = 0; j < kNx; ++j) {
      State plus = x, minus = x;
      plus(j) += h;
      minus(j) -= h;
      const State column = (discrete_step(plus, u, dt).x_next - discrete_step(minus, u, dt).x_next) / (2.0 * h);
      EXPECT_LT((step.A.col(j) - column).cwiseAbs().maxCoeff(), 1e-6) << "A column " << j;
    }
    for (int j = 0; j < kNu; ++j) {
      Control plus = u, minus = u;
      plus(j) += h;
      minus(j) -= h;
      const State column = (discrete_step(x, plus, dt).x_next - discrete_step(x, minus, dt).x_next) / (2.0 * h);
      EXPECT_LT((step.B.col(j) - column).cwiseAbs().maxCoeff(), 1e-6) << "B column " << j;
    }
  }
}

TEST(Model, UnicycleHasNoLateralMotion)
{
  State x = State::Zero();
  x << 1.0, -2.0, 0.7, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  Control u = Control::Zero();
  u(0) = 0.3;
  const double dt = 0.1;
  const Step step = discrete_step(x, u, dt);
  const Eigen::Vector2d displacement = step.x_next.head<2>() - x.head<2>();
  const Eigen::Vector2d heading(std::cos(x(2)), std::sin(x(2)));
  EXPECT_NEAR(displacement.dot(heading), 0.3 * dt, 1e-12);
  EXPECT_NEAR(displacement.x() * -heading.y() + displacement.y() * heading.x(), 0.0, 1e-12);
  EXPECT_NEAR(step.x_next(2), x(2), 1e-12);
  EXPECT_LT((step.x_next.tail<6>() - x.tail<6>()).cwiseAbs().maxCoeff(), 1e-12);

  // Joint velocities integrate exactly; a pure turn does not translate.
  u << 0.0, 0.4, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  const Step turn = discrete_step(x, u, dt);
  EXPECT_LT((turn.x_next.head<2>() - x.head<2>()).norm(), 1e-12);
  EXPECT_NEAR(turn.x_next(2), x(2) + 0.4 * dt, 1e-12);
  EXPECT_LT((turn.x_next.tail<6>() - (x.tail<6>() + u.tail<6>() * dt)).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(Model, PointJacobianMatchesFiniteDifferences)
{
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> offset(-0.6, 0.6);
  const double h = 1e-6;
  const std::set<std::string> primitive = primitive_links();
  std::vector<std::string> links(primitive.begin(), primitive.end());
  links.push_back("panel");
  for (int trial = 0; trial < 5; ++trial) {
    const State x = random_state(rng);
    for (const auto & link : links) {
      const int index = link_index(link);
      const Eigen::Vector3d p_link(offset(rng), offset(rng), offset(rng));
      const PointJacobian result = point_jacobian(index, x, p_link);
      EXPECT_LT((result.p - link_pose(index, x) * p_link).cwiseAbs().maxCoeff(), 1e-12) << link;
      for (int j = 0; j < kNx; ++j) {
        State plus = x, minus = x;
        plus(j) += h;
        minus(j) -= h;
        const Eigen::Vector3d column =
          (point_jacobian(index, plus, p_link).p - point_jacobian(index, minus, p_link).p) / (2.0 * h);
        EXPECT_LT((result.J.col(j) - column).cwiseAbs().maxCoeff(), 1e-6) << link << " column " << j;
      }
    }
  }
}

TEST(Model, PanelErrorZeroAtReference)
{
  std::mt19937 rng(13);
  for (int trial = 0; trial < 10; ++trial) {
    const State x = random_state(rng);
    const PanelError error = panel_error(x, projector().panel_pose(joint_map(x)));
    EXPECT_LT(error.e.cwiseAbs().maxCoeff(), 1e-9) << "trial " << trial;
    // The reference is in base_footprint: the base pose does not enter the error.
    EXPECT_EQ(error.J.leftCols<3>().cwiseAbs().maxCoeff(), 0.0);
  }
  // Rotating the panel by an angle a about any axis gives 1 - cos(a).
  const State x = named_pose("vertical_carry");
  Eigen::Isometry3d reference = projector().panel_pose(joint_map(x));
  reference.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d(1.0, 2.0, -0.5).normalized()));
  EXPECT_NEAR(panel_error(x, reference).e(3), 1.0 - std::cos(0.3), 1e-12);
}

TEST(Model, PanelErrorJacobianMatchesFiniteDifferences)
{
  std::mt19937 rng(17);
  const double h = 1e-6;
  for (int trial = 0; trial < 10; ++trial) {
    const State x = random_state(rng);
    const Eigen::Isometry3d reference = projector().panel_pose(joint_map(random_state(rng)));
    const PanelError error = panel_error(x, reference);
    for (int j = 0; j < kNx; ++j) {
      State plus = x, minus = x;
      plus(j) += h;
      minus(j) -= h;
      const Eigen::Vector4d column = (panel_error(plus, reference).e - panel_error(minus, reference).e) / (2.0 * h);
      EXPECT_LT((error.J.col(j) - column).cwiseAbs().maxCoeff(), 1e-6) << "column " << j;
    }
  }
}

TEST(Model, ManipulabilityPositiveWhenBentAndDropsWhenStretched)
{
  // home (all joints zero) stands the whole arm upright with tool0 on the pan axis: fully
  // stretched and singular. The wrist bend of the other qualified poses takes tool0 off the
  // arm's line.
  EXPECT_LT(manipulability(named_pose("home")), 1e-9);
  EXPECT_GT(manipulability(named_pose("vertical_carry")), 0.01);
  EXPECT_GT(manipulability(named_pose("level_extension")), 0.01);

  State bent = named_pose("home");
  bent(4) = 0.6;  // shoulder_lift
  bent(5) = 1.2;  // elbow
  const double reference = manipulability(bent);
  EXPECT_GT(reference, 0.1);

  // Independent of the base pose.
  State moved = bent;
  moved.head<3>() << 3.0, -4.0, 1.1;
  EXPECT_NEAR(manipulability(moved), reference, 1e-12);

  // Straightening the elbow (wrist in line) lowers it monotonically to zero.
  double previous = reference;
  for (const double elbow : {0.9, 0.6, 0.3, 0.1}) {
    State x = bent;
    x(5) = elbow;
    const double value = manipulability(x);
    EXPECT_LT(value, previous) << "elbow " << elbow;
    previous = value;
  }
  State stretched = bent;
  stretched(5) = 0.0;
  EXPECT_LT(manipulability(stretched), 1e-9);
}
