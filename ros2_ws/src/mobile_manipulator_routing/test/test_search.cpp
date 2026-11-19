// The route search after spec 2 Amendment 1: the Smac Lattice port behind the Search interface, with typed
// outcomes, resumable slices and the swept validation pass. Equivalence with Nav2's planner is checked separately in
// the container-only package mobile_manipulator_routing_nav2_check.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "mobile_manipulator_routing/search.hpp"

namespace mr = mobile_manipulator_routing;

namespace
{
const mr::Polygon kFootprint{{0.30, 0.25}, {0.30, -0.25}, {-0.30, -0.25}, {-0.30, 0.25}};

// A walled room of `w` x `h` metres at 5 cm; `walls` are extra occupied boxes (x0, y0, x1, y1) in metres.
std::shared_ptr<const mr::GridSnapshot> room(double w, double h, const std::vector<std::array<double, 4>> & walls = {},
                                             std::uint64_t revision = 1)
{
  const int cw = static_cast<int>(std::lround(w / 0.05)), ch = static_cast<int>(std::lround(h / 0.05));
  std::vector<mr::Cell> cells(static_cast<std::size_t>(cw) * ch, mr::Cell::Free);
  for (int y = 0; y < ch; ++y)
    for (int x = 0; x < cw; ++x) {
      const double px = (x + 0.5) * 0.05, py = (y + 0.5) * 0.05;
      bool blocked = x < 2 || y < 2 || x >= cw - 2 || y >= ch - 2;
      for (const auto & b : walls) blocked = blocked || (px >= b[0] && px <= b[2] && py >= b[1] && py <= b[3]);
      if (blocked) cells[static_cast<std::size_t>(y) * cw + x] = mr::Cell::Occupied;
    }
  return std::make_shared<const mr::GridSnapshot>(cw, ch, 0.05, mr::Point2{0.0, 0.0}, std::move(cells),
                                                  std::vector<mr::Cell>{}, mr::InflationParameters{}, revision);
}

mr::SearchConfiguration configuration()
{
  mr::SearchConfiguration c;
  c.lattice.lattice_filepath = std::string(ROUTING_SOURCE_DIR) + "/data/diff_5cm.json";
  return c;
}

std::shared_ptr<mr::RoutePlanner> planner(std::shared_ptr<const mr::GridSnapshot> grid,
                                          mr::SearchConfiguration c = configuration(),
                                          mr::Polygon validation = kFootprint)
{
  mr::smac::Footprint f;
  for (const auto & p : kFootprint) f.push_back({p.x, p.y});
  return std::make_shared<mr::RoutePlanner>(std::move(grid), f, std::move(validation), std::move(c));
}

mr::SearchResult run(mr::Search & search, mr::WorkBudget budget = {256, 524288})
{
  for (int i = 0; i < 1000000 && search.result().status == mr::SearchStatus::Pending; ++i) search.advance(budget);
  return search.result();
}

mr::SearchRequest request(mr::Pose2 start, mr::Pose2 goal) { return {start, goal, {}}; }
}  // namespace

TEST(Search, OpenRouteIsFoundValidatedAndEndsWithinTheTolerance)
{
  auto p = planner(room(6.0, 4.0));
  mr::Search search(p, request({1.0, 1.0, 0.0}, {4.5, 2.5, 0.0}));
  const auto r = run(search);
  ASSERT_EQ(r.status, mr::SearchStatus::Found) << r.reason;
  EXPECT_EQ(r.kind, mr::PathKind::Translating);
  ASSERT_GE(r.poses.size(), 3u);
  EXPECT_NEAR(r.poses.front().x, 1.0, 1e-6);
  EXPECT_NEAR(r.poses.front().y, 1.0, 1e-6);
  EXPECT_LE(std::hypot(r.resolved_goal.x - 4.5, r.resolved_goal.y - 2.5), 0.20 + 0.05);
  EXPECT_DOUBLE_EQ(r.requested_goal.x, 4.5);
  EXPECT_GT(r.statistics.expansions, 0u);
  EXPECT_GT(r.statistics.cell_tests, 0u);  // the validation pass ran
  EXPECT_GT(r.length_m, 3.0);
}

TEST(Search, SlicingDoesNotChangeTheResult)
{
  const auto grid = room(8.0, 6.0, {{3.0, 0.0, 3.4, 4.2}, {5.2, 1.8, 5.6, 6.0}});
  std::vector<mr::Pose2> reference;
  for (const std::size_t slice : {1000000ul, 1ul, 7ul, 256ul}) {
    auto p = planner(grid);
    mr::Search search(p, request({1.0, 1.0, 1.5}, {7.0, 1.0, -1.5}));
    const auto r = run(search, {slice, slice == 1 ? 5000ul : 524288ul});
    ASSERT_EQ(r.status, mr::SearchStatus::Found) << slice << ": " << r.reason;
    if (reference.empty()) { reference = r.poses; continue; }
    ASSERT_EQ(r.poses.size(), reference.size()) << slice;
    for (std::size_t i = 0; i < reference.size(); ++i) {
      EXPECT_EQ(r.poses[i].x, reference[i].x);
      EXPECT_EQ(r.poses[i].y, reference[i].y);
      EXPECT_EQ(r.poses[i].yaw, reference[i].yaw);
    }
  }
}

TEST(Search, OutcomesAreDistinct)
{
  const auto grid = room(6.0, 4.0, {{3.0, 0.0, 3.3, 4.0}});  // a full-height wall: the right half is unreachable
  auto p = planner(grid);
  const auto status = [&](mr::Pose2 s, mr::Pose2 g) { mr::Search search(p, request(s, g)); return run(search); };
  EXPECT_EQ(status({-1.0, 1.0, 0.0}, {2.0, 2.0, 0.0}).status, mr::SearchStatus::InvalidStart);
  EXPECT_EQ(status({1.0, 1.0, 0.0}, {9.0, 2.0, 0.0}).status, mr::SearchStatus::InvalidGoal);
  EXPECT_EQ(status({3.15, 2.0, 0.0}, {1.0, 1.0, 0.0}).status, mr::SearchStatus::InvalidStart);  // start in the wall
  EXPECT_EQ(status({1.0, 1.0, 0.0}, {4.5, 2.0, 0.0}).status, mr::SearchStatus::NoPath);
  auto c = configuration();
  c.lattice.max_iterations = 20;
  auto limited = planner(room(8.0, 6.0), c);
  mr::Search search(limited, request({1.0, 1.0, 0.0}, {7.0, 5.0, 3.0}));
  EXPECT_EQ(run(search).status, mr::SearchStatus::BudgetExhausted);
  EXPECT_EQ(search.result().statistics.expansions, 20u);
}

TEST(Search, SameCellGoalIsAStationaryRoute)
{
  auto p = planner(room(6.0, 4.0));
  mr::Search search(p, request({2.0, 2.0, 0.0}, {2.01, 2.01, 3.0}));
  const auto r = run(search);
  ASSERT_EQ(r.status, mr::SearchStatus::Found) << r.reason;
  EXPECT_EQ(r.kind, mr::PathKind::Stationary);
  ASSERT_EQ(r.poses.size(), 1u);
  EXPECT_NEAR(r.poses.front().x, 2.0, 1e-9);
  EXPECT_NEAR(std::remainder(r.poses.front().yaw - 3.0, 2 * M_PI), 0.0, 1e-9);
}

TEST(Search, ZeroWorkIsPendingAndCancellationAndRevisionsAreLatched)
{
  auto p = planner(room(6.0, 4.0));
  mr::Search pending(p, request({1.0, 1.0, 0.0}, {4.5, 2.5, 0.0}));
  EXPECT_EQ(pending.advance({0, 100}).status, mr::SearchStatus::Pending);
  pending.cancel();
  EXPECT_EQ(pending.result().status, mr::SearchStatus::Canceled);
  EXPECT_EQ(pending.advance().status, mr::SearchStatus::Canceled);

  mr::Search stale(p, request({1.0, 1.0, 0.0}, {4.5, 2.5, 0.0}));
  auto changed = stale.result().revisions;
  ++changed.goal;
  EXPECT_EQ(stale.advance({}, {}, changed).status, mr::SearchStatus::Stale);

  mr::Search mismatch(p, {{1.0, 1.0, 0.0}, {4.5, 2.5, 0.0}, {0, 0, 0, 99, 0, 0}});
  EXPECT_EQ(mismatch.result().status, mr::SearchStatus::Stale);
}

TEST(Search, ARouteFailingTheSweptValidationIsRejected)
{
  // The validation footprint is far larger than the planner's, so the route through the 1.4 m gap fails it.
  const mr::Polygon wide{{0.9, 0.9}, {0.9, -0.9}, {-0.9, -0.9}, {-0.9, 0.9}};
  auto p = planner(room(8.0, 6.0, {{3.8, 0.0, 4.2, 2.3}, {3.8, 3.7, 4.2, 6.0}}), configuration(), wide);
  mr::Search search(p, request({1.5, 3.0, 0.0}, {6.5, 3.0, 0.0}));
  const auto r = run(search);
  EXPECT_EQ(r.status, mr::SearchStatus::Rejected) << r.reason;
}

TEST(Search, RejectsMalformedConfiguration)
{
  auto c = configuration();
  c.lattice.lattice_filepath.clear();
  EXPECT_THROW(planner(room(6.0, 4.0), c), std::invalid_argument);
  c = configuration();
  c.max_point_step_m = 0.0;
  EXPECT_THROW(planner(room(6.0, 4.0), c), std::invalid_argument);
  EXPECT_THROW(mr::Search(nullptr, request({1, 1, 0}, {2, 2, 0})), std::invalid_argument);
}
