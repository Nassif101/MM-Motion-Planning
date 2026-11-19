#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "mobile_manipulator_routing/footprint.hpp"

namespace routing = mobile_manipulator_routing;
namespace
{
routing::Polygon rectangle(double x, double y)
{
  return {{-x,-y},{x,-y},{x,y},{-x,y}};
}

std::shared_ptr<const routing::GridSnapshot> grid(
  const std::vector<routing::CellIndex> & occupied = {},
  const std::vector<routing::CellIndex> & unknown = {})
{
  constexpr int width = 80, height = 80;
  std::vector<routing::Cell> cells(width*height,routing::Cell::Free);
  for (const auto & c : occupied) cells[c.y*width+c.x] = routing::Cell::Occupied;
  for (const auto & c : unknown) cells[c.y*width+c.x] = routing::Cell::Unknown;
  return std::make_shared<const routing::GridSnapshot>(width,height,0.05,routing::Point2{},
    std::move(cells),std::vector<routing::Cell>{},routing::InflationParameters{0.0,5.0});
}

routing::CheckStatus complete(routing::MotionCheck check, std::size_t slice = 100000)
{
  for (std::size_t call = 0; call < 1000000; ++call) {
    auto budget = slice;
    const auto result = check.advance(budget);
    if (result != routing::CheckStatus::Pending) return result;
  }
  ADD_FAILURE() << "motion query did not finish";
  return routing::CheckStatus::Pending;
}
}  // namespace

TEST(Footprint, DetectsOccupiedCellEntirelyInsidePolygon)
{
  routing::FootprintChecker checker(grid({{40,40}}),rectangle(0.3,0.2));
  EXPECT_FALSE(checker.pose_clear({2.0,2.0,0.0}));
  EXPECT_TRUE(checker.pose_clear({1.0,1.0,0.0}));
}

TEST(Footprint, CellEdgeAndCornerContactsAreBlocked)
{
  routing::FootprintChecker checker(grid({{40,40}}),rectangle(0.1,0.1));
  EXPECT_FALSE(checker.pose_clear({1.9,2.025,0.0}));  // right edge is the cell's left edge
  EXPECT_FALSE(checker.pose_clear({1.9,1.9,0.0}));    // one shared corner
  EXPECT_TRUE(checker.pose_clear({1.899,1.899,0.0}));
}

TEST(Footprint, RoundingCannotOmitACellAtTheFootprintBoundary)
{
  // Both expressions mean 0.35, but one is immediately below the other in binary64.
  // Enumerating floor(right/resolution) without a numerical envelope omits cell 7.
  ASSERT_LT(0.25+0.1,7*0.05);
  routing::FootprintChecker checker(grid({{7,40}}),rectangle(0.1,0.1));
  EXPECT_FALSE(checker.pose_clear({0.25,2.025,0.0}));
  EXPECT_TRUE(checker.pose_clear({0.249,2.025,0.0}));
}

TEST(Footprint, UnknownAndMapExteriorAreBlocked)
{
  routing::FootprintChecker checker(grid({},{{40,40}}),rectangle(0.1,0.1));
  EXPECT_FALSE(checker.pose_clear({2.0,2.0,0.0}));
  EXPECT_FALSE(checker.pose_clear({0.05,1.0,0.0}));
  EXPECT_FALSE(checker.pose_clear({0.1,1.0,0.0}));  // touches unknown exterior
  EXPECT_FALSE(checker.pose_clear({10.0,1.0,0.0}));
  EXPECT_TRUE(checker.pose_clear({0.101,1.0,0.0}));
}

TEST(Footprint, ChecksTranslationBetweenClearEndpoints)
{
  routing::FootprintChecker checker(grid({{40,40}}),rectangle(0.08,0.08));
  const routing::Pose2 a{1.5,2.025,0.0}, b{2.5,2.025,0.0};
  ASSERT_TRUE(checker.pose_clear(a));
  ASSERT_TRUE(checker.pose_clear(b));
  EXPECT_EQ(complete(checker.motion({a,b})),routing::CheckStatus::Blocked);
}

TEST(Footprint, ChecksRotatingCornerBetweenClearHeadings)
{
  // Horizontal/vertical bars miss this diagonal cell, but their 45-degree pose hits it.
  routing::FootprintChecker checker(grid({{46,46}}),rectangle(0.5,0.05));
  const routing::Pose2 a{2.0,2.0,0.0}, b{2.0,2.0,routing::kPi/2.0};
  ASSERT_TRUE(checker.pose_clear(a));
  ASSERT_TRUE(checker.pose_clear(b));
  EXPECT_EQ(complete(checker.motion({a,b})),routing::CheckStatus::Blocked);
}

TEST(Footprint, ConservativeEnvelopeCoversEvenCoarseInterpolation)
{
  routing::FootprintChecker checker(grid({{46,46}}),rectangle(0.5,0.05));
  // A single interval still checks the full displacement bound, rather than only endpoints.
  EXPECT_EQ(complete(checker.motion({{2,2,0},{2,2,routing::kPi/2}},2.0)),routing::CheckStatus::Blocked);
}

TEST(Footprint, WrapsHeadingAcrossPiAlongTheShortMotion)
{
  routing::FootprintChecker checker(grid(),rectangle(0.4,0.1));
  EXPECT_EQ(complete(checker.motion({{2,2,routing::kPi-0.02},{2,2,-routing::kPi+0.02}})),
    routing::CheckStatus::Clear);
}

TEST(Footprint, BudgetCanPauseWithinOnePoseAndSlicingDoesNotChangeResult)
{
  routing::FootprintChecker checker(grid(),rectangle(0.3,0.2));
  auto check = checker.motion({{1.5,1.5,0.0}});
  std::size_t zero = 0;
  EXPECT_EQ(check.advance(zero),routing::CheckStatus::Pending);
  EXPECT_EQ(zero,0u);
  std::size_t one = 1;
  EXPECT_EQ(check.advance(one),routing::CheckStatus::Pending);
  EXPECT_EQ(one,0u);
  EXPECT_EQ(complete(std::move(check),1),routing::CheckStatus::Clear);
  const std::vector<routing::Pose2> path{{1,1,0},{1.5,1.5,0.6},{2,2,0.8}};
  EXPECT_EQ(complete(checker.motion(path),1),complete(checker.motion(path),100000));
  routing::FootprintChecker blocked(grid({{30,30}}),rectangle(0.1,0.1));
  EXPECT_EQ(complete(blocked.motion(path),1),complete(blocked.motion(path),100000));
}

TEST(Footprint, CancellationIsLatchedAndHonouredWithoutCellBudget)
{
  routing::FootprintChecker checker(grid(),rectangle(0.2,0.1));
  auto check = checker.motion({{1,1,0},{2,2,0}});
  std::size_t budget = 0;
  EXPECT_EQ(check.advance(budget,[]{ return true; }),routing::CheckStatus::Canceled);
  budget = 100;
  EXPECT_EQ(check.advance(budget),routing::CheckStatus::Canceled);
  EXPECT_EQ(budget,100u);
}

TEST(Footprint, RejectsInvalidPolygonsAndMotionInputs)
{
  const auto map = grid();
  EXPECT_THROW(routing::FootprintChecker(nullptr,rectangle(1,1)),std::invalid_argument);
  EXPECT_THROW(routing::FootprintChecker(map,{{0,0},{1,0}}),std::invalid_argument);
  EXPECT_THROW(routing::FootprintChecker(map,{{0,0},{1,0},{2,0}}),std::invalid_argument);
  EXPECT_THROW(routing::FootprintChecker(map,{{0,0},{1,0},{1,0},{0,1}}),std::invalid_argument);
  EXPECT_THROW(routing::FootprintChecker(map,{{0,0},{1,0},{0.2,0.2},{0,1}}),std::invalid_argument);
  EXPECT_THROW(routing::FootprintChecker(map,{{0,0},{1,1},{0,1},{1,0}}),std::invalid_argument);
  EXPECT_THROW(routing::FootprintChecker(map,{{0,0},{1,0},{0,std::numeric_limits<double>::infinity()}}),
    std::invalid_argument);
  routing::FootprintChecker checker(map,rectangle(0.1,0.1));
  EXPECT_THROW(checker.motion({}),std::invalid_argument);
  EXPECT_THROW(checker.motion({{1,1,0}},0),std::invalid_argument);
  EXPECT_THROW(checker.motion({{1,1,0}},std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
  EXPECT_THROW(checker.motion({{1,1,std::numeric_limits<double>::quiet_NaN()}}),std::invalid_argument);
  EXPECT_THROW(checker.motion(std::vector<routing::Pose2>(100001)),std::invalid_argument);
  EXPECT_THROW(checker.motion({{1,1,0},{2,2,0}},1e-20),std::invalid_argument);
}

TEST(Footprint, AcceptsEitherWindingAndCountsOwnedStorage)
{
  auto polygon = rectangle(0.2,0.1);
  routing::FootprintChecker ccw(grid(),polygon);
  std::reverse(polygon.begin(),polygon.end());
  routing::FootprintChecker cw(grid(),polygon);
  EXPECT_EQ(ccw.pose_clear({2,2,0.4}),cw.pose_clear({2,2,0.4}));
  EXPECT_GT(cw.radius(),0.2);
  EXPECT_GE(cw.storage_bytes(),sizeof(routing::FootprintChecker)+polygon.size()*sizeof(routing::Point2));
  const auto check = cw.motion({{1,1,0},{2,2,0}});
  EXPECT_GT(check.storage_bytes(),2*sizeof(routing::Pose2));
}

#include <random>
#include "mobile_manipulator_routing/primitives.hpp"

// Plan Task 3: a lattice motion's swept mask agrees with the sampled check it replaces, for every pinned primitive
// from many start cells on random maps (unknown cells and the map's exterior included), and slices like it.
TEST(Footprint, SweptMasksAgreeWithTheSampledCheckForEveryPinnedPrimitive)
{
  const auto bank = routing::PrimitiveSet::load(std::string(ROUTING_SOURCE_DIR)+"/data/diff_5cm.json",0.05);
  // a 0.47 x 0.40 m half-size base, as the projected footprint's order of size, with a chamfered corner
  const routing::Polygon base{{-0.47,-0.395},{0.43,-0.395},{0.47,-0.35},{0.47,0.395},{-0.47,0.395}};
  std::mt19937 random(7);
  int blocked = 0, clear = 0;
  for (int map = 0; map < 6; ++map) {
    constexpr int width = 120, height = 100;
    std::vector<routing::Cell> cells(width*height,routing::Cell::Free);
    const double density[] = {0.0, 0.0003, 0.0006, 0.0012, 0.0025, 0.005};
    std::bernoulli_distribution occupied(density[map]), unknown(map == 0 ? 0.0 : 0.0002);
    for (auto & c : cells) c = occupied(random) ? routing::Cell::Occupied : unknown(random) ? routing::Cell::Unknown : c;
    const auto g = std::make_shared<const routing::GridSnapshot>(width,height,0.05,routing::Point2{-1.3,0.7},
      std::move(cells),std::vector<routing::Cell>{},routing::InflationParameters{0.0,5.0});
    const routing::FootprintChecker checker(g,base);
    std::uniform_int_distribution<int> sx(0,width-1), sy(0,height-1);
    for (std::size_t m = 0; m < bank.motions.size(); ++m) {
      const auto & motion = bank.motions[m];
      std::vector<routing::Pose2> relative(motion.poses.begin(),motion.poses.end());
      relative.back() = {motion.dx*0.05,motion.dy*0.05,bank.headings[motion.end_heading]};
      const auto mask = std::make_shared<const routing::SweptMask>(checker.swept_mask(relative));
      for (int trial = 0; trial < 6; ++trial) {
        const routing::CellIndex start{sx(random),sy(random)};
        const auto centre = g->cell_center(start.x,start.y);
        std::vector<routing::Pose2> absolute;
        for (const auto & p : relative) absolute.push_back({centre.x+p.x,centre.y+p.y,p.yaw});
        const auto sampled = complete(checker.motion(absolute));
        const auto masked = complete(checker.masked(mask,start),37);
        ASSERT_EQ(masked,sampled) << "primitive " << m << " from (" << start.x << ", " << start.y << ") on map " << map;
        (sampled == routing::CheckStatus::Clear ? clear : blocked) += 1;
      }
    }
  }
  EXPECT_GT(clear,200);  // both outcomes exercised
  EXPECT_GT(blocked,200);
}

TEST(Footprint, SweptMaskIsSmallerThanTheSampledWorkAndPausesWithinTheMask)
{
  const auto g = grid();
  const routing::FootprintChecker checker(g,rectangle(0.47,0.395));
  const std::vector<routing::Pose2> straight{{0,0,0},{0.5,0,0}};
  const auto mask = std::make_shared<const routing::SweptMask>(checker.swept_mask(straight));
  // a 0.5 m translation of a 0.94 x 0.79 m box sweeps about (0.94 + 0.5) x 0.79 m: ~460 cells, one unit each
  EXPECT_GT(mask->offsets.size(),400u);
  EXPECT_LT(mask->offsets.size(),600u);
  auto check = checker.masked(mask,{40,40});
  std::size_t budget = 10;
  EXPECT_EQ(check.advance(budget),routing::CheckStatus::Pending);
  EXPECT_EQ(budget,0u);
  EXPECT_EQ(complete(std::move(check)),routing::CheckStatus::Clear);
  EXPECT_THROW(checker.masked(mask,{-1,0}),std::invalid_argument);
  EXPECT_THROW(checker.masked(nullptr,{1,1}),std::invalid_argument);
}
