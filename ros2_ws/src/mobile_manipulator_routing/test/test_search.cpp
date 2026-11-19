#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "mobile_manipulator_routing/search.hpp"

namespace mr = mobile_manipulator_routing;
namespace
{
constexpr double kResolution = 0.2;
mr::Polygon footprint() { return {{-0.025,-0.025},{0.025,-0.025},{0.025,0.025},{-0.025,0.025}}; }

std::shared_ptr<const mr::GridSnapshot> grid(const std::vector<mr::CellIndex> & occupied = {}, int size = 12)
{
  std::vector<mr::Cell> cells(static_cast<std::size_t>(size*size),mr::Cell::Free);
  for (auto at : occupied) cells[static_cast<std::size_t>(at.y*size+at.x)] = mr::Cell::Occupied;
  return std::make_shared<const mr::GridSnapshot>(size,size,kResolution,mr::Point2{},std::move(cells),
    std::vector<mr::Cell>{},mr::InflationParameters{0.0,5.0},7);
}

std::shared_ptr<mr::PrimitiveSet> bank()
{
  auto result = std::make_shared<mr::PrimitiveSet>();
  result->resolution = kResolution;
  result->headings = {0.0,mr::kPi/2.0,mr::kPi,3.0*mr::kPi/2.0};
  result->successors.resize(4);
  const int dx[]{1,0,-1,0}, dy[]{0,1,0,-1};
  for (std::size_t h = 0; h < 4; ++h) {
    mr::MotionPrimitive forward;
    forward.id = result->motions.size(); forward.start_heading = forward.end_heading = h;
    forward.dx = dx[h]; forward.dy = dy[h]; forward.length_m = kResolution;
    forward.poses = {{0,0,result->headings[h]},
      {dx[h]*kResolution,dy[h]*kResolution,result->headings[h]}};
    result->successors[h].push_back(result->motions.size());
    result->motions.push_back(forward);
    for (std::size_t end : {(h+1)%4,(h+3)%4}) {
      mr::MotionPrimitive rotate;
      rotate.id = result->motions.size(); rotate.start_heading = h; rotate.end_heading = end;
      rotate.angular_travel = mr::kPi/2.0; rotate.pure_rotation = rotate.turning = true;
      rotate.poses = {{0,0,result->headings[h]},{0,0,result->headings[end]}};
      result->successors[h].push_back(result->motions.size());
      result->motions.push_back(rotate);
    }
  }
  return result;
}

mr::SearchConfiguration configuration()
{
  mr::SearchConfiguration result;
  result.non_straight_penalty = 1.0;
  result.cost_weight = 0.0;
  return result;
}

mr::SearchResult run(mr::Search & search, mr::WorkBudget budget = {512,100000})
{
  for (std::size_t call = 0; call < 200000; ++call) {
    const auto & result = search.advance(budget);
    if (result.status != mr::SearchStatus::Pending) return result;
  }
  ADD_FAILURE() << "bounded fixture search did not terminate";
  return search.result();
}

std::vector<mr::Pose2> oracle_connector(mr::Pose2 from, mr::Pose2 to)
{
  const double length = std::hypot(to.x-from.x,to.y-from.y);
  if (length <= 1e-10) return {from,to};
  const double heading = std::atan2(to.y-from.y,to.x-from.x);
  return {from,{from.x,from.y,heading},{to.x,to.y,heading},to};
}

double oracle_connector_cost(mr::Pose2 from, mr::Pose2 to)
{
  const double length = std::hypot(to.x-from.x,to.y-from.y);
  if (length <= 1e-10) return 2.5*std::abs(mr::wrap_angle(to.yaw-from.yaw));
  const double heading = std::atan2(to.y-from.y,to.x-from.x);
  return length+2.5*(std::abs(mr::wrap_angle(heading-from.yaw))+std::abs(mr::wrap_angle(to.yaw-heading)));
}

bool clear(mr::FootprintChecker & checker, std::vector<mr::Pose2> path)
{
  auto query = checker.motion(std::move(path));
  std::size_t unlimited = std::numeric_limits<std::size_t>::max();
  return query.advance(unlimited) == mr::CheckStatus::Clear;
}

// Independent dense Dijkstra over the tiny fixture graph and its complete r2
// connector set. There is no heuristic, resumable state machine or A* queue entry type.
double dijkstra(const std::shared_ptr<const mr::GridSnapshot> & map,
                const std::shared_ptr<const mr::PrimitiveSet> & primitives, const mr::SearchRequest & request)
{
  mr::FootprintChecker checker(map,footprint());
  const auto start = *map->world_to_cell({request.start.x,request.start.y});
  const auto goal = *map->world_to_cell({request.goal.x,request.goal.y});
  const auto encode = [&](int x, int y, std::size_t h) {
    return (static_cast<std::size_t>(y)*map->width()+x)*primitives->headings.size()+h;
  };
  const auto pose = [&](std::size_t state) {
    const auto linear = state/primitives->headings.size();
    const auto xy = map->cell_center(static_cast<int>(linear%map->width()),static_cast<int>(linear/map->width()));
    return mr::Pose2{xy.x,xy.y,primitives->headings[state%primitives->headings.size()]};
  };
  std::vector<double> distances(static_cast<std::size_t>(map->width()*map->height())*primitives->headings.size(),
    std::numeric_limits<double>::infinity());
  using Entry = std::pair<double,std::size_t>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
  for (int y = start.y-2; y <= start.y+2; ++y) for (int x = start.x-2; x <= start.x+2; ++x) {
    if (!map->contains(x,y)) continue;
    for (std::size_t h = 0; h < primitives->headings.size(); ++h) {
      const auto target = encode(x,y,h);
      if (!clear(checker,oracle_connector(request.start,pose(target)))) continue;
      const double cost = oracle_connector_cost(request.start,pose(target));
      distances[target] = cost; queue.push({cost,target});
    }
  }
  double best = std::numeric_limits<double>::infinity();
  if (std::abs(start.x-goal.x) <= 2 && std::abs(start.y-goal.y) <= 2 &&
      clear(checker,oracle_connector(request.start,request.goal)))
    best = oracle_connector_cost(request.start,request.goal);
  while (!queue.empty()) {
    const auto [cost,state] = queue.top(); queue.pop();
    if (cost != distances[state] || cost >= best) continue;
    const auto from = pose(state);
    const auto cell = *map->world_to_cell({from.x,from.y});
    if (std::abs(cell.x-goal.x) <= 2 && std::abs(cell.y-goal.y) <= 2 &&
        clear(checker,oracle_connector(from,request.goal)))
      best = std::min(best,cost+oracle_connector_cost(from,request.goal));
    for (auto index : primitives->successors[state%primitives->headings.size()]) {
      const auto & motion = primitives->motions[index];
      const int x = cell.x+motion.dx, y = cell.y+motion.dy;
      if (!map->contains(x,y)) continue;
      std::vector<mr::Pose2> path;
      for (const auto & p : motion.poses) path.push_back({from.x+p.x,from.y+p.y,p.yaw});
      if (!clear(checker,std::move(path))) continue;
      const auto target = encode(x,y,motion.end_heading);
      const double edge = motion.pure_rotation ? 2.5*motion.angular_travel : motion.length_m;
      const double next = cost+edge;
      if (next < distances[target]) { distances[target] = next; queue.push({next,target}); }
    }
  }
  return best;
}

// Independently sample the returned continuous pose interpolation and a dense
// rectangular footprint, rather than invoking the production motion checker.
void expect_dense_clear(const mr::GridSnapshot & map, const std::vector<mr::Pose2> & path)
{
  ASSERT_FALSE(path.empty());
  const auto inspect = [&](const mr::Pose2 & p) {
    for (int iy = -2; iy <= 2; ++iy) for (int ix = -2; ix <= 2; ++ix) {
      const double x = ix*0.0125, y = iy*0.0125;
      const auto cell = map.world_to_cell({p.x+std::cos(p.yaw)*x-std::sin(p.yaw)*y,
        p.y+std::sin(p.yaw)*x+std::cos(p.yaw)*y});
      EXPECT_TRUE(cell.has_value());
      if (cell) EXPECT_FALSE(map.blocked(cell->x,cell->y));
    }
  };
  inspect(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i-1]; const auto & b = path[i];
    const double yaw = mr::wrap_angle(b.yaw-a.yaw);
    const auto count = static_cast<std::size_t>(std::max(1.0,std::ceil(
      (std::hypot(b.x-a.x,b.y-a.y)+0.04*std::abs(yaw))/0.002)));
    for (std::size_t n = 1; n <= count; ++n) {
      const double t = static_cast<double>(n)/count;
      inspect({a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.yaw+yaw*t});
    }
  }
}
}  // namespace

TEST(Search, OpenRouteAgreesWithIndependentDijkstra)
{
  const auto map = grid(); const auto motions = bank();
  const mr::SearchRequest request{{0.5,0.5,0},{1.7,0.5,0},{}};
  mr::Search search(map,motions,footprint(),request,configuration());
  const auto result = run(search);
  ASSERT_EQ(result.status,mr::SearchStatus::Found) << result.reason;
  EXPECT_NEAR(result.cost,dijkstra(map,motions,request),1e-10);
  EXPECT_NEAR(result.cost,1.2,1e-10);
  EXPECT_EQ(result.kind,mr::PathKind::Translating);
  EXPECT_DOUBLE_EQ(result.poses.front().x,request.start.x);
  EXPECT_DOUBLE_EQ(result.poses.back().x,request.goal.x);
  expect_dense_clear(*map,result.poses);
}

TEST(Search, DetourAgreesWithDijkstraAndDisconnectedMapExhaustsGraph)
{
  std::vector<mr::CellIndex> wall;
  for (int y = 0; y < 8; ++y) wall.push_back({6,y});
  const auto motions = bank(); const auto map = grid(wall);
  const mr::SearchRequest request{{0.5,0.7,0},{1.9,0.7,0},{}};
  mr::Search search(map,motions,footprint(),request,configuration());
  const auto result = run(search);
  ASSERT_EQ(result.status,mr::SearchStatus::Found) << result.reason;
  EXPECT_NEAR(result.cost,dijkstra(map,motions,request),1e-9);
  EXPECT_GT(result.length_m,1.4);
  expect_dense_clear(*map,result.poses);
  for (int y = 8; y < 12; ++y) wall.push_back({6,y});
  mr::Search disconnected(grid(wall),motions,footprint(),request,configuration());
  const auto absent = run(disconnected);
  EXPECT_EQ(absent.status,mr::SearchStatus::NoPath);
  EXPECT_TRUE(absent.poses.empty());
  EXPECT_NE(absent.reason.find("configured"),std::string::npos);
}

TEST(Search, ExactOffGridDestinationAndFinalHeadingArePreserved)
{
  const auto map = grid();
  const mr::SearchRequest request{{0.53,0.57,0.2},{1.77,1.51,-0.3},{}};
  mr::Search search(map,bank(),footprint(),request,configuration());
  const auto result = run(search);
  ASSERT_EQ(result.status,mr::SearchStatus::Found) << result.reason;
  EXPECT_DOUBLE_EQ(result.requested_goal.x,request.goal.x);
  EXPECT_DOUBLE_EQ(result.resolved_goal.x,request.goal.x);
  EXPECT_DOUBLE_EQ(result.poses.back().x,request.goal.x);
  EXPECT_DOUBLE_EQ(result.poses.back().y,request.goal.y);
  EXPECT_NEAR(mr::wrap_angle(result.poses.back().yaw-request.goal.yaw),0.0,1e-12);
  expect_dense_clear(*map,result.poses);
}

TEST(Search, StationaryHeadingChangeHasPositiveCostAndNoArtificialTranslation)
{
  const auto map = grid();
  mr::Search search(map,bank(),footprint(),{{0.5,0.5,0},{0.5,0.5,0.1},{}},configuration());
  const auto result = run(search);
  ASSERT_EQ(result.status,mr::SearchStatus::Found) << result.reason;
  EXPECT_EQ(result.kind,mr::PathKind::Stationary);
  EXPECT_DOUBLE_EQ(result.length_m,0.0);
  EXPECT_NEAR(result.cost,0.25,1e-12);
  for (const auto & pose : result.poses) { EXPECT_DOUBLE_EQ(pose.x,0.5); EXPECT_DOUBLE_EQ(pose.y,0.5); }
  expect_dense_clear(*map,result.poses);
  auto config = configuration(); config.max_path_poses = 1;
  mr::Search already_there(map,bank(),footprint(),{{0.5,0.5,0},{0.5,0.5,0},{}},config);
  const auto same = run(already_there);
  ASSERT_EQ(same.status,mr::SearchStatus::Found) << same.reason;
  EXPECT_EQ(same.poses.size(),1u);
}

TEST(Search, CheaperCurvedRouteWinsOverFirstExpensiveGoalConnector)
{
  auto motions = bank();
  mr::MotionPrimitive curve;
  curve.id = motions->motions.size(); curve.start_heading = 0; curve.end_heading = 1;
  curve.dx = curve.dy = 1; curve.length_m = mr::kPi*0.1;
  curve.angular_travel = mr::kPi/2.0; curve.turning = true;
  curve.poses = {{0,0,0},{std::sqrt(0.02),0.2-std::sqrt(0.02),mr::kPi/4.0},{0.2,0.2,mr::kPi/2.0}};
  motions->successors[0].push_back(motions->motions.size()); motions->motions.push_back(curve);
  auto config = configuration(); config.non_straight_penalty = 1.2;
  mr::Search search(grid(),motions,footprint(),{{0.5,0.5,0},{0.7,0.7,mr::kPi/2.0},{}},config);
  const auto result = run(search);
  ASSERT_EQ(result.status,mr::SearchStatus::Found) << result.reason;
  EXPECT_NEAR(result.cost,1.2*curve.length_m,1e-10);
  EXPECT_LT(result.cost,oracle_connector_cost({0.5,0.5,0},{0.7,0.7,mr::kPi/2.0}));
  EXPECT_GT(result.length_m,std::hypot(0.2,0.2));
}

TEST(Search, BlockedSamePositionRotationCanUseATranslatingGraphRoute)
{
  const auto map = grid({{7,7}},20);
  const mr::Polygon bar{{-0.5,-0.05},{0.5,-0.05},{0.5,0.05},{-0.5,0.05}};
  const mr::SearchRequest request{{1.1,1.1,0},{1.1,1.1,mr::kPi/2.0},{}};
  mr::FootprintChecker checker(map,bar);
  ASSERT_TRUE(checker.pose_clear(request.start));
  ASSERT_TRUE(checker.pose_clear(request.goal));
  ASSERT_FALSE(clear(checker,{request.start,request.goal}));
  mr::Search search(map,bank(),bar,request,configuration());
  const auto result = run(search);
  ASSERT_EQ(result.status,mr::SearchStatus::Found) << result.reason;
  EXPECT_EQ(result.kind,mr::PathKind::Translating);
  EXPECT_GT(result.length_m,0.0);
  EXPECT_TRUE(clear(checker,result.poses));
  EXPECT_DOUBLE_EQ(result.poses.back().x,request.goal.x);
  EXPECT_DOUBLE_EQ(result.poses.back().y,request.goal.y);
}

TEST(Search, WorkSlicingAndTieOrderingDoNotChangeSelectedPath)
{
  const auto map = grid({{6,4},{6,5},{6,6},{6,7}}); const auto motions = bank();
  const mr::SearchRequest request{{0.5,1.1,0},{1.9,1.1,0},{}};
  mr::Search small(map,motions,footprint(),request,configuration());
  mr::Search large(map,motions,footprint(),request,configuration());
  const auto a = run(small,{3,7}), b = run(large,{512,100000});
  ASSERT_EQ(a.status,mr::SearchStatus::Found) << a.reason;
  ASSERT_EQ(b.status,mr::SearchStatus::Found) << b.reason;
  EXPECT_DOUBLE_EQ(a.cost,b.cost);
  EXPECT_EQ(a.statistics.expansions,b.statistics.expansions);
  EXPECT_EQ(a.statistics.cell_tests,b.statistics.cell_tests);
  ASSERT_EQ(a.poses.size(),b.poses.size());
  for (std::size_t i = 0; i < a.poses.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.poses[i].x,b.poses[i].x);
    EXPECT_DOUBLE_EQ(a.poses[i].y,b.poses[i].y);
    EXPECT_DOUBLE_EQ(a.poses[i].yaw,b.poses[i].yaw);
  }
  expect_dense_clear(*map,a.poses);
}

TEST(Search, ZeroWorkIsPendingAndSmallCellBudgetPausesWithinCollisionCheck)
{
  mr::Search search(grid(),bank(),footprint(),{{0.5,0.5,0},{1.7,0.5,0},{}},configuration());
  EXPECT_EQ(search.advance({0,100}).status,mr::SearchStatus::Pending);
  EXPECT_EQ(search.advance({100,0}).status,mr::SearchStatus::Pending);
  EXPECT_EQ(search.result().statistics.cell_tests,0u);
  EXPECT_EQ(search.advance({1,1}).status,mr::SearchStatus::Pending);
  EXPECT_LE(search.result().statistics.cell_tests,1u);
  const auto result = run(search);
  EXPECT_EQ(result.status,mr::SearchStatus::Found);
}

TEST(Search, InvalidPosesAndBlockedFootprintsHaveDistinctOutcomes)
{
  auto motions = bank();
  mr::Search outside(grid(),motions,footprint(),{{-1,0.5,0},{1.7,0.5,0},{}},configuration());
  EXPECT_EQ(outside.result().status,mr::SearchStatus::InvalidStart);
  mr::Search nonfinite(grid(),motions,footprint(),{{0.5,0.5,0},
    {1.7,0.5,std::numeric_limits<double>::quiet_NaN()},{}},configuration());
  EXPECT_EQ(nonfinite.result().status,mr::SearchStatus::InvalidGoal);
  mr::Search start_blocked(grid({{2,2}}),motions,footprint(),{{0.5,0.5,0},{1.7,0.5,0},{}},configuration());
  EXPECT_EQ(run(start_blocked).status,mr::SearchStatus::InvalidStart);
  mr::Search goal_blocked(grid({{8,2}}),motions,footprint(),{{0.5,0.5,0},{1.7,0.5,0},{}},configuration());
  EXPECT_EQ(run(goal_blocked).status,mr::SearchStatus::InvalidGoal);
}

TEST(Search, CancellationAndRevisionInvalidationAreLatched)
{
  const auto map = grid(); const auto motions = bank();
  mr::SearchRequest request{{0.5,0.5,0},{1.7,0.5,0},{1,2,3,0,4,5}};
  mr::Search stale(map,motions,footprint(),request,configuration());
  ASSERT_EQ(stale.result().revisions.map,7u);
  stale.advance({1,1});
  auto changed = stale.result().revisions; ++changed.goal;
  EXPECT_EQ(stale.advance({}, {}, changed).status,mr::SearchStatus::Stale);
  EXPECT_EQ(stale.advance().status,mr::SearchStatus::Stale);
  request.revisions.map = 8;
  mr::Search wrong_snapshot(map,motions,footprint(),request,configuration());
  EXPECT_EQ(wrong_snapshot.result().status,mr::SearchStatus::Stale);
  request.revisions.map = 0;
  mr::Search canceled(map,motions,footprint(),request,configuration());
  canceled.advance({1,1}); canceled.cancel();
  EXPECT_EQ(canceled.advance().status,mr::SearchStatus::Canceled);
  mr::Search callback(map,motions,footprint(),request,configuration());
  int polls = 0;
  EXPECT_EQ(callback.advance({128,1000},[&] { return ++polls > 15; }).status,mr::SearchStatus::Canceled);
  EXPECT_LT(callback.result().statistics.cell_tests,1000u);
}

TEST(Search, ExpansionMemoryAndPathBoundsAreReportedRatherThanNoPath)
{
  auto config = configuration(); config.max_expansions = 1;
  const auto map = grid(); const auto motions = bank();
  const mr::SearchRequest request{{0.5,0.5,0},{1.9,0.5,0},{}};
  mr::Search limited(map,motions,footprint(),request,config);
  EXPECT_EQ(run(limited).status,mr::SearchStatus::BudgetExhausted);
  config = configuration(); config.max_memory_bytes = 1;
  mr::Search memory(map,motions,footprint(),request,config);
  EXPECT_EQ(memory.result().status,mr::SearchStatus::BudgetExhausted);
  EXPECT_LE(memory.result().statistics.peak_storage_bytes,config.max_memory_bytes);
  config = configuration(); config.max_path_poses = 1;
  mr::Search path(map,motions,footprint(),request,config);
  EXPECT_EQ(run(path).status,mr::SearchStatus::BudgetExhausted);
}

TEST(Search, RejectsMalformedConfigurationAndTypedPrimitiveInputs)
{
  const auto map = grid();
  const mr::SearchRequest request{{0.5,0.5,0},{1.7,0.5,0},{}};
  for (int mutation = 0; mutation < 7; ++mutation) {
    SCOPED_TRACE(mutation);
    auto config = configuration();
    if (mutation == 0) config.non_straight_penalty = 0.9;
    if (mutation == 1) config.cost_weight = -1;
    if (mutation == 2) config.rotation_cost_m_per_rad = 0;
    if (mutation == 3) config.max_point_step_m = 0;
    if (mutation == 4) config.max_expansions = 0;
    if (mutation == 5) config.max_path_poses = 100001;
    if (mutation == 6) config.connector_radius_cells = 3;
    EXPECT_THROW(mr::Search(map,bank(),footprint(),request,config),std::invalid_argument);
  }
  for (int mutation = 0; mutation < 7; ++mutation) {
    SCOPED_TRACE(mutation);
    auto motions = bank();
    if (mutation == 0) motions->motions[0].length_m = 0.01;
    if (mutation == 1) motions->motions[1].angular_travel = 0.0;
    if (mutation == 2) motions->motions[0].turning = true;
    if (mutation == 3) motions->motions[0].pure_rotation = true;
    if (mutation == 4) motions->motions[0].poses.back().y = 1e-5;
    if (mutation == 5) motions->successors[0].push_back(0);
    if (mutation == 6) {
      motions->motions[0].poses.back().x = -kResolution;
      motions->motions[0].dx = -1;
    }
    EXPECT_THROW(mr::Search(map,motions,footprint(),request,configuration()),std::invalid_argument);
  }
}

TEST(Search, ExpansionBudgetCountsNodeExpansions)
{
  // the slice unit is a node expansion (the plan's 256 per tick); seeds, successors and stale entries cost cell units
  mr::Search whole(grid({{6,3},{6,4},{6,5},{6,6},{6,7}}),bank(),footprint(),{{0.5,0.25,0},{0.5,0.5,0},{}},
                   configuration());
  const auto reference = run(whole,{1u << 20,1u << 30});
  ASSERT_EQ(reference.status,mr::SearchStatus::Found) << reference.reason;
  ASSERT_GT(reference.statistics.expansions,5u);
  mr::Search sliced(grid({{6,3},{6,4},{6,5},{6,6},{6,7}}),bank(),footprint(),{{0.5,0.25,0},{0.5,0.5,0},{}},
                    configuration());
  std::size_t calls = 0, previous = 0;
  while (sliced.advance({1,1u << 30}).status == mr::SearchStatus::Pending) {
    ++calls;
    ASSERT_LE(sliced.result().statistics.expansions,previous+1);
    previous = sliced.result().statistics.expansions;
    ASSERT_LT(calls,10000u);
  }
  EXPECT_LE(calls,reference.statistics.expansions+1);  // one expansion per slice, nothing else needs a slice
  EXPECT_EQ(sliced.result().statistics.expansions,reference.statistics.expansions);
  EXPECT_DOUBLE_EQ(sliced.result().cost,reference.cost);
}

TEST(Search, ASharedMaskBankGivesTheSameRouteAndRefusesOtherInputs)
{
  const auto map = grid({{6,3},{6,4},{6,5},{6,6},{6,7}});
  const auto motions = bank();
  auto masks = std::make_shared<mr::SweptMaskBank>(map,motions,footprint(),configuration().max_point_step_m);
  mr::Search own(map,motions,footprint(),{{0.5,0.25,0},{0.5,0.5,0},{}},configuration());
  mr::Search shared(map,motions,footprint(),{{0.5,0.25,0},{0.5,0.5,0},{}},configuration(),masks);
  const auto a = run(own), b = run(shared);
  ASSERT_EQ(a.status,mr::SearchStatus::Found);
  ASSERT_EQ(b.status,mr::SearchStatus::Found);
  EXPECT_DOUBLE_EQ(a.cost,b.cost);
  EXPECT_EQ(a.statistics.cell_tests,b.statistics.cell_tests);
  ASSERT_EQ(a.poses.size(),b.poses.size());
  EXPECT_GT(masks->storage_bytes(),sizeof(mr::SweptMaskBank));
  // a later occupancy revision of the same geometry is served; another footprint or step is not
  mr::Search again(grid({{6,3}}),motions,footprint(),{{0.5,0.25,0},{0.5,0.5,0},{}},configuration(),masks);
  EXPECT_EQ(run(again).status,mr::SearchStatus::Found);
  auto bigger = footprint();
  for (auto & p : bigger) p.x *= 1.5;
  EXPECT_THROW(mr::Search(map,motions,bigger,{{0.5,0.25,0},{0.5,0.5,0},{}},configuration(),masks),std::invalid_argument);
  auto coarse = configuration();
  coarse.max_point_step_m *= 2.0;
  EXPECT_THROW(mr::Search(map,motions,footprint(),{{0.5,0.25,0},{0.5,0.5,0},{}},coarse,masks),std::invalid_argument);
}

namespace
{
// the pinned Nav2 bank's 5 cm grid (the fixtures above use 0.2 m)
std::shared_ptr<const mr::GridSnapshot> grid05(const std::vector<mr::CellIndex> & occupied, int size)
{
  std::vector<mr::Cell> cells(static_cast<std::size_t>(size*size),mr::Cell::Free);
  for (auto at : occupied) cells[static_cast<std::size_t>(at.y*size+at.x)] = mr::Cell::Occupied;
  return std::make_shared<const mr::GridSnapshot>(size,size,0.05,mr::Point2{},std::move(cells),
    std::vector<mr::Cell>{},mr::InflationParameters{0.0,5.0},7);
}
}  // namespace

TEST(Search, TheFreeSpaceHeuristicKeepsReorientationRoutesOptimal)
{
  // The pinned Nav2 bank (16 nonuniform headings, arcs and in-place rotations) on a 2 m map with a wall: goals
  // behind the start, beside it and with the opposite yaw, against the independent Dijkstra. An over-estimating
  // bound would return a costlier route.
  const auto motions = std::make_shared<const mr::PrimitiveSet>(
    mr::PrimitiveSet::load(std::string(ROUTING_SOURCE_DIR)+"/data/diff_5cm.json",0.05));
  std::vector<mr::CellIndex> wall;
  for (int y = 6; y < 26; ++y) wall.push_back({24,y});
  const auto map = grid05(wall,40);
  const std::vector<mr::SearchRequest> cases{
    {{1.0,1.0,0.0},{0.6,1.0,0.0},{}},              // 0.4 m behind, same yaw
    {{1.0,1.0,0.0},{1.0,1.0,mr::kPi},{}},          // turn around in place
    {{0.7,0.5,0.3},{0.5,1.5,-2.0},{}},             // beside, off-bin yaw
    {{0.5,0.5,mr::kPi/2},{1.6,0.9,-mr::kPi/2},{}}, // past the wall's end, reversed heading
  };
  for (const auto & request : cases) {
    mr::Search search(map,motions,footprint(),request,configuration());
    const auto result = run(search,{4096,1u << 24});
    const double expected = dijkstra(map,motions,request);
    ASSERT_EQ(result.status,mr::SearchStatus::Found) << result.reason;
    EXPECT_NEAR(result.cost,expected,1e-9) << "to (" << request.goal.x << ", " << request.goal.y << ")";
  }
}

TEST(Search, AFreeSpaceBoundNeverExceedsTheObstacleFreeCost)
{
  // the table against the search itself on an open map: for random lattice starts, the bound (with the goal-yaw
  // correction) is at most the found route's cost from that state
  const auto motions = std::make_shared<const mr::PrimitiveSet>(
    mr::PrimitiveSet::load(std::string(ROUTING_SOURCE_DIR)+"/data/diff_5cm.json",0.05));
  const auto map = grid05({},40);
  auto table = std::make_shared<mr::FreeSpaceHeuristic>(motions,1.0,2.5,30);
  const mr::Pose2 goal{1.025,1.025,0.4};
  const auto bin = motions->nearest_heading(goal.yaw);
  const double gap = 2.5*std::abs(mr::wrap_angle(goal.yaw-motions->headings[bin]));
  std::mt19937 random(3);
  std::uniform_int_distribution<int> cell(4,35);
  std::uniform_int_distribution<std::size_t> heading(0,motions->headings.size()-1);
  for (int trial = 0; trial < 40; ++trial) {
    const int x = cell(random), y = cell(random);
    const auto h = heading(random);
    const auto c = map->cell_center(x,y);
    mr::Search search(map,motions,footprint(),{{c.x,c.y,motions->headings[h]},goal,{}},configuration(),nullptr,table);
    const auto result = run(search,{4096,1u << 24});
    ASSERT_EQ(result.status,mr::SearchStatus::Found);
    EXPECT_LE(table->bound(x-20,y-20,h,bin)-gap,result.cost+1e-9) << x << " " << y << " " << h;
  }
  EXPECT_THROW(mr::Search(map,motions,footprint(),{{0.5,0.5,0},goal,{}},configuration(),nullptr,
                          std::make_shared<mr::FreeSpaceHeuristic>(motions,1.2,2.5)),std::invalid_argument);
}
