#include "mobile_manipulator_routing/search.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mobile_manipulator_routing
{
using Outcome = smac::SmacPlannerLattice::Outcome;

const char * to_string(SearchStatus status)
{
  switch (status) {
    case SearchStatus::Pending: return "pending";
    case SearchStatus::Found: return "found";
    case SearchStatus::InvalidStart: return "invalid_start";
    case SearchStatus::InvalidGoal: return "invalid_goal";
    case SearchStatus::NoPath: return "no_path";
    case SearchStatus::BudgetExhausted: return "budget_exhausted";
    case SearchStatus::Canceled: return "canceled";
    case SearchStatus::Stale: return "stale";
    case SearchStatus::Rejected: return "rejected";
  }
  return "unknown";
}

namespace
{
std::shared_ptr<const GridSnapshot> unknown_as_free(const GridSnapshot & grid)
{
  auto known = grid.known();
  auto observed = grid.observed();
  for (auto & c : known) if (c == Cell::Unknown) c = Cell::Free;
  for (auto & c : observed) if (c == Cell::Unknown) c = Cell::Free;
  return std::make_shared<const GridSnapshot>(grid.width(), grid.height(), grid.resolution(), grid.origin(),
                                              std::move(known), std::move(observed), InflationParameters{},
                                              grid.revision());
}

void validate(const SearchConfiguration & c)
{
  if (c.lattice.lattice_filepath.empty()) throw std::invalid_argument("the route planner needs a lattice file");
  if (c.lattice.max_iterations <= 0 || !std::isfinite(c.lattice.tolerance) || c.lattice.tolerance < 0.0f ||
      !std::isfinite(c.max_point_step_m) || c.max_point_step_m <= 0.0 || c.max_path_poses == 0 ||
      c.max_path_poses > 100000 || c.connector_radius_cells < 0 || c.refusal_retries < 0)
    throw std::invalid_argument("invalid route planner limits");
}
}  // namespace

RoutePlanner::RoutePlanner(std::shared_ptr<const GridSnapshot> grid_in, const smac::Footprint & smac_footprint,
                           Polygon validation_footprint_in, SearchConfiguration configuration_in)
: grid(std::move(grid_in)), validation_footprint(std::move(validation_footprint_in)),
  configuration(std::move(configuration_in))
{
  if (!grid) throw std::invalid_argument("the route planner needs a grid");
  validate(configuration);
  costmap = std::make_shared<const smac::LayeredCostmap>(smac::build_costmap(*grid, smac_footprint,
                                                                             configuration.costmap));
  validation_grid = unknown_as_free(*grid);
  FootprintChecker check(validation_grid, validation_footprint);  // validates the polygon
  (void)check;
  smac = std::make_shared<smac::SmacPlannerLattice>(configuration.lattice, costmap->costmap.getResolution());
}

struct Search::Impl
{
  std::shared_ptr<RoutePlanner> planner;
  SearchRequest request;
  SearchResult output;
  bool started = false;
  std::optional<FootprintChecker> checker;
  std::optional<MotionCheck> check;
  bool checking_smoothed = false;
  // Route refusal retry: the costmap Smac searches (the planner's, or a copy marked around refusing cells), the cells
  // that blocked this attempt's checks, the first refusal's reason
  std::shared_ptr<const smac::LayeredCostmap> costmap;
  std::vector<CellIndex> blocking;
  std::string refusal;
  std::optional<Pose2> first_end;  // where the first attempt's route ended: a re-plan must end there too (review I1)

  void finish(SearchStatus status, std::string reason)
  {
    output.status = status;
    output.reason = std::move(reason);
    check.reset();
  }

  static std::vector<Pose2> poses_of(const smac::Plan & plan)
  {
    std::vector<Pose2> out;
    out.reserve(plan.size());
    for (const auto & p : plan) out.push_back({p.x, p.y, smac::getYaw(p.orientation)});
    return out;
  }

  void note_blocking()
  {
    if (check && check->blocking_cell()) blocking.push_back(*check->blocking_cell());
  }

  // After a refusal: true when a re-plan starts (on the next advance), false when the refusal stands
  bool retry()
  {
    if (blocking.empty() || static_cast<int>(output.statistics.refusal_retries) >= planner->configuration.refusal_retries)
      return false;
    auto marked = std::make_shared<smac::LayeredCostmap>(*costmap);
    auto & cm = marked->costmap;
    const auto & grid = *planner->grid;
    std::size_t changed = 0;
    // the blocking cell and its 8-neighbours, each below lethal raised to lethal (occupied and no-information cells
    // are left as they are); Smac's obstacle heuristic stays the unmarked costmap's (cached, as upstream)
    for (const auto & cell : blocking)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          if (!grid.contains(cell.x + dx, cell.y + dy)) continue;
          const auto centre = grid.cell_center(cell.x + dx, cell.y + dy);
          unsigned int mx = 0, my = 0;
          if (!cm.worldToMap(centre.x, centre.y, mx, my)) continue;
          if (cm.getCost(mx, my) < smac::kLethalObstacle) {
            cm.setCost(mx, my, smac::kLethalObstacle);
            ++changed;
          }
        }
    if (changed == 0) return false;  // review M2: nothing new to avoid, so a re-plan would repeat the last attempt
    if (!first_end && !output.poses.empty()) first_end = output.poses.back();
    costmap = std::move(marked);
    blocking.clear();
    ++output.statistics.refusal_retries;
    started = false;
    check.reset();
    output.poses.clear();
    output.smoothed = false;
    return true;
  }

  void refuse(std::string reason)
  {
    const bool first = refusal.empty();
    if (first) refusal = reason;
    if (retry()) return;
    const auto n = output.statistics.refusal_retries;
    // review M1: the re-plans and how the last one ended
    finish(SearchStatus::Rejected, n == 0 ? refusal : refusal + "; re-planned " + std::to_string(n) +
           (n == 1 ? " time" : " times") + " around the refused segments without a validated route (the last: " +
           (first ? refusal : reason) + ")");
  }

  void start_check(bool smoothed)
  {
    checking_smoothed = smoothed;
    const auto & plan = smoothed ? planner->smac->plan() : planner->smac->raw_plan();
    output.poses = poses_of(plan);
    output.smoothed = smoothed;
    if (!checker) checker.emplace(planner->validation_grid, planner->validation_footprint);
    check.emplace(checker->motion(output.poses, planner->configuration.max_point_step_m));
  }

  void found()
  {
    const auto & smac = *planner->smac;
    const bool replan = output.statistics.refusal_retries > 0;  // a re-plan's failure keeps the refusal
    if (smac.plan().empty()) {
      if (replan) refuse("the re-plan returned an empty plan");
      else finish(SearchStatus::NoPath, "the planner returned an empty plan");
      return;
    }
    if (smac.plan().size() > planner->configuration.max_path_poses) {
      if (replan) refuse("the re-plan exceeds the pose limit");
      else finish(SearchStatus::BudgetExhausted, "the plan exceeds the pose limit");
      return;
    }
    start_check(smac.smoothed());
  }

  void accept()
  {
    const auto & poses = output.poses;
    if (first_end && (std::hypot(poses.back().x - first_end->x, poses.back().y - first_end->y) > 1e-9 ||
                      std::abs(std::remainder(poses.back().yaw - first_end->yaw, 2.0 * M_PI)) > 1e-9)) {
      // review I1: the marks made the goal pose invalid for Smac, which returned another node within its tolerance
      refuse("the re-planned route ends elsewhere than the first attempt's");
      return;
    }
    output.resolved_goal = poses.back();
    output.length_m = 0.0;
    bool translating = false;
    for (std::size_t i = 1; i < poses.size(); ++i) {
      const double d = std::hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y);
      output.length_m += d;
      translating = translating || d > 1e-12;
    }
    output.kind = translating ? PathKind::Translating : PathKind::Stationary;
    output.status = SearchStatus::Found;
    output.reason = output.smoothed ? "smoothed plan validated" :
      (planner->smac->smoothed() ? "smoothed plan failed validation; raw plan validated" : "raw plan validated");
    if (output.statistics.refusal_retries > 0)
      output.reason += " after " + std::to_string(output.statistics.refusal_retries) + " re-plans around refused segments";
    check.reset();
  }

  void update_statistics()
  {
    output.statistics.expansions = static_cast<std::size_t>(std::max(planner->smac->iterations(), 0));
    output.statistics.generated = planner->smac->graph_size();
    output.statistics.peak_queue = std::max(output.statistics.peak_queue, planner->smac->queue_size());
  }

  const SearchResult & advance(WorkBudget budget, const std::function<bool()> & canceled,
                              std::optional<Revisions> current)
  {
    if (output.status != SearchStatus::Pending) return output;
    if (current) {
      if (current->map == 0) current->map = planner->grid->revision();
      if (*current != request.revisions) { finish(SearchStatus::Stale, "input revisions changed"); return output; }
    }
    if (canceled && canceled()) { finish(SearchStatus::Canceled, "search canceled"); return output; }
    if (budget.expansions == 0 || budget.cell_tests == 0) return output;

    if (!check) {
      Outcome outcome;
      if (!started) {
        started = true;
        if (!costmap) costmap = planner->costmap;
        outcome = planner->smac->begin(costmap, request.start.x, request.start.y, request.start.yaw,
                                       request.goal.x, request.goal.y, request.goal.yaw);
      } else {
        outcome = planner->smac->outcome();
      }
      if (outcome == Outcome::Pending) {
        const auto iterations = static_cast<int>(std::min<std::size_t>(budget.expansions,
                                                                       std::numeric_limits<int>::max()));
        outcome = planner->smac->advance(iterations);
        update_statistics();
        if (outcome == Outcome::Pending) return output;
      }
      update_statistics();
      if (outcome != Outcome::Found && outcome != Outcome::Pending && output.statistics.refusal_retries > 0) {
        refuse(std::string("the re-plan ended: ") + smac::to_string(outcome));  // keeps the first refusal's reason
        return output;
      }
      switch (outcome) {
        case Outcome::Found: found(); break;
        case Outcome::StartOutsideMapBounds:
        case Outcome::StartOccupied: finish(SearchStatus::InvalidStart, smac::to_string(outcome)); return output;
        case Outcome::GoalOutsideMapBounds: finish(SearchStatus::InvalidGoal, smac::to_string(outcome)); return output;
        case Outcome::NoValidPathCouldBeFound: finish(SearchStatus::NoPath, smac::to_string(outcome)); return output;
        case Outcome::PlannerTimedOut: finish(SearchStatus::BudgetExhausted, smac::to_string(outcome)); return output;
        case Outcome::Pending: return output;
      }
      if (output.status != SearchStatus::Pending) return output;
    }

    std::size_t cells = budget.cell_tests;
    const auto before = cells;
    const auto status = check->advance(cells, canceled);
    output.statistics.cell_tests += before - cells;
    if (status == CheckStatus::Pending) return output;
    if (status == CheckStatus::Canceled) { finish(SearchStatus::Canceled, "search canceled"); return output; }
    if (status == CheckStatus::Clear) { accept(); return output; }
    note_blocking();
    if (checking_smoothed) { start_check(false); return output; }
    refuse(planner->smac->smoothed() ?
           "the smoothed and the raw plan failed the swept validation" : "the plan failed the swept validation");
    return output;
  }
};

Search::Search(std::shared_ptr<RoutePlanner> planner, SearchRequest request)
: impl_(std::make_unique<Impl>())
{
  if (!planner) throw std::invalid_argument("a search needs a route planner");
  impl_->planner = std::move(planner);
  impl_->request = request;
  auto & output = impl_->output;
  output.requested_goal = request.goal;
  output.resolved_goal = request.goal;
  if (request.revisions.map && request.revisions.map != impl_->planner->grid->revision()) {
    impl_->request.revisions = request.revisions;
    output.revisions = request.revisions;
    impl_->finish(SearchStatus::Stale, "request map revision does not match its snapshot");
    return;
  }
  impl_->request.revisions.map = impl_->planner->grid->revision();
  output.revisions = impl_->request.revisions;
  if (!finite(request.start)) { impl_->finish(SearchStatus::InvalidStart, "nonfinite start pose"); return; }
  if (!finite(request.goal)) { impl_->finish(SearchStatus::InvalidGoal, "nonfinite goal pose"); return; }
}

Search::~Search() = default;
Search::Search(Search &&) noexcept = default;
Search & Search::operator=(Search &&) noexcept = default;

const SearchResult & Search::advance(WorkBudget budget, const std::function<bool()> & canceled,
                                     const std::optional<Revisions> & current_revisions)
{
  return impl_->advance(budget, canceled, current_revisions);
}

const SearchResult & Search::result() const { return impl_->output; }

void Search::cancel()
{
  if (impl_->output.status == SearchStatus::Pending) impl_->finish(SearchStatus::Canceled, "search canceled");
}
}  // namespace mobile_manipulator_routing
