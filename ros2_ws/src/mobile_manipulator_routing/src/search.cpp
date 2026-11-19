#include "mobile_manipulator_routing/search.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

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
      c.max_path_poses > 100000 || c.connector_radius_cells < 0)
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
    if (smac.plan().empty()) { finish(SearchStatus::NoPath, "the planner returned an empty plan"); return; }
    if (smac.plan().size() > planner->configuration.max_path_poses) {
      finish(SearchStatus::BudgetExhausted, "the plan exceeds the pose limit");
      return;
    }
    start_check(smac.smoothed());
  }

  void accept()
  {
    const auto & poses = output.poses;
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
        outcome = planner->smac->begin(planner->costmap, request.start.x, request.start.y, request.start.yaw,
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
    if (checking_smoothed) { start_check(false); return output; }
    finish(SearchStatus::Rejected, planner->smac->smoothed() ?
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
