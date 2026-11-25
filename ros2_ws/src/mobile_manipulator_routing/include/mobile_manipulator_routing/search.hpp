#pragma once

#include <memory>
#include <optional>
#include <string>
#include "mobile_manipulator_routing/footprint.hpp"
#include "mobile_manipulator_routing/smac/planner.hpp"

namespace mobile_manipulator_routing
{
// Spec 2 Amendment 1 / ADR 0019: one route search is one createPlan of the Smac Lattice port, followed by the swept
// validation pass (A1.2 item 3). `Rejected` means Smac returned a route that failed validation, raw and smoothed, and
// the re-plans around the refusing cells (SearchConfiguration::refusal_retries) did not produce a validated one.
enum class SearchStatus { Pending, Found, InvalidStart, InvalidGoal, NoPath, BudgetExhausted, Canceled, Stale, Rejected };
enum class PathKind { Translating, Stationary };
const char * to_string(SearchStatus status);
struct SearchRequest { Pose2 start, goal; Revisions revisions; };
struct SearchConfiguration
{
  smac::LatticeSettings lattice;       // the Lattice block; lattice_filepath is set by the owner
  smac::CostmapSettings costmap;       // footprint padding and inflation of the global costmap
  double max_point_step_m = 0.0125;    // the validation pass's sampling step
  int connector_radius_cells = 2;      // the owner's adoption connector (Session)
  std::size_t max_path_poses = 100000;
  // Route refusal retry (2026-11-25): a route the validation refuses is planned again this many times, Smac's costmap
  // copied with the blocking cells' free 8-neighbours made lethal (cumulative); 0: the refusal stands at once.
  int refusal_retries = 3;
};
struct SearchStatistics
{
  std::size_t expansions = 0;   // Smac iterations
  std::size_t cell_tests = 0;   // validation work
  std::size_t generated = 0;    // graph nodes
  std::size_t peak_queue = 0;
  std::size_t refusal_retries = 0;  // the re-plans around refused segments this search ran
};
struct SearchResult
{
  SearchStatus status = SearchStatus::Pending;
  PathKind kind = PathKind::Translating;
  Revisions revisions;
  Pose2 requested_goal, resolved_goal;
  std::vector<Pose2> poses;  // the validated plan (smoothed, or raw when the smoothed plan failed validation)
  bool smoothed = false;     // whether `poses` is the smoothed plan
  SearchStatistics statistics;
  double length_m = 0.0;
  std::string reason;
  static constexpr const char * planner = "smac_lattice_1.3.13_port";
};

// The session's planner: one Smac Lattice port, serving its searches in order (A1.2 item 4).
struct RoutePlanner
{
  RoutePlanner(std::shared_ptr<const GridSnapshot> grid, const smac::Footprint & smac_footprint,
               Polygon validation_footprint, SearchConfiguration configuration);
  std::shared_ptr<const GridSnapshot> grid;
  std::shared_ptr<const smac::LayeredCostmap> costmap;
  std::shared_ptr<const GridSnapshot> validation_grid;  // `grid` with unknown cells free, as Nav2's costmap treats them
  Polygon validation_footprint;
  SearchConfiguration configuration;
  std::shared_ptr<smac::SmacPlannerLattice> smac;
};

class Search
{
public:
  Search(std::shared_ptr<RoutePlanner> planner, SearchRequest request);
  ~Search();
  Search(Search &&) noexcept;
  Search & operator=(Search &&) noexcept;
  Search(const Search &) = delete;
  Search & operator=(const Search &) = delete;
  // `budget.expansions` Smac iterations and `budget.cell_tests` validation cell tests per call.
  const SearchResult & advance(WorkBudget budget = {}, const std::function<bool()> & canceled = {},
                              const std::optional<Revisions> & current_revisions = std::nullopt);
  const SearchResult & result() const;
  void cancel();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace mobile_manipulator_routing
