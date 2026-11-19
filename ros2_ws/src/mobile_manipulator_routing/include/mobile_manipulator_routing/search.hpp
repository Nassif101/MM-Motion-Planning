#pragma once

#include <memory>
#include <optional>
#include <string>
#include "mobile_manipulator_routing/footprint.hpp"
#include "mobile_manipulator_routing/heuristic.hpp"
#include "mobile_manipulator_routing/primitives.hpp"

namespace mobile_manipulator_routing
{
enum class SearchStatus { Pending, Found, InvalidStart, InvalidGoal, NoPath, BudgetExhausted, Canceled, Stale };
enum class PathKind { Translating, Stationary };
const char * to_string(SearchStatus status);
struct SearchRequest { Pose2 start, goal; Revisions revisions; };
struct SearchConfiguration
{
  std::size_t max_expansions = 1000000;
  std::size_t max_memory_bytes = 256u << 20;
  std::size_t max_path_poses = 100000;
  int connector_radius_cells = 2;
  double non_straight_penalty = 1.2, cost_weight = 1.0, rotation_cost_m_per_rad = 2.5;
  double max_point_step_m = 0.0125;
};
struct SearchStatistics
{
  std::size_t expansions = 0, cell_tests = 0, generated = 0, peak_storage_bytes = 0;
};
struct SearchResult
{
  SearchStatus status = SearchStatus::Pending;
  PathKind kind = PathKind::Translating;
  Revisions revisions;
  Pose2 requested_goal, resolved_goal;
  std::vector<Pose2> poses;
  SearchStatistics statistics;
  double cost = 0.0, length_m = 0.0;
  std::string reason;
  // Exhaustion is relative to this finite connector policy, not continuous-space impossibility.
  static constexpr const char * connector_policy = "rotate_translate_rotate_r2_v1";
};
// The lattice primitives' swept masks for one footprint, interpolation step and grid geometry (size, resolution,
// origin; not occupancy), built on first use and shared by a session's searches. Calls are serialized by the owner.
class SweptMaskBank
{
public:
  SweptMaskBank(std::shared_ptr<const GridSnapshot> geometry, std::shared_ptr<const PrimitiveSet> primitives,
                Polygon footprint, double max_point_step_m = 0.0125);
  const std::shared_ptr<const SweptMask> & mask(std::size_t primitive);
  // the same inputs a search would build its own masks from
  bool serves(const GridSnapshot & grid, const PrimitiveSet & primitives, const Polygon & footprint,
              double max_point_step_m) const;
  std::size_t storage_bytes() const;
private:
  std::shared_ptr<const GridSnapshot> geometry_;
  std::shared_ptr<const PrimitiveSet> primitives_;
  Polygon footprint_;
  double step_;
  FootprintChecker checker_;
  std::vector<std::shared_ptr<const SweptMask>> masks_;
};

class Search
{
public:
  // `masks`, `heuristic` (optional): shared banks for these inputs (std::invalid_argument otherwise); without them
  // the search builds its own. The heuristic is max(Euclidean distance, the free-space table's bound).
  Search(std::shared_ptr<const GridSnapshot> grid, std::shared_ptr<const PrimitiveSet> primitives,
         Polygon footprint, SearchRequest request, SearchConfiguration configuration = {},
         std::shared_ptr<SweptMaskBank> masks = nullptr, std::shared_ptr<FreeSpaceHeuristic> heuristic = nullptr);
  ~Search();
  Search(Search &&) noexcept;
  Search & operator=(Search &&) noexcept;
  Search(const Search &) = delete;
  Search & operator=(const Search &) = delete;
  const SearchResult & advance(WorkBudget budget = {}, const std::function<bool()> & canceled = {},
                              const std::optional<Revisions> & current_revisions = std::nullopt);
  const SearchResult & result() const;
  void cancel();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace mobile_manipulator_routing
