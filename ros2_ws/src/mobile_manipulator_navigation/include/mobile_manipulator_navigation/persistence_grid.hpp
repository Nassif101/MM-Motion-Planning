#pragma once
// Persistence filter for live obstacles in the global costmap (no ROS dependencies).
//
// A cell becomes an obstacle only after it has been observed for at least
// `persistence_s`, with no gap between observations longer than `max_gap_s`; a longer gap
// starts the count again. Once confirmed it stays an obstacle until it has not been
// observed for `decay_s`. A worker walking past occupies any one cell for well under a
// second and is never confirmed, so the global route ignores it and the local costmap
// handles it; a box, or a worker who stops in the way, is confirmed and the route changes.
#include <unordered_map>
#include <vector>

namespace mobile_manipulator_navigation
{
struct PersistenceParameters
{
  double persistence_s = 2.0;
  double max_gap_s = 1.0;
  double decay_s = 10.0;
};

class PersistenceGrid
{
public:
  explicit PersistenceGrid(PersistenceParameters parameters = {}) : parameters_(parameters) {}

  // Records an observation of a cell at time t (seconds).
  void observe(unsigned index, double t)
  {
    auto [it, inserted] = cells_.try_emplace(index, Cell{t, t, false});
    Cell & cell = it->second;
    if (!inserted) {
      if (!cell.confirmed && t - cell.last > parameters_.max_gap_s) cell.first = t;  // streak broken
      if (t > cell.last) cell.last = t;
    }
    if (!cell.confirmed && cell.last - cell.first >= parameters_.persistence_s) cell.confirmed = true;
  }

  bool obstacle(unsigned index, double now) const
  {
    const auto it = cells_.find(index);
    return it != cells_.end() && it->second.confirmed && now - it->second.last <= parameters_.decay_s;
  }

  // Forgets expired cells: unconfirmed ones after max_gap_s, confirmed ones after
  // decay_s. Returns the confirmed cells that stopped being obstacles.
  std::vector<unsigned> prune(double now)
  {
    std::vector<unsigned> cleared;
    for (auto it = cells_.begin(); it != cells_.end();) {
      const double age = now - it->second.last;
      if (it->second.confirmed ? age > parameters_.decay_s : age > parameters_.max_gap_s) {
        if (it->second.confirmed) cleared.push_back(it->first);
        it = cells_.erase(it);
      } else {
        ++it;
      }
    }
    return cleared;
  }

  std::vector<unsigned> obstacles(double now) const
  {
    std::vector<unsigned> out;
    for (const auto & [index, cell] : cells_) {
      if (cell.confirmed && now - cell.last <= parameters_.decay_s) out.push_back(index);
    }
    return out;
  }

  size_t tracked() const { return cells_.size(); }
  void clear() { cells_.clear(); }

private:
  struct Cell
  {
    double first, last;
    bool confirmed;
  };

  PersistenceParameters parameters_;
  std::unordered_map<unsigned, Cell> cells_;
};
}  // namespace mobile_manipulator_navigation
