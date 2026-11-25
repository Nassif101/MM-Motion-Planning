#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <vector>
#include "mobile_manipulator_routing/grid.hpp"
namespace mobile_manipulator_routing
{
enum class CheckStatus { Pending, Clear, Blocked, Canceled };
// A lattice motion's swept cells (plan Task 3). Lattice states sit at cell centres, so the motion's sampled footprints
// (motion()'s interpolation, dilation and closed-contact test) touch the same cells relative to the start cell
// wherever the motion starts. The mask keeps those offsets once; a translated check is one grid lookup per offset.
struct SweptMask
{
  std::vector<CellIndex> offsets;  // unique, sorted by (y, x)
  // The union of the samples' dilated envelopes, relative to the start cell's centre: touching the map's exterior
  // blocks, as for motion()
  double left = 0.0, right = 0.0, bottom = 0.0, top = 0.0;
  std::size_t storage_bytes() const { return sizeof(*this)+offsets.capacity()*sizeof(CellIndex); }
};
class MotionCheck
{
public:
  ~MotionCheck();
  MotionCheck(MotionCheck &&) noexcept;
  MotionCheck & operator=(MotionCheck &&) noexcept;
  MotionCheck(const MotionCheck &) = delete;
  MotionCheck & operator=(const MotionCheck &) = delete;
  // Consumes one unit per tested cell, including clear cells. Can stop inside a sweep.
  CheckStatus advance(std::size_t & cell_budget, const std::function<bool()> & canceled = {});
  std::size_t storage_bytes() const;
  // After Blocked: the occupied or unknown grid cell that blocked the sweep; none when the envelope met the map's
  // exterior, or before Blocked (route refusal retry, 2026-11-25).
  std::optional<CellIndex> blocking_cell() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  explicit MotionCheck(std::unique_ptr<Impl> impl);
  friend class FootprintChecker;
};
class FootprintChecker
{
public:
  FootprintChecker(std::shared_ptr<const GridSnapshot> grid, Polygon footprint);
  MotionCheck motion(std::vector<Pose2> poses, double max_point_step_m = 0.0125) const;
  // `relative`: the motion's poses with positions relative to the start cell's centre (absolute headings). The
  // dilation's rounding envelope uses this checker's map extents, so a mask is valid for grids of the same geometry.
  SweptMask swept_mask(std::vector<Pose2> relative, double max_point_step_m = 0.0125) const;
  // The mask translated to start at cell `start`'s centre; one budget unit per offset; resumable like motion().
  MotionCheck masked(std::shared_ptr<const SweptMask> mask, CellIndex start) const;
  bool pose_clear(const Pose2 & pose) const;
  double radius() const { return radius_; }
  std::size_t storage_bytes() const;
private:
  std::shared_ptr<const GridSnapshot> grid_;
  Polygon footprint_;
  double radius_ = 0.0;
};
}  // namespace mobile_manipulator_routing
