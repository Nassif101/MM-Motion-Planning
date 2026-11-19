#include "mobile_manipulator_routing/search.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory_resource>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace mobile_manipulator_routing
{
namespace
{
constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
constexpr double kPoseTolerance = 1e-10;

std::size_t sum_bytes(std::size_t a, std::size_t b)
{
  if (b > std::numeric_limits<std::size_t>::max()-a) throw std::bad_alloc();
  return a+b;
}

// Counts bytes requested by the PMR containers. External covers the immutable inputs,
// checker and a conservative reserved query workspace, not allocator/OS RSS overhead.
class StorageResource final : public std::pmr::memory_resource
{
public:
  explicit StorageResource(std::size_t limit) : limit_(limit) {}
  void external(std::size_t bytes)
  {
    if (bytes > limit_ || used_ > limit_-bytes) throw std::bad_alloc();
    external_ = bytes;
    peak_ = std::max(peak_,external_+used_);
  }
  std::size_t peak() const { return peak_; }
private:
  void * do_allocate(std::size_t bytes, std::size_t alignment) override
  {
    if (bytes > limit_-external_ || used_ > limit_-external_-bytes) throw std::bad_alloc();
    void * result = std::pmr::new_delete_resource()->allocate(bytes,alignment);
    used_ += bytes;
    peak_ = std::max(peak_,external_+used_);
    return result;
  }
  void do_deallocate(void * pointer, std::size_t bytes, std::size_t alignment) override
  {
    std::pmr::new_delete_resource()->deallocate(pointer,bytes,alignment);
    used_ -= bytes;
  }
  bool do_is_equal(const std::pmr::memory_resource & other) const noexcept override { return this == &other; }
  std::size_t limit_, used_ = 0, external_ = 0, peak_ = 0;
};

std::vector<Pose2> connector(Pose2 from, Pose2 to)
{
  std::vector<Pose2> result;
  result.reserve(4);
  result.push_back(from);
  const double length = std::hypot(to.x-from.x,to.y-from.y);
  if (length > kPoseTolerance) {
    const double heading = std::atan2(to.y-from.y,to.x-from.x);
    if (std::abs(wrap_angle(heading-from.yaw)) > kPoseTolerance)
      result.push_back({from.x,from.y,heading});
    result.push_back({to.x,to.y,heading});
  }
  if (result.back().x != to.x || result.back().y != to.y ||
      std::abs(wrap_angle(to.yaw-result.back().yaw)) > kPoseTolerance)
    result.push_back(to);
  else result.back() = to;
  return result;
}

enum class EntryKind { DirectGoal, Goal, Seed, Node };
struct QueueEntry
{
  double f = 0.0, g = 0.0, source_g = 0.0;
  std::uint64_t key = 0, serial = 0;
  std::size_t node = kNone;
  EntryKind kind = EntryKind::Node;
};
struct Later
{
  bool operator()(const QueueEntry & a, const QueueEntry & b) const
  {
    if (a.f != b.f) return a.f > b.f;
    if (a.g != b.g) return a.g < b.g;  // prefer progress at equal lower bound
    if (a.kind != b.kind) return a.kind > b.kind;
    if (a.key != b.key) return a.key > b.key;
    return a.serial > b.serial;
  }
};
struct Node
{
  std::uint64_t key;
  double g;
  std::size_t parent, incoming;
  bool closed = false;
};
enum class Phase { Start, Goal, Schedule, Graph };
enum class Purpose { Start, Goal, Seed, Primitive, GoalEdge, DirectGoal };
struct PendingEdge
{
  Purpose purpose;
  std::uint64_t key = 0;
  double g = 0.0;
  std::size_t parent = kNone, primitive = kNone;
};
struct Expansion
{
  std::size_t node, next = 0;
  double g;
  bool goal_scheduled = false;
};
}  // namespace

struct Search::Impl
{
  std::shared_ptr<const GridSnapshot> grid;
  std::shared_ptr<const PrimitiveSet> primitives;
  SearchRequest request;
  SearchConfiguration configuration;
  SearchResult output;
  StorageResource storage;
  std::pmr::vector<Node> nodes;
  std::pmr::unordered_map<std::uint64_t,std::size_t> index;
  std::pmr::vector<QueueEntry> queue;
  std::unique_ptr<FootprintChecker> checker;
  std::optional<MotionCheck> check;
  // Each primitive's swept cells from a lattice state: the shared bank's or this search's own (built on first use);
  // each mask is charged to the storage limit when this search first uses it
  std::shared_ptr<SweptMaskBank> bank;
  std::vector<bool> charged;
  // the free-space table's bound for the goal's heading bin, less the rotation cost of the goal yaw's offset from it
  std::shared_ptr<FreeSpaceHeuristic> table;
  std::size_t goal_bin = 0;
  double goal_bin_gap = 0.0;
  PendingEdge pending{Purpose::Start};
  std::optional<Expansion> expansion;
  Phase phase = Phase::Start;
  CellIndex start_cell{0,0}, goal_cell{0,0};
  std::size_t seed_cursor = 0, external_bytes = 0, query_allowance = 0;
  bool direct_scheduled = false;
  std::uint64_t serial = 0;

  Impl(std::shared_ptr<const GridSnapshot> map, std::shared_ptr<const PrimitiveSet> motions,
       Polygon footprint, SearchRequest query, SearchConfiguration config, std::shared_ptr<SweptMaskBank> masks,
       std::shared_ptr<FreeSpaceHeuristic> heuristic)
  : grid(std::move(map)), primitives(std::move(motions)), request(query), configuration(config),
    storage(config.max_memory_bytes), nodes(&storage), index(&storage), queue(&storage), bank(std::move(masks)),
    table(std::move(heuristic))
  {
    if (!grid || !primitives) throw std::invalid_argument("search needs immutable grid and primitive inputs");
    if (bank && !bank->serves(*grid,*primitives,footprint,configuration.max_point_step_m))
      throw std::invalid_argument("the shared swept-mask bank does not match this search's inputs");
    if (configuration.connector_radius_cells != 2)
      throw std::invalid_argument("only rotate_translate_rotate_r2_v1 connectors are supported");
    if (configuration.max_expansions == 0 || configuration.max_path_poses == 0 ||
        configuration.max_path_poses > 100000 || !std::isfinite(configuration.non_straight_penalty) ||
        configuration.non_straight_penalty < 1.0 || !std::isfinite(configuration.cost_weight) ||
        configuration.cost_weight < 0.0 || !std::isfinite(configuration.rotation_cost_m_per_rad) ||
        configuration.rotation_cost_m_per_rad <= 0.0 || !std::isfinite(configuration.max_point_step_m) ||
        configuration.max_point_step_m <= 0.0)
      throw std::invalid_argument("invalid search limits, cost weights or interpolation step");
    validate_primitives();
    output.revisions = request.revisions;
    output.requested_goal = request.goal;
    output.resolved_goal = request.goal;
    if (request.revisions.map && request.revisions.map != grid->revision()) {
      finish(SearchStatus::Stale,"request map revision does not match its snapshot");
      return;
    }
    request.revisions.map = grid->revision();
    output.revisions = request.revisions;
    if (!finite(request.start)) { finish(SearchStatus::InvalidStart,"nonfinite start pose"); return; }
    if (!finite(request.goal)) { finish(SearchStatus::InvalidGoal,"nonfinite goal pose"); return; }
    request.start.yaw = wrap_angle(request.start.yaw);
    request.goal.yaw = wrap_angle(request.goal.yaw);
    const auto start = grid->world_to_cell({request.start.x,request.start.y});
    const auto goal = grid->world_to_cell({request.goal.x,request.goal.y});
    if (!start) { finish(SearchStatus::InvalidStart,"start is outside map"); return; }
    if (!goal) { finish(SearchStatus::InvalidGoal,"goal is outside map"); return; }
    start_cell = *start; goal_cell = *goal;
    if (table && !table->serves(*primitives,configuration.non_straight_penalty,configuration.rotation_cost_m_per_rad))
      throw std::invalid_argument("the shared heuristic does not match this search's primitives or costs");
    if (!table) table = std::make_shared<FreeSpaceHeuristic>(primitives,configuration.non_straight_penalty,
                                                             configuration.rotation_cost_m_per_rad);
    goal_bin = primitives->nearest_heading(request.goal.yaw);
    goal_bin_gap = configuration.rotation_cost_m_per_rad*
                   std::abs(wrap_angle(request.goal.yaw-primitives->headings[goal_bin]));
    try {
      if (!bank) bank = std::make_shared<SweptMaskBank>(grid,primitives,footprint,configuration.max_point_step_m);
      checker = std::make_unique<FootprintChecker>(grid,std::move(footprint));
      std::size_t maximum_poses = 4;
      for (const auto & primitive : primitives->motions)
        maximum_poses = std::max(maximum_poses,primitive.poses.size()+1);
      // MotionCheck owns its raw poses, two polygon buffers and a small cursor. This
      // reserve also covers temporary connector/primitive vectors before moving them in.
      query_allowance = sum_bytes(4096,maximum_poses*sizeof(Pose2));
      query_allowance = sum_bytes(query_allowance,checker->storage_bytes()*2);
      external_bytes = sum_bytes(sizeof(Impl),grid->storage_bytes());
      external_bytes = sum_bytes(external_bytes,primitives->storage_bytes());
      external_bytes = sum_bytes(external_bytes,checker->storage_bytes());
      external_bytes = sum_bytes(external_bytes,query_allowance);
      // the goal bin's table (shared or own), whether or not this search built it
      const std::size_t side = static_cast<std::size_t>(2*table->radius_cells()+1);
      external_bytes = sum_bytes(external_bytes,side*side*primitives->headings.size()*sizeof(float));
      storage.external(external_bytes);
    } catch (const std::bad_alloc &) { memory_failure(); }
    update_statistics();
  }

  void validate_primitives() const
  {
    if (!std::isfinite(primitives->resolution) ||
        std::abs(primitives->resolution-grid->resolution()) > 1e-9 ||
        primitives->headings.empty() || primitives->headings.size() > 256 ||
        primitives->successors.size() != primitives->headings.size() || primitives->motions.empty() ||
        primitives->motions.size() > 65536)
      throw std::invalid_argument("primitive bank is invalid or does not match map resolution");
    for (std::size_t i = 0; i < primitives->headings.size(); ++i) {
      if (!std::isfinite(primitives->headings[i])) throw std::invalid_argument("nonfinite primitive heading");
      for (std::size_t j = 0; j < i; ++j)
        if (std::abs(wrap_angle(primitives->headings[i]-primitives->headings[j])) < 1e-10)
          throw std::invalid_argument("duplicate primitive heading");
    }
    std::bitset<65536> referenced;
    std::size_t references = 0;
    for (std::size_t h = 0; h < primitives->successors.size(); ++h) {
      if (primitives->successors[h].empty()) throw std::invalid_argument("heading has no successors");
      for (auto motion : primitives->successors[h]) {
        if (motion >= primitives->motions.size() || primitives->motions[motion].start_heading != h)
          throw std::invalid_argument("invalid primitive successor index");
        if (referenced[motion]) throw std::invalid_argument("duplicate primitive successor");
        referenced.set(motion); ++references;
      }
    }
    if (references != primitives->motions.size()) throw std::invalid_argument("unreferenced primitive motion");
    for (const auto & motion : primitives->motions) {
      if (motion.start_heading >= primitives->headings.size() || motion.end_heading >= primitives->headings.size() ||
          motion.poses.size() < 2 || motion.poses.size() > 4096 || !std::isfinite(motion.length_m) ||
          motion.length_m < 0.0 || !std::isfinite(motion.angular_travel) || motion.angular_travel < 0.0)
        throw std::invalid_argument("invalid primitive geometry or bounded sample count");
      for (const auto & pose : motion.poses)
        if (!finite(pose)) throw std::invalid_argument("nonfinite primitive sample");
      const auto & first = motion.poses.front();
      const auto & last = motion.poses.back();
      const auto roundoff = [](double a, double b) {
        return 64.0*std::numeric_limits<double>::epsilon()*std::max({1.0,std::abs(a),std::abs(b)});
      };
      if (std::hypot(first.x,first.y) > 1e-9 ||
          std::abs(wrap_angle(first.yaw-primitives->headings[motion.start_heading])) > 1e-6 ||
          std::abs(last.x-motion.dx*grid->resolution()) > roundoff(last.x,motion.dx*grid->resolution()) ||
          std::abs(last.y-motion.dy*grid->resolution()) > roundoff(last.y,motion.dy*grid->resolution()) ||
          std::abs(wrap_angle(last.yaw-primitives->headings[motion.end_heading])) > 1e-6 ||
          (motion.length_m == 0.0 && motion.angular_travel == 0.0))
        throw std::invalid_argument("primitive endpoints do not match declared lattice state");
      double sampled_length = 0.0, angular = 0.0;
      bool stationary = true;
      for (std::size_t i = 0; i < motion.poses.size(); ++i) {
        const auto & at = motion.poses[i];
        stationary = stationary && std::hypot(at.x,at.y) <= 1e-9;
        if (i) {
          const auto & previous = motion.poses[i-1];
          const double dx = at.x-previous.x, dy = at.y-previous.y;
          const double turn = wrap_angle(at.yaw-previous.yaw);
          const double heading = previous.yaw+turn*0.5;
          if (dx*std::cos(heading)+dy*std::sin(heading) < -1e-5)
            throw std::invalid_argument("reverse primitive translation is not supported");
          sampled_length += std::hypot(dx,dy);
          angular += std::abs(turn);
        }
      }
      if (!std::isfinite(sampled_length) || !std::isfinite(angular) ||
          motion.length_m+roundoff(motion.length_m,sampled_length) < sampled_length ||
          motion.length_m+roundoff(motion.length_m,std::hypot(last.x,last.y)) < std::hypot(last.x,last.y) ||
          std::abs(angular-motion.angular_travel) > 1e-8*std::max(1.0,angular) ||
          stationary != motion.pure_rotation || (angular > 1e-10) != motion.turning)
        throw std::invalid_argument("primitive length, angular travel or motion flags are inconsistent");
    }
  }

  void update_statistics() { output.statistics.peak_storage_bytes = storage.peak(); }
  void finish(SearchStatus status, const char * reason)
  {
    output.status = status;
    output.reason = reason;
    update_statistics();
  }
  void memory_failure()
  {
    output.poses.clear();
    output.length_m = output.cost = 0.0;
    finish(SearchStatus::BudgetExhausted,"routing storage limit or allocation failure");
  }

  std::uint64_t key(int x, int y, std::size_t heading) const
  {
    return (static_cast<std::uint64_t>(y)*static_cast<std::uint64_t>(grid->width())+
      static_cast<std::uint64_t>(x))*primitives->headings.size()+heading;
  }
  CellIndex cell(std::uint64_t value) const
  {
    const auto linear = value/primitives->headings.size();
    return {static_cast<int>(linear%static_cast<std::uint64_t>(grid->width())),
      static_cast<int>(linear/static_cast<std::uint64_t>(grid->width()))};
  }
  Pose2 pose(std::uint64_t value) const
  {
    const auto at = cell(value);
    const auto xy = grid->cell_center(at.x,at.y);
    return {xy.x,xy.y,primitives->headings[value%primitives->headings.size()]};
  }
  bool near_goal(std::uint64_t value) const
  {
    const auto at = cell(value);
    return std::abs(at.x-goal_cell.x) <= 2 && std::abs(at.y-goal_cell.y) <= 2;
  }
  double heuristic(std::uint64_t value) const
  {
    const auto at = pose(value);
    const auto c = cell(value);
    const double euclidean = std::hypot(at.x-request.goal.x,at.y-request.goal.y);
    const double turning = table->bound(c.x-goal_cell.x,c.y-goal_cell.y,value%primitives->headings.size(),goal_bin)-
                           goal_bin_gap;
    return std::max(euclidean,turning);
  }

  double path_cost(const std::vector<Pose2> & path, double length_floor = 0.0, bool primitive_turn = false) const
  {
    double length = 0.0, soft = 0.0, rotation = 0.0;
    bool moving_turn = primitive_turn;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const auto & a = path[i-1]; const auto & b = path[i];
      const double segment = std::hypot(b.x-a.x,b.y-a.y);
      const double turn = std::abs(wrap_angle(b.yaw-a.yaw));
      if (segment <= 1e-12) { rotation += turn; continue; }
      length += segment;
      moving_turn = moving_turn || turn > 1e-10;
      // Fixed, bounded midpoint quadrature defines this implementation's soft cost.
      // Primitive samples are already short; connector quadrature is capped at 64.
      const double requested = std::ceil(segment/(grid->resolution()*0.5));
      const auto count = static_cast<std::size_t>(std::max(1.0,std::min(64.0,requested)));
      for (std::size_t sample = 0; sample < count; ++sample) {
        const double t = (static_cast<double>(sample)+0.5)/static_cast<double>(count);
        const auto at = grid->world_to_cell({a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t});
        soft += segment/static_cast<double>(count)*(at ? grid->cost(at->x,at->y) : 1.0);
      }
    }
    const double cost = std::max(length,length_floor)*(moving_turn ? configuration.non_straight_penalty : 1.0) +
      configuration.cost_weight*soft + configuration.rotation_cost_m_per_rad*rotation;
    if (!std::isfinite(cost) || cost < 0.0) throw std::invalid_argument("search cost is not finite under configured weights");
    return cost;
  }

  void push(QueueEntry entry)
  {
    if (!std::isfinite(entry.f) || !std::isfinite(entry.g))
      throw std::invalid_argument("search cost/heuristic overflow");
    entry.serial = serial++;
    queue.push_back(entry);
    std::push_heap(queue.begin(),queue.end(),Later{});
  }

  void relax(const PendingEdge & edge)
  {
    const auto existing = index.find(edge.key);
    std::size_t at;
    if (existing == index.end()) {
      at = nodes.size();
      nodes.push_back({edge.key,edge.g,edge.parent,edge.primitive,false});
      index.emplace(edge.key,at);
      ++output.statistics.generated;
    } else {
      at = existing->second;
      if (!(edge.g < nodes[at].g)) return;
      nodes[at].g = edge.g; nodes[at].parent = edge.parent;
      nodes[at].incoming = edge.primitive; nodes[at].closed = false;
    }
    push({edge.g+heuristic(edge.key),edge.g,0.0,edge.key,0,at,EntryKind::Node});
  }

  std::vector<Pose2> primitive_path(std::size_t source, std::size_t primitive) const
  {
    const auto from = pose(nodes[source].key);
    const auto & motion = primitives->motions[primitive];
    std::vector<Pose2> path;
    path.reserve(motion.poses.size()+1);
    for (const auto & at : motion.poses) path.push_back({from.x+at.x,from.y+at.y,at.yaw});
    // Only floating-point roundoff is accepted at lattice endpoints. Set the checked
    // final sample to the canonical state; do not append a lateral correction motion.
    const auto target_cell = cell(nodes[source].key);
    const auto to = grid->cell_center(target_cell.x+motion.dx,target_cell.y+motion.dy);
    const Pose2 exact{to.x,to.y,primitives->headings[motion.end_heading]};
    path.back() = exact;
    return path;
  }

  const std::shared_ptr<const SweptMask> & mask(std::size_t primitive)
  {
    if (charged.empty()) charged.resize(primitives->motions.size(),false);
    const auto & built = bank->mask(primitive);
    if (!charged[primitive]) {
      external_bytes = sum_bytes(external_bytes,built->storage_bytes());
      storage.external(external_bytes);
      charged[primitive] = true;
    }
    return built;
  }

  void begin_check(PendingEdge edge, std::vector<Pose2> path)
  {
    pending = edge;
    check.emplace(checker->motion(std::move(path),configuration.max_point_step_m));
    if (check->storage_bytes() > query_allowance)
      throw std::bad_alloc();
  }

  void materialize(std::size_t goal_parent, double cost)
  {
    std::pmr::vector<std::size_t> chain(&storage);
    for (auto at = goal_parent; at != kNone; at = nodes[at].parent) {
      if (chain.size() >= nodes.size()) throw std::logic_error("route parent cycle");
      chain.push_back(at);
    }
    std::reverse(chain.begin(),chain.end());
    const auto walk = [&](const auto & emit) {
      const auto visit = [&](const std::vector<Pose2> & path) { for (const auto & at : path) emit(at); };
      if (chain.empty()) visit(connector(request.start,request.goal));
      else {
        visit(connector(request.start,pose(nodes[chain.front()].key)));
        for (std::size_t i = 1; i < chain.size(); ++i)
          visit(primitive_path(chain[i-1],nodes[chain[i]].incoming));
        visit(connector(pose(nodes[chain.back()].key),request.goal));
      }
    };
    std::size_t capacity = 0;
    std::optional<Pose2> previous;
    walk([&](const Pose2 & at) {
      if (!previous || at.x != previous->x || at.y != previous->y ||
          std::abs(wrap_angle(at.yaw-previous->yaw)) > 1e-12) ++capacity;
      previous = at;
    });
    if (capacity > configuration.max_path_poses) {
      finish(SearchStatus::BudgetExhausted,"raw path pose limit exceeded");
      return;
    }
    storage.external(sum_bytes(external_bytes,capacity*sizeof(Pose2)));
    output.poses.reserve(capacity);
    walk([&](const Pose2 & at) {
      if (!output.poses.empty() && at.x == output.poses.back().x && at.y == output.poses.back().y &&
          std::abs(wrap_angle(at.yaw-output.poses.back().yaw)) <= 1e-12) output.poses.back() = at;
      else output.poses.push_back(at);
    });
    output.kind = PathKind::Stationary;
    for (std::size_t i = 1; i < output.poses.size(); ++i) {
      const double length = std::hypot(output.poses[i].x-output.poses[i-1].x,
        output.poses[i].y-output.poses[i-1].y);
      output.length_m += length;
      if (length > kPoseTolerance) output.kind = PathKind::Translating;
    }
    output.cost = cost;
    finish(SearchStatus::Found,"validated route in configured lattice/connector graph");
  }

  void finish_check(CheckStatus status)
  {
    check.reset();
    if (status == CheckStatus::Canceled) { finish(SearchStatus::Canceled,"search canceled"); return; }
    if (pending.purpose == Purpose::Start) {
      if (status != CheckStatus::Clear) finish(SearchStatus::InvalidStart,"start footprint is blocked");
      else phase = Phase::Goal;
    } else if (pending.purpose == Purpose::Goal) {
      if (status != CheckStatus::Clear) finish(SearchStatus::InvalidGoal,"goal footprint is blocked");
      else phase = Phase::Schedule;
    } else if (status == CheckStatus::Clear) {
      if (pending.purpose == Purpose::Seed || pending.purpose == Purpose::Primitive) relax(pending);
      else materialize(pending.parent,pending.g);
    }
  }

  void schedule_one()
  {
    if (!direct_scheduled) {
      direct_scheduled = true;
      if (std::abs(start_cell.x-goal_cell.x) <= 2 && std::abs(start_cell.y-goal_cell.y) <= 2) {
        const double g = path_cost(connector(request.start,request.goal));
        push({g,g,0.0,0,0,kNone,EntryKind::DirectGoal});
      }
      return;
    }
    const std::size_t total = 25*primitives->headings.size();
    if (seed_cursor >= total) { phase = Phase::Graph; return; }
    const std::size_t candidate = seed_cursor++;
    const auto heading = candidate%primitives->headings.size();
    const auto offset = candidate/primitives->headings.size();
    const int x = start_cell.x+static_cast<int>(offset%5)-2;
    const int y = start_cell.y+static_cast<int>(offset/5)-2;
    if (!grid->contains(x,y)) return;
    const auto destination = key(x,y,heading);
    const double g = path_cost(connector(request.start,pose(destination)));
    push({g+heuristic(destination),g,0.0,destination,0,kNone,EntryKind::Seed});
  }

  void expansion_one()
  {
    auto & active = *expansion;
    const auto source = active.node;
    const auto source_key = nodes[source].key;
    const auto & successors = primitives->successors[source_key%primitives->headings.size()];
    if (active.next < successors.size()) {
      const auto primitive = successors[active.next++];
      const auto & motion = primitives->motions[primitive];
      const auto from = cell(source_key);
      const auto x = static_cast<std::int64_t>(from.x)+motion.dx;
      const auto y = static_cast<std::int64_t>(from.y)+motion.dy;
      if (x < 0 || y < 0 || x >= grid->width() || y >= grid->height()) return;
      const auto target = key(static_cast<int>(x),static_cast<int>(y),motion.end_heading);
      auto path = primitive_path(source,primitive);
      const double g = active.g+path_cost(path,motion.length_m,motion.turning && !motion.pure_rotation);
      const auto existing = index.find(target);
      if (existing != index.end() && !(g < nodes[existing->second].g)) return;
      pending = {Purpose::Primitive,target,g,source,primitive};
      check.emplace(checker->masked(mask(primitive),from));
      return;
    }
    if (!active.goal_scheduled) {
      active.goal_scheduled = true;
      if (near_goal(source_key)) {
        const double g = active.g+path_cost(connector(pose(source_key),request.goal));
        push({g,g,active.g,source_key,0,source,EntryKind::Goal});
      }
      return;
    }
    expansion.reset();
  }

  void pop_one()
  {
    if (queue.empty()) {
      finish(SearchStatus::NoPath,"configured lattice/connector graph exhausted");
      return;
    }
    std::pop_heap(queue.begin(),queue.end(),Later{});
    const auto entry = queue.back(); queue.pop_back();
    if (entry.kind == EntryKind::DirectGoal) {
      begin_check({Purpose::DirectGoal,0,entry.g},connector(request.start,request.goal));
      return;
    }
    if (entry.kind == EntryKind::Seed) {
      const auto existing = index.find(entry.key);
      if (existing != index.end() && !(entry.g < nodes[existing->second].g)) return;
      begin_check({Purpose::Seed,entry.key,entry.g},connector(request.start,pose(entry.key)));
      return;
    }
    if (entry.kind == EntryKind::Goal) {
      if (nodes[entry.node].g != entry.source_g) return;
      begin_check({Purpose::GoalEdge,entry.key,entry.g,entry.node},connector(pose(entry.key),request.goal));
      return;
    }
    auto & node = nodes[entry.node];
    if (node.closed || node.g != entry.g) return;
    if (output.statistics.expansions >= configuration.max_expansions) {
      finish(SearchStatus::BudgetExhausted,"node expansion limit reached");
      return;
    }
    node.closed = true;
    ++output.statistics.expansions;
    expansion = Expansion{entry.node,0,entry.g,false};
  }

  const SearchResult & advance(WorkBudget budget, const std::function<bool()> & canceled,
                              std::optional<Revisions> current)
  {
    if (output.status != SearchStatus::Pending) return output;
    if (current) {
      if (current->map == 0) current->map = grid->revision();
      if (*current != request.revisions) {
        finish(SearchStatus::Stale,"input revisions changed");
        return output;
      }
    }
    if (canceled && canceled()) { finish(SearchStatus::Canceled,"search canceled"); return output; }
    if (budget.expansions == 0 || budget.cell_tests == 0) return output;
    // `expansions` counts node expansions (the plan's slice unit); every other search operation (seed scheduling,
    // a successor's set-up, a stale heap entry) costs one unit of the cell budget, so a slice stays bounded
    std::size_t expansions = budget.expansions;
    try {
      while (output.status == SearchStatus::Pending) {
        if (canceled && canceled()) { finish(SearchStatus::Canceled,"search canceled"); break; }
        if (check) {
          const auto before = budget.cell_tests;
          const auto status = check->advance(budget.cell_tests,canceled);
          output.statistics.cell_tests += before-budget.cell_tests;
          if (status == CheckStatus::Pending) break;
          finish_check(status);
          continue;
        }
        if (budget.cell_tests == 0) break;
        if (phase == Phase::Start) begin_check({Purpose::Start},{request.start});
        else if (phase == Phase::Goal) begin_check({Purpose::Goal},{request.goal});
        else {
          --budget.cell_tests;
          if (phase == Phase::Schedule) schedule_one();
          else if (expansion) expansion_one();
          else {
            if (expansions == 0) { ++budget.cell_tests; break; }  // the next pop may expand: not this slice
            const auto before = output.statistics.expansions;
            pop_one();
            if (output.statistics.expansions > before) --expansions;
          }
        }
      }
    } catch (const std::bad_alloc &) { memory_failure(); }
    update_statistics();
    return output;
  }
};

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
  }
  return "unknown";
}

SweptMaskBank::SweptMaskBank(std::shared_ptr<const GridSnapshot> geometry, std::shared_ptr<const PrimitiveSet> primitives,
                             Polygon footprint, double max_point_step_m)
: geometry_(std::move(geometry)), primitives_(std::move(primitives)), footprint_(footprint), step_(max_point_step_m),
  checker_(geometry_,std::move(footprint))
{
  if (!primitives_) throw std::invalid_argument("swept-mask bank needs primitives");
  masks_.resize(primitives_->motions.size());
}

const std::shared_ptr<const SweptMask> & SweptMaskBank::mask(std::size_t primitive)
{
  auto & slot = masks_.at(primitive);
  if (!slot) {
    // the primitive's poses relative to its start cell's centre, the last on the exact lattice state (primitive_path)
    const auto & motion = primitives_->motions[primitive];
    std::vector<Pose2> relative(motion.poses.begin(),motion.poses.end());
    relative.back() = {motion.dx*geometry_->resolution(),motion.dy*geometry_->resolution(),
                       primitives_->headings[motion.end_heading]};
    slot = std::make_shared<const SweptMask>(checker_.swept_mask(std::move(relative),step_));
  }
  return slot;
}

bool SweptMaskBank::serves(const GridSnapshot & grid, const PrimitiveSet & primitives, const Polygon & footprint,
                           double max_point_step_m) const
{
  const auto same_point = [](const Point2 & a, const Point2 & b) { return a.x == b.x && a.y == b.y; };
  return (&primitives == primitives_.get()) && max_point_step_m == step_ && grid.width() == geometry_->width() &&
         grid.height() == geometry_->height() && grid.resolution() == geometry_->resolution() &&
         same_point(grid.origin(),geometry_->origin()) && footprint.size() == footprint_.size() &&
         std::equal(footprint.begin(),footprint.end(),footprint_.begin(),same_point);
}

std::size_t SweptMaskBank::storage_bytes() const
{
  std::size_t bytes = sizeof(*this)+masks_.capacity()*sizeof(masks_[0]);
  for (const auto & m : masks_) if (m) bytes += m->storage_bytes();
  return bytes;
}

Search::Search(std::shared_ptr<const GridSnapshot> grid, std::shared_ptr<const PrimitiveSet> primitives,
               Polygon footprint, SearchRequest request, SearchConfiguration configuration,
               std::shared_ptr<SweptMaskBank> masks, std::shared_ptr<FreeSpaceHeuristic> heuristic)
: impl_(std::make_unique<Impl>(std::move(grid),std::move(primitives),std::move(footprint),request,configuration,
                               std::move(masks),std::move(heuristic))) {}
Search::~Search() = default;
Search::Search(Search &&) noexcept = default;
Search & Search::operator=(Search &&) noexcept = default;
const SearchResult & Search::advance(WorkBudget budget, const std::function<bool()> & canceled,
                                     const std::optional<Revisions> & current_revisions)
{
  if (!impl_) throw std::logic_error("moved-from search");
  return impl_->advance(budget,canceled,current_revisions);
}
const SearchResult & Search::result() const
{
  if (!impl_) throw std::logic_error("moved-from search");
  return impl_->output;
}
void Search::cancel()
{
  if (!impl_) throw std::logic_error("moved-from search");
  if (impl_->output.status == SearchStatus::Pending) impl_->finish(SearchStatus::Canceled,"search canceled");
}
}  // namespace mobile_manipulator_routing
