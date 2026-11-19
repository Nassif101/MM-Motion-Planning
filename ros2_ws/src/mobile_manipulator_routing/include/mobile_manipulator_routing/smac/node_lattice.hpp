// Copyright (c) 2021, Samsung Research America; Copyright (c) 2020, Samsung Research America (NodeHybrid's obstacle
// heuristic). Licensed under the Apache License, Version 2.0; see third_party/NOTICE.md.
//
// Ported from navigation2 1.3.13 (f4108e5b) nav2_smac_planner: types.hpp, constants.hpp, utils.hpp (lattice JSON),
// node_lattice.hpp/.cpp, node_hybrid.cpp (resetObstacleHeuristic, getObstacleHeuristic, distanceHeuristic2D) and
// node_basic.cpp (the Lattice specialization). Spec 2 Amendment 1 / ADR 0019.
//
// Upstream keeps the motion table, the distance lookup table and the obstacle heuristic in static members shared by
// every planner in the process. Here they live in one LatticeContext per planner, and every node keeps a pointer to
// it; the values and the order of operations are unchanged. Lattice never initializes NodeHybrid's motion table, so
// the obstacle heuristic runs with Hybrid's zero-initialized defaults: no downsampling, linear cost penalty.
#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mobile_manipulator_routing/smac/collision_checker.hpp"
#include "mobile_manipulator_routing/smac/dubins.hpp"

namespace mobile_manipulator_routing::smac
{
enum class MotionModel { UNKNOWN = 0, TWOD = 1, DUBIN = 2, REEDS_SHEPP = 3, STATE_LATTICE = 4, OMNI = 5 };
enum class TurnDirection { UNKNOWN = 0, FORWARD = 1, LEFT = 2, RIGHT = 3, REVERSE = 4, REV_LEFT = 5, REV_RIGHT = 6 };

struct SearchInfo
{
  float minimum_turning_radius{8.0};
  float non_straight_penalty{1.05};
  float change_penalty{0.0};
  float reverse_penalty{2.0};
  float cost_penalty{2.0};
  float retrospective_penalty{0.015};
  float rotation_penalty{5.0};
  float analytic_expansion_ratio{3.5};
  float analytic_expansion_max_length{60.0};
  float analytic_expansion_max_cost{200.0};
  bool analytic_expansion_max_cost_override{false};
  std::string lattice_filepath;
  bool cache_obstacle_heuristic{false};
  bool allow_reverse_expansion{false};
  bool allow_primitive_interpolation{false};
  bool downsample_obstacle_heuristic{true};
  bool use_quadratic_cost_penalty{false};
};

struct MotionPose
{
  MotionPose() {}
  MotionPose(const float & x, const float & y, const float & theta, const TurnDirection & turn_dir)
  : _x(x), _y(y), _theta(theta), _turn_dir(turn_dir) {}
  MotionPose operator-(const MotionPose & p2)
  {
    return MotionPose(this->_x - p2._x, this->_y - p2._y, this->_theta - p2._theta, TurnDirection::UNKNOWN);
  }
  float _x;
  float _y;
  float _theta;
  TurnDirection _turn_dir;
};
using MotionPoses = std::vector<MotionPose>;

struct LatticeMetadata
{
  float min_turning_radius;
  float grid_resolution;
  unsigned int number_of_headings;
  std::vector<float> heading_angles;
  unsigned int number_of_trajectories;
  std::string motion_model;
};

struct MotionPrimitive
{
  unsigned int trajectory_id;
  float start_angle;
  float end_angle;
  float turning_radius;
  float trajectory_length;
  float arc_length;
  float straight_length;
  bool left_turn;
  MotionPoses poses;
};
using MotionPrimitives = std::vector<MotionPrimitive>;
using MotionPrimitivePtrs = std::vector<MotionPrimitive *>;
using TrigValues = std::pair<double, double>;
using LookupTable = std::vector<float>;

// NodeHybrid::Coordinates, shared by Lattice.
struct Coordinates
{
  Coordinates() {}
  Coordinates(const float & x_in, const float & y_in, const float & theta_in) : x(x_in), y(y_in), theta(theta_in) {}
  inline bool operator==(const Coordinates & rhs) const
  {
    return this->x == rhs.x && this->y == rhs.y && this->theta == rhs.theta;
  }
  inline bool operator!=(const Coordinates & rhs) const { return !(*this == rhs); }
  float x, y, theta;
};
using CoordinateVector = std::vector<Coordinates>;

struct LatticeMotionTable
{
  void initMotionModel(unsigned int & size_x_in, SearchInfo & search_info);
  MotionPrimitivePtrs getMotionPrimitives(const class NodeLattice * node, unsigned int & direction_change_index);
  static LatticeMetadata getLatticeMetadata(const std::string & lattice_filepath);
  unsigned int getClosestAngularBin(const double & theta);
  float & getAngleFromBin(const unsigned int & bin_idx);
  double getAngle(const double & theta);

  unsigned int size_x = 0;
  unsigned int num_angle_quantization = 0;
  float change_penalty = 0;
  float non_straight_penalty = 0;
  float cost_penalty = 0;
  float reverse_penalty = 0;
  float travel_distance_reward = 0;
  float rotation_penalty = 0;
  float min_turning_radius = 0;
  bool allow_reverse_expansion = false;
  std::vector<std::vector<MotionPrimitive>> motion_primitives;
  std::shared_ptr<DubinsStateSpace> state_space;
  std::vector<TrigValues> trig_values;
  std::string current_lattice_filepath;
  LatticeMetadata lattice_metadata;
  MotionModel motion_model = MotionModel::UNKNOWN;
};

using ObstacleHeuristicElement = std::pair<float, uint64_t>;
struct ObstacleHeuristicComparator
{
  bool operator()(const ObstacleHeuristicElement & a, const ObstacleHeuristicElement & b) const
  {
    return a.first > b.first;
  }
};
using ObstacleHeuristicQueue = std::vector<ObstacleHeuristicElement>;

// The state upstream keeps in static members.
struct LatticeContext
{
  LatticeMotionTable motion_table;          // NodeLattice::motion_table
  float size_lookup = 25;                   // NodeLattice::size_lookup
  LookupTable dist_heuristic_lookup_table;  // NodeLattice::dist_heuristic_lookup_table
  // NodeHybrid's statics used by Lattice's obstacle heuristic
  LookupTable obstacle_heuristic_lookup_table;
  ObstacleHeuristicQueue obstacle_heuristic_queue;
  const Costmap * costmap = nullptr;        // NodeHybrid::costmap_ros->getCostmap()
  bool downsample_obstacle_heuristic = false;  // NodeHybrid::motion_table, never initialized by Lattice
  bool use_quadratic_cost_penalty = false;
};

class NodeLattice
{
public:
  using NodePtr = NodeLattice *;
  using NodeVector = std::vector<NodePtr>;
  using NodeGetter = std::function<bool(const uint64_t &, NodeLattice * &)>;

  NodeLattice(const uint64_t index, LatticeContext * context);
  void reset();
  float getAccumulatedCost() const { return _accumulated_cost; }
  void setAccumulatedCost(const float & cost_in) { _accumulated_cost = cost_in; }
  void setMotionPrimitive(MotionPrimitive * prim) { _motion_primitive = prim; }
  MotionPrimitive * getMotionPrimitive() const { return _motion_primitive; }
  float getCost() const { return _cell_cost; }
  bool wasVisited() const { return _was_visited; }
  void visited() { _was_visited = true; }
  uint64_t getIndex() const { return _index; }
  void backwards(bool back = true) { _backwards = back; }
  bool isBackward() const { return _backwards; }
  void setPose(const Coordinates & pose_in) { pose = pose_in; }

  bool isNodeValid(const bool & traverse_unknown, GridCollisionChecker * collision_checker,
                   MotionPrimitive * primitive = nullptr, bool is_backwards = false);
  float getTraversalCost(const NodePtr & child);

  static uint64_t getIndex(const unsigned int & x, const unsigned int & y, const unsigned int & angle,
                           const unsigned int & width, const unsigned int & angle_quantization)
  {
    return static_cast<uint64_t>(angle) + static_cast<uint64_t>(x) * static_cast<uint64_t>(angle_quantization) +
           static_cast<uint64_t>(y) * static_cast<uint64_t>(width) * static_cast<uint64_t>(angle_quantization);
  }
  static uint64_t getIndex(const LatticeContext & context, const unsigned int & x, const unsigned int & y,
                           const unsigned int & angle)
  {
    return getIndex(x, y, angle, context.motion_table.size_x, context.motion_table.num_angle_quantization);
  }
  static Coordinates getCoords(const uint64_t & index, const unsigned int & width,
                               const unsigned int & angle_quantization)
  {
    return Coordinates((index / angle_quantization) % width, index / (angle_quantization * width),
                       index % angle_quantization);
  }

  static float getHeuristicCost(LatticeContext & context, const Coordinates & node_coords,
                                const Coordinates & goal_coords);
  static void precomputeDistanceHeuristic(LatticeContext & context, const float & lookup_table_dim,
                                          const unsigned int & dim_3_size, const SearchInfo & search_info);
  static float getDistanceHeuristic(LatticeContext & context, const Coordinates & node_coords,
                                    const Coordinates & goal_coords, const float & obstacle_heuristic);
  // NodeHybrid::resetObstacleHeuristic / getObstacleHeuristic
  static void resetObstacleHeuristic(LatticeContext & context, const Costmap * costmap,
                                     const unsigned int & start_x, const unsigned int & start_y,
                                     const unsigned int & goal_x, const unsigned int & goal_y);
  static float getObstacleHeuristic(LatticeContext & context, const Coordinates & node_coords,
                                    const Coordinates & goal_coords, const float & cost_penalty);

  void getNeighbors(NodeGetter & validity_checker, GridCollisionChecker * collision_checker,
                    const bool & traverse_unknown, NodeVector & neighbors);
  bool backtracePath(CoordinateVector & path);
  void addNodeToPath(NodePtr current_node, CoordinateVector & path);

  NodeLattice * parent;
  Coordinates pose;
  LatticeContext * context;

private:
  float _cell_cost;
  float _accumulated_cost;
  uint64_t _index;
  bool _was_visited;
  MotionPrimitive * _motion_primitive;
  bool _backwards;
  bool _is_node_valid;
};

// NodeBasic<NodeLattice>: the queue's copy of a node's search state.
struct NodeBasic
{
  explicit NodeBasic(const uint64_t new_index) : graph_node_ptr(nullptr), index(new_index) {}
  void populateSearchNode(NodeLattice * & node);
  void processSearchNode();
  Coordinates pose;
  NodeLattice * graph_node_ptr;
  MotionPrimitive * prim_ptr = nullptr;
  uint64_t index;
  bool backward = false;
};

// angles::shortest_angular_distance (angles 1.16, as installed).
double shortest_angular_distance(double from, double to);
}  // namespace mobile_manipulator_routing::smac
