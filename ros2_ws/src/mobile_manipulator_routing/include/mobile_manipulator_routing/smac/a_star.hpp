// Copyright (c) 2020, Samsung Research America; Copyright (c) 2020, Applied Electric Vehicles Pty Ltd.
// Licensed under the Apache License, Version 2.0; see third_party/NOTICE.md.
//
// Ported from navigation2 1.3.13 (f4108e5b) nav2_smac_planner: a_star.hpp/.cpp and analytic_expansion.hpp/.cpp, for
// NodeLattice. Spec 2 Amendment 1 / ADR 0019. The difference from upstream (A1.2 item 1): createPath's loop state is
// kept in the object, so the search can stop after a number of iterations and continue later with identical results;
// the wall-time limit and cancel checker are removed.
#pragma once
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mobile_manipulator_routing/smac/node_lattice.hpp"

namespace mobile_manipulator_routing::smac
{
class AnalyticExpansion
{
public:
  using NodePtr = NodeLattice *;
  using NodeGetter = NodeLattice::NodeGetter;
  struct AnalyticExpansionNode
  {
    AnalyticExpansionNode(NodePtr & node_in, Coordinates & initial_coords_in, Coordinates & proposed_coords_in)
    : node(node_in), initial_coords(initial_coords_in), proposed_coords(proposed_coords_in) {}
    NodePtr node;
    Coordinates initial_coords;
    Coordinates proposed_coords;
  };
  using AnalyticExpansionNodes = std::vector<AnalyticExpansionNode>;

  AnalyticExpansion(const SearchInfo & search_info, const bool & traverse_unknown, LatticeContext * context);
  void setCollisionChecker(GridCollisionChecker * collision_checker) { _collision_checker = collision_checker; }
  NodePtr tryAnalyticExpansion(const NodePtr & current_node, const NodePtr & goal_node, const NodeGetter & getter,
                               int & analytic_iterations, int & closest_distance);
  AnalyticExpansionNodes getAnalyticPath(const NodePtr & node, const NodePtr & goal, const NodeGetter & getter,
                                         const DubinsStateSpace & state_space);
  NodePtr setAnalyticPath(const NodePtr & node, const NodePtr & goal, const AnalyticExpansionNodes & expanded_nodes);

private:
  SearchInfo _search_info;
  bool _traverse_unknown;
  GridCollisionChecker * _collision_checker = nullptr;
  LatticeContext * _context;
  std::vector<std::unique_ptr<NodeLattice>> _detached_nodes;
};

class AStarAlgorithm
{
public:
  using NodePtr = NodeLattice *;
  using Graph = std::unordered_map<uint64_t, NodeLattice>;
  using NodeVector = std::vector<NodePtr>;
  using NodeElement = std::pair<float, NodeBasic>;
  struct NodeComparator
  {
    bool operator()(const NodeElement & a, const NodeElement & b) const { return a.first > b.first; }
  };
  using NodeQueue = std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator>;
  enum class Progress { Running, Found, Failed };

  AStarAlgorithm(const SearchInfo & search_info, LatticeContext * context);
  void initialize(const bool & allow_unknown, int & max_iterations, const int & max_on_approach_iterations,
                  const float & lookup_table_size, const unsigned int & dim_3_size);
  void setCollisionChecker(GridCollisionChecker * collision_checker);
  void setStart(const float & mx, const float & my, const unsigned int & dim_3);
  void setGoal(const float & mx, const float & my, const unsigned int & dim_3);
  // createPath, split: begin() runs everything before the loop; advance() runs up to `iterations` loop iterations
  // (upstream's iteration count) and returns Found or Failed when createPath would have returned.
  void beginPath(const float & tolerance);
  Progress advance(int iterations);
  const CoordinateVector & path() const { return _path; }
  int iterations() const { return _iterations; }
  int & getMaxIterations() { return _max_iterations; }
  std::size_t graph_size() const { return _graph.size(); }
  std::size_t queue_size() const { return _queue.size(); }

private:
  bool areInputsValid();
  bool isGoal(NodePtr & node) { return node == _goal; }
  NodePtr getNextNode();
  void addNode(const float & cost, NodePtr & node);
  NodePtr addToGraph(const uint64_t & index);
  float getHeuristicCost(const NodePtr & node);
  void clearQueue() { NodeQueue q; std::swap(_queue, q); }
  void clearGraph() { Graph g; std::swap(_graph, g); _graph.reserve(100000); }

  bool _traverse_unknown = true;
  bool _is_initialized = false;
  int _max_iterations = 0;
  int _max_on_approach_iterations = 0;
  unsigned int _x_size = 0, _y_size = 0, _dim3_size = 0;
  SearchInfo _search_info;
  Coordinates _goal_coordinates;
  NodePtr _start = nullptr, _goal = nullptr;
  Graph _graph;
  NodeQueue _queue;
  GridCollisionChecker * _collision_checker = nullptr;
  const Costmap * _costmap = nullptr;
  std::unique_ptr<AnalyticExpansion> _expander;
  LatticeContext * _context;

  // createPath's loop state
  float _tolerance = 0.0f;
  std::pair<float, uint64_t> _best_heuristic_node;
  int _iterations = 0;
  int _approach_iterations = 0;
  int _analytic_iterations = 0;
  int _closest_distance = 0;
  NodeLattice::NodeGetter _neighbor_getter;
  CoordinateVector _path;
  bool _active = false;
};
}  // namespace mobile_manipulator_routing::smac
