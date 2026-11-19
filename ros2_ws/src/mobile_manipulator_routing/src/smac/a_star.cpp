// Copyright (c) 2020, Samsung Research America; Copyright (c) 2020, Applied Electric Vehicles Pty Ltd;
// Copyright (c) 2021, Samsung Research America. Licensed under the Apache License, Version 2.0; see
// third_party/NOTICE.md. Ported from navigation2 1.3.13 nav2_smac_planner a_star.cpp and analytic_expansion.cpp.
#include "mobile_manipulator_routing/smac/a_star.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mobile_manipulator_routing::smac
{
AnalyticExpansion::AnalyticExpansion(const SearchInfo & search_info, const bool & traverse_unknown,
                                     LatticeContext * context)
: _search_info(search_info), _traverse_unknown(traverse_unknown), _context(context)
{
}

AnalyticExpansion::NodePtr AnalyticExpansion::tryAnalyticExpansion(const NodePtr & current_node,
                                                                   const NodePtr & goal_node,
                                                                   const NodeGetter & getter,
                                                                   int & analytic_iterations,
                                                                   int & closest_distance)
{
  auto & motion_table = _context->motion_table;
  const Coordinates node_coords = NodeLattice::getCoords(current_node->getIndex(),
                                                         _collision_checker->getCostmap()->getSizeInCellsX(),
                                                         motion_table.num_angle_quantization);
  closest_distance = std::min(closest_distance,
                              static_cast<int>(NodeLattice::getHeuristicCost(*_context, node_coords,
                                                                             goal_node->pose)));
  int desired_iterations = std::max(static_cast<int>(closest_distance / _search_info.analytic_expansion_ratio),
                                    static_cast<int>(std::ceil(_search_info.analytic_expansion_ratio)));
  analytic_iterations = std::min(analytic_iterations, desired_iterations);

  if (analytic_iterations <= 0) {
    analytic_iterations = desired_iterations;
    AnalyticExpansionNodes analytic_nodes = getAnalyticPath(current_node, goal_node, getter,
                                                            *motion_table.state_space);
    if (!analytic_nodes.empty()) {
      NodePtr node = current_node;
      NodePtr test_node = current_node;
      AnalyticExpansionNodes refined_analytic_nodes;
      for (int i = 0; i < 8; i++) {
        if (test_node->parent && test_node->parent->parent && test_node->parent->parent->parent &&
            test_node->parent->parent->parent->parent && test_node->parent->parent->parent->parent->parent)
        {
          test_node = test_node->parent->parent->parent->parent->parent;
          refined_analytic_nodes = getAnalyticPath(test_node, goal_node, getter, *motion_table.state_space);
          if (refined_analytic_nodes.empty()) {
            break;
          }
          analytic_nodes = refined_analytic_nodes;
          node = test_node;
        } else {
          break;
        }
      }

      auto scoringFn = [&](const AnalyticExpansionNodes & expansion) {
          if (expansion.size() < 2) {
            return std::numeric_limits<float>::max();
          }
          float score = 0.0;
          float normalized_cost = 0.0;
          const float distance = hypotf(expansion[1].proposed_coords.x - expansion[0].proposed_coords.x,
                                        expansion[1].proposed_coords.y - expansion[0].proposed_coords.y);
          const float & weight = motion_table.cost_penalty;
          for (auto iter = expansion.begin(); iter != expansion.end(); ++iter) {
            normalized_cost = iter->node->getCost() / 252.0f;
            score += distance * (1.0 + weight * normalized_cost);
          }
          return score;
        };

      float best_score = scoringFn(analytic_nodes);
      float score = std::numeric_limits<float>::max();
      float min_turn_rad = motion_table.min_turning_radius;
      const float max_min_turn_rad = 4.0 * min_turn_rad;
      while (min_turn_rad < max_min_turn_rad) {
        min_turn_rad += 0.5;
        const DubinsStateSpace state_space(min_turn_rad);
        refined_analytic_nodes = getAnalyticPath(node, goal_node, getter, state_space);
        score = scoringFn(refined_analytic_nodes);
        if (score <= best_score) {
          analytic_nodes = refined_analytic_nodes;
          best_score = score;
        }
      }
      return setAnalyticPath(node, goal_node, analytic_nodes);
    }
  }
  analytic_iterations--;
  return NodePtr(nullptr);
}

AnalyticExpansion::AnalyticExpansionNodes AnalyticExpansion::getAnalyticPath(const NodePtr & node,
                                                                              const NodePtr & goal,
                                                                              const NodeGetter & node_getter,
                                                                              const DubinsStateSpace & state_space)
{
  auto & motion_table = _context->motion_table;
  DubinsStateSpace::State from, to, s;
  from.x = node->pose.x;
  from.y = node->pose.y;
  from.yaw = motion_table.getAngleFromBin(node->pose.theta);
  to.x = goal->pose.x;
  to.y = goal->pose.y;
  to.yaw = motion_table.getAngleFromBin(goal->pose.theta);

  float d = state_space.distance(from, to);
  static const float sqrt_2 = sqrtf(2.0f);
  if (d > _search_info.analytic_expansion_max_length || d < sqrt_2) {
    return AnalyticExpansionNodes();
  }
  unsigned int num_intervals = static_cast<unsigned int>(std::floor(d / sqrt_2));

  AnalyticExpansionNodes possible_nodes;
  possible_nodes.reserve(num_intervals);
  double theta;
  NodePtr prev(node);
  uint64_t index = 0;
  NodePtr next(nullptr);
  float angle = 0.0;
  Coordinates proposed_coordinates;
  bool failure = false;
  std::vector<float> node_costs;
  node_costs.reserve(num_intervals);

  for (float i = 1; i <= num_intervals; i++) {
    state_space.interpolate(from, to, i / num_intervals, s);
    const double reals[3] = {s.x, s.y, s.yaw};
    theta = (reals[2] < 0.0) ? (reals[2] + 2.0 * M_PI) : reals[2];
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    angle = motion_table.getAngle(theta);
    index = NodeLattice::getIndex(*_context, static_cast<unsigned int>(reals[0]), static_cast<unsigned int>(reals[1]),
                                  static_cast<unsigned int>(angle));
    if (node_getter(index, next)) {
      Coordinates initial_node_coords = next->pose;
      proposed_coordinates = {static_cast<float>(reals[0]), static_cast<float>(reals[1]), angle};
      next->setPose(proposed_coordinates);
      if (next->isNodeValid(_traverse_unknown, _collision_checker) && next != prev) {
        possible_nodes.emplace_back(next, initial_node_coords, proposed_coordinates);
        node_costs.emplace_back(next->getCost());
        prev = next;
      } else {
        next->setPose(initial_node_coords);
        failure = true;
        break;
      }
    } else {
      failure = true;
      break;
    }
  }

  if (!failure) {
    const float max_cost = _search_info.analytic_expansion_max_cost;
    auto max_cost_it = std::max_element(node_costs.begin(), node_costs.end());
    if (max_cost_it != node_costs.end() && *max_cost_it > max_cost) {
      bool cost_exit_high_cost_region = false;
      for (auto iter = node_costs.rbegin(); iter != node_costs.rend(); ++iter) {
        const float & curr_cost = *iter;
        if (curr_cost <= max_cost) {
          cost_exit_high_cost_region = true;
        } else if (curr_cost > max_cost && cost_exit_high_cost_region) {
          failure = true;
          break;
        }
      }
      if (failure) {
        if (d < 2.0f * M_PI * motion_table.min_turning_radius && _search_info.analytic_expansion_max_cost_override) {
          failure = false;
        }
      }
    }
  }

  for (const auto & node_pose : possible_nodes) {
    const auto & n = node_pose.node;
    n->setPose(node_pose.initial_coords);
  }
  if (failure) {
    return AnalyticExpansionNodes();
  }
  return possible_nodes;
}

AnalyticExpansion::NodePtr AnalyticExpansion::setAnalyticPath(const NodePtr & node, const NodePtr & goal_node,
                                                              const AnalyticExpansionNodes & expanded_nodes)
{
  _detached_nodes.clear();
  NodePtr prev = node;
  for (const auto & node_pose : expanded_nodes) {
    auto n = node_pose.node;
    n->setMotionPrimitive(nullptr);  // cleanNode
    if (n->getIndex() != goal_node->getIndex()) {
      if (n->wasVisited()) {
        _detached_nodes.push_back(std::make_unique<NodeLattice>(static_cast<uint64_t>(-1), _context));
        n = _detached_nodes.back().get();
      }
      n->parent = prev;
      n->pose = node_pose.proposed_coords;
      n->visited();
      prev = n;
    }
  }
  if (goal_node != prev) {
    goal_node->parent = prev;
    goal_node->setMotionPrimitive(nullptr);  // cleanNode
    goal_node->visited();
  }
  return goal_node;
}

AStarAlgorithm::AStarAlgorithm(const SearchInfo & search_info, LatticeContext * context)
: _search_info(search_info),
  _goal_coordinates(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN()),
  _context(context)
{
  _graph.reserve(100000);
}

void AStarAlgorithm::initialize(const bool & allow_unknown, int & max_iterations,
                                const int & max_on_approach_iterations, const float & lookup_table_size,
                                const unsigned int & dim_3_size)
{
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
  if (!_is_initialized) {
    NodeLattice::precomputeDistanceHeuristic(*_context, lookup_table_size, dim_3_size, _search_info);
  }
  _is_initialized = true;
  _dim3_size = dim_3_size;
  _expander = std::make_unique<AnalyticExpansion>(_search_info, _traverse_unknown, _context);
}

void AStarAlgorithm::setCollisionChecker(GridCollisionChecker * collision_checker)
{
  _collision_checker = collision_checker;
  _costmap = collision_checker->getCostmap();
  unsigned int x_size = _costmap->getSizeInCellsX();
  unsigned int y_size = _costmap->getSizeInCellsY();
  clearGraph();
  _active = false;
  if (_x_size != x_size || _y_size != y_size) {
    _x_size = x_size;
    _y_size = y_size;
    _context->motion_table.initMotionModel(_x_size, _search_info);
  }
  _expander->setCollisionChecker(_collision_checker);
}

AStarAlgorithm::NodePtr AStarAlgorithm::addToGraph(const uint64_t & index)
{
  auto iter = _graph.find(index);
  if (iter != _graph.end()) {
    return &(iter->second);
  }
  return &(_graph.emplace(std::piecewise_construct, std::forward_as_tuple(index),
                          std::forward_as_tuple(index, _context)).first->second);
}

void AStarAlgorithm::setStart(const float & mx, const float & my, const unsigned int & dim_3)
{
  _start = addToGraph(NodeLattice::getIndex(*_context, static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
                                            dim_3));
  _start->setPose(Coordinates(mx, my, dim_3));
}

void AStarAlgorithm::setGoal(const float & mx, const float & my, const unsigned int & dim_3)
{
  _goal = addToGraph(NodeLattice::getIndex(*_context, static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
                                           dim_3));
  Coordinates goal_coords(mx, my, dim_3);
  if (!_search_info.cache_obstacle_heuristic || goal_coords != _goal_coordinates) {
    if (!_start) {
      throw std::runtime_error("Start must be set before goal.");
    }
    NodeLattice::resetObstacleHeuristic(*_context, _collision_checker->getCostmap(), _start->pose.x, _start->pose.y,
                                        mx, my);
  }
  _goal_coordinates = goal_coords;
  _goal->setPose(_goal_coordinates);
}

bool AStarAlgorithm::areInputsValid()
{
  if (_graph.empty()) {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }
  if (!_start || !_goal) {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }
  if (_tolerance < 0.001 && !_goal->isNodeValid(_traverse_unknown, _collision_checker)) {
    throw std::runtime_error("Goal was in lethal cost");  // nav2_core::GoalOccupied; unreachable with tolerance
  }
  return true;
}

void AStarAlgorithm::beginPath(const float & tolerance)
{
  _tolerance = tolerance;
  _best_heuristic_node = {std::numeric_limits<float>::max(), 0};
  clearQueue();
  _path.clear();
  areInputsValid();
  addNode(0.0, _start);
  _start->setAccumulatedCost(0.0);
  _iterations = 0;
  _approach_iterations = 0;
  _analytic_iterations = 0;
  _closest_distance = std::numeric_limits<int>::max();
  const uint64_t max_index = static_cast<uint64_t>(_x_size) * static_cast<uint64_t>(_y_size) *
    static_cast<uint64_t>(_dim3_size);
  _neighbor_getter = [this, max_index](const uint64_t & index, NodePtr & neighbor_rtn) -> bool {
      if (index >= max_index) {
        return false;
      }
      neighbor_rtn = addToGraph(index);
      return true;
    };
  _active = true;
}

AStarAlgorithm::Progress AStarAlgorithm::advance(int iterations_budget)
{
  if (!_active) {
    throw std::logic_error("advance() without an active search");
  }
  NodePtr current_node = nullptr;
  NodePtr neighbor = nullptr;
  NodePtr expansion_result = nullptr;
  float g_cost = 0.0;
  NodeVector neighbors;
  const long long stop_at = static_cast<long long>(_iterations) + std::max(iterations_budget, 0);

  while (_iterations < _max_iterations && !_queue.empty()) {
    if (_iterations >= stop_at) {  // the only addition to upstream's loop: pause before the next pop
      return Progress::Running;
    }
    current_node = getNextNode();
    if (current_node->wasVisited()) {  // onVisitationCheckNode
      continue;
    }
    _iterations++;
    current_node->visited();

    expansion_result = nullptr;
    expansion_result = _expander->tryAnalyticExpansion(current_node, _goal, _neighbor_getter, _analytic_iterations,
                                                       _closest_distance);
    if (expansion_result != nullptr) {
      current_node = expansion_result;
    }

    if (isGoal(current_node)) {
      _active = false;
      return current_node->backtracePath(_path) ? Progress::Found : Progress::Failed;
    } else if (_best_heuristic_node.first < _tolerance) {
      _approach_iterations++;
      if (_approach_iterations >= _max_on_approach_iterations) {
        _active = false;
        return _graph.at(_best_heuristic_node.second).backtracePath(_path) ? Progress::Found : Progress::Failed;
      }
    }

    neighbors.clear();
    current_node->getNeighbors(_neighbor_getter, _collision_checker, _traverse_unknown, neighbors);
    for (auto neighbor_iterator = neighbors.begin(); neighbor_iterator != neighbors.end(); ++neighbor_iterator) {
      neighbor = *neighbor_iterator;
      g_cost = current_node->getAccumulatedCost() + current_node->getTraversalCost(neighbor);
      if (g_cost < neighbor->getAccumulatedCost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;
        addNode(g_cost + getHeuristicCost(neighbor), neighbor);
      }
    }
  }

  _active = false;
  if (_best_heuristic_node.first < _tolerance) {
    return _graph.at(_best_heuristic_node.second).backtracePath(_path) ? Progress::Found : Progress::Failed;
  }
  return Progress::Failed;
}

AStarAlgorithm::NodePtr AStarAlgorithm::getNextNode()
{
  NodeBasic node = _queue.top().second;
  _queue.pop();
  node.processSearchNode();
  return node.graph_node_ptr;
}

void AStarAlgorithm::addNode(const float & cost, NodePtr & node)
{
  NodeBasic queued_node(node->getIndex());
  queued_node.populateSearchNode(node);
  _queue.emplace(cost, queued_node);
}

float AStarAlgorithm::getHeuristicCost(const NodePtr & node)
{
  const Coordinates node_coords = NodeLattice::getCoords(node->getIndex(), _x_size, _dim3_size);
  float heuristic = NodeLattice::getHeuristicCost(*_context, node_coords, _goal_coordinates);
  if (heuristic < _best_heuristic_node.first) {
    _best_heuristic_node = {heuristic, node->getIndex()};
  }
  return heuristic;
}
}  // namespace mobile_manipulator_routing::smac
