// Copyright (c) 2021, Samsung Research America; Copyright (c) 2020, Samsung Research America.
// Licensed under the Apache License, Version 2.0; see third_party/NOTICE.md.
// Ported from navigation2 1.3.13 nav2_smac_planner; the source files are named in node_lattice.hpp.
#include "mobile_manipulator_routing/smac/node_lattice.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace mobile_manipulator_routing::smac
{
namespace
{
// angles::normalize_angle (angles 1.16)
double normalize_angle(double angle)
{
  const double result = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (result <= 0.0) return result + M_PI;
  return result - M_PI;
}

// utils.hpp
void fromJsonToMetaData(const nlohmann::json & json, LatticeMetadata & lattice_metadata)
{
  json.at("turning_radius").get_to(lattice_metadata.min_turning_radius);
  json.at("grid_resolution").get_to(lattice_metadata.grid_resolution);
  json.at("num_of_headings").get_to(lattice_metadata.number_of_headings);
  json.at("heading_angles").get_to(lattice_metadata.heading_angles);
  json.at("number_of_trajectories").get_to(lattice_metadata.number_of_trajectories);
  json.at("motion_model").get_to(lattice_metadata.motion_model);
}

void fromJsonToPose(const nlohmann::json & json, MotionPose & pose)
{
  pose._x = json[0];
  pose._y = json[1];
  pose._theta = json[2];
}

void fromJsonToMotionPrimitive(const nlohmann::json & json, MotionPrimitive & motion_primitive)
{
  json.at("trajectory_id").get_to(motion_primitive.trajectory_id);
  json.at("start_angle_index").get_to(motion_primitive.start_angle);
  json.at("end_angle_index").get_to(motion_primitive.end_angle);
  json.at("trajectory_radius").get_to(motion_primitive.turning_radius);
  json.at("trajectory_length").get_to(motion_primitive.trajectory_length);
  json.at("arc_length").get_to(motion_primitive.arc_length);
  json.at("straight_length").get_to(motion_primitive.straight_length);
  json.at("left_turn").get_to(motion_primitive.left_turn);
  for (unsigned int i = 0; i < json["poses"].size(); i++) {
    MotionPose pose;
    fromJsonToPose(json["poses"][i], pose);
    motion_primitive.poses.push_back(pose);
  }
}

inline float distanceHeuristic2D(const uint64_t idx, const unsigned int size_x, const unsigned int target_x,
                                 const unsigned int target_y)
{
  int dx = static_cast<int>(idx % size_x) - static_cast<int>(target_x);
  int dy = static_cast<int>(idx / size_x) - static_cast<int>(target_y);
  return std::sqrt(dx * dx + dy * dy);
}
}  // namespace

double shortest_angular_distance(double from, double to) { return normalize_angle(to - from); }

void LatticeMotionTable::initMotionModel(unsigned int & size_x_in, SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  allow_reverse_expansion = search_info.allow_reverse_expansion;
  rotation_penalty = search_info.rotation_penalty;
  min_turning_radius = search_info.minimum_turning_radius;

  if (current_lattice_filepath == search_info.lattice_filepath) {
    return;
  }
  current_lattice_filepath = search_info.lattice_filepath;
  lattice_metadata = getLatticeMetadata(current_lattice_filepath);
  std::ifstream latticeFile(current_lattice_filepath);
  if (!latticeFile.is_open()) {
    throw std::runtime_error("Could not open lattice file");
  }
  nlohmann::json json;
  latticeFile >> json;
  num_angle_quantization = lattice_metadata.number_of_headings;

  if (!state_space) {
    if (lattice_metadata.motion_model == "omni" || allow_reverse_expansion) {
      throw std::runtime_error("the port supports the Dubins (forward-only, non-omni) lattice only");
    }
    state_space = std::make_shared<DubinsStateSpace>(lattice_metadata.min_turning_radius);
    motion_model = MotionModel::DUBIN;
  }

  float prev_start_angle = 0.0;
  std::vector<MotionPrimitive> primitives;
  nlohmann::json json_primitives = json["primitives"];
  for (unsigned int i = 0; i < json_primitives.size(); ++i) {
    MotionPrimitive new_primitive;
    fromJsonToMotionPrimitive(json_primitives[i], new_primitive);
    if (prev_start_angle != new_primitive.start_angle) {
      motion_primitives.push_back(primitives);
      primitives.clear();
      prev_start_angle = new_primitive.start_angle;
    }
    primitives.push_back(new_primitive);
  }
  motion_primitives.push_back(primitives);

  trig_values.reserve(lattice_metadata.number_of_headings);
  for (unsigned int i = 0; i < lattice_metadata.heading_angles.size(); ++i) {
    trig_values.emplace_back(std::cos(lattice_metadata.heading_angles[i]),
                             std::sin(lattice_metadata.heading_angles[i]));
  }
}

MotionPrimitivePtrs LatticeMotionTable::getMotionPrimitives(const NodeLattice * node,
                                                            unsigned int & direction_change_index)
{
  MotionPrimitives & prims_at_heading = motion_primitives[node->pose.theta];
  MotionPrimitivePtrs primitive_projection_list;
  for (unsigned int i = 0; i != prims_at_heading.size(); i++) {
    primitive_projection_list.push_back(&prims_at_heading[i]);
  }
  direction_change_index = static_cast<unsigned int>(primitive_projection_list.size());
  // allow_reverse_expansion is false in the supported configuration
  return primitive_projection_list;
}

LatticeMetadata LatticeMotionTable::getLatticeMetadata(const std::string & lattice_filepath)
{
  std::ifstream lattice_file(lattice_filepath);
  if (!lattice_file.is_open()) {
    throw std::runtime_error("Could not open lattice file!");
  }
  nlohmann::json j;
  lattice_file >> j;
  LatticeMetadata metadata;
  fromJsonToMetaData(j["lattice_metadata"], metadata);
  return metadata;
}

unsigned int LatticeMotionTable::getClosestAngularBin(const double & theta)
{
  float min_dist = std::numeric_limits<float>::max();
  unsigned int closest_idx = 0;
  float dist = 0.0;
  for (unsigned int i = 0; i != lattice_metadata.heading_angles.size(); i++) {
    dist = std::fabs(shortest_angular_distance(theta, lattice_metadata.heading_angles[i]));
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

float & LatticeMotionTable::getAngleFromBin(const unsigned int & bin_idx)
{
  return lattice_metadata.heading_angles[bin_idx];
}

double LatticeMotionTable::getAngle(const double & theta) { return getClosestAngularBin(theta); }

NodeLattice::NodeLattice(const uint64_t index, LatticeContext * context_in)
: parent(nullptr), pose(0.0f, 0.0f, 0.0f), context(context_in),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()), _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index), _was_visited(false), _motion_primitive(nullptr), _backwards(false), _is_node_valid(false)
{
}

void NodeLattice::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
  _motion_primitive = nullptr;
  _backwards = false;
  _is_node_valid = false;
}

bool NodeLattice::isNodeValid(const bool & traverse_unknown, GridCollisionChecker * collision_checker,
                              MotionPrimitive * motion_primitive, bool is_backwards)
{
  auto & motion_table = context->motion_table;
  // Already found, we can return the result
  if (!std::isnan(_cell_cost)) {
    return _is_node_valid;
  }
  // Upstream keeps this in a function-local static (one value per process); the bin count is fixed at 72 here.
  const double bin_size = 2.0 * M_PI / collision_checker->getPrecomputedAngles().size();
  const double angle = std::fmod(motion_table.getAngleFromBin(this->pose.theta), 2.0 * M_PI) / bin_size;
  if (collision_checker->inCollision(this->pose.x, this->pose.y, angle, traverse_unknown)) {
    _is_node_valid = false;
    _cell_cost = collision_checker->getCost();
    return false;
  }
  float max_cell_cost = collision_checker->getCost();

  if (motion_primitive) {
    const float & grid_resolution = motion_table.lattice_metadata.grid_resolution;
    const float & resolution_diag_sq = 2.0 * grid_resolution * grid_resolution;
    MotionPose last_pose(1e9, 1e9, 1e9, TurnDirection::UNKNOWN);
    MotionPose pose_dist(0.0, 0.0, 0.0, TurnDirection::UNKNOWN);
    MotionPose initial_pose, prim_pose;
    initial_pose._x = this->pose.x - (motion_primitive->poses.back()._x / grid_resolution);
    initial_pose._y = this->pose.y - (motion_primitive->poses.back()._y / grid_resolution);
    initial_pose._theta = motion_table.getAngleFromBin(motion_primitive->start_angle);

    for (auto it = motion_primitive->poses.begin(); it != motion_primitive->poses.end(); ++it) {
      pose_dist = *it - last_pose;
      if (pose_dist._x * pose_dist._x + pose_dist._y * pose_dist._y > resolution_diag_sq) {
        last_pose = *it;
        prim_pose._x = initial_pose._x + (it->_x / grid_resolution);
        prim_pose._y = initial_pose._y + (it->_y / grid_resolution);
        if (is_backwards) {
          prim_pose._theta = std::fmod(it->_theta + M_PI, 2.0 * M_PI);
        } else {
          prim_pose._theta = std::fmod(it->_theta, 2.0 * M_PI);
        }
        if (collision_checker->inCollision(prim_pose._x, prim_pose._y, prim_pose._theta / bin_size,
                                           traverse_unknown)) {
          _is_node_valid = false;
          _cell_cost = std::max(max_cell_cost, collision_checker->getCost());
          return false;
        }
        max_cell_cost = std::max(max_cell_cost, collision_checker->getCost());
      }
    }
  }
  _cell_cost = max_cell_cost;
  _is_node_valid = true;
  return _is_node_valid;
}

float NodeLattice::getTraversalCost(const NodePtr & child)
{
  auto & motion_table = context->motion_table;
  const float normalized_cost = child->getCost() / 252.0;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error("Node attempted to get traversal cost without a known collision cost!");
  }
  MotionPrimitive * prim = this->getMotionPrimitive();
  MotionPrimitive * transition_prim = child->getMotionPrimitive();
  const float prim_length = transition_prim->trajectory_length / motion_table.lattice_metadata.grid_resolution;
  if (prim == nullptr) {
    return prim_length;
  }
  if (transition_prim->trajectory_length < 1e-4) {
    return motion_table.rotation_penalty * (1.0 + motion_table.cost_penalty * normalized_cost);
  }
  float travel_cost = 0.0;
  float travel_cost_raw = prim_length * (motion_table.travel_distance_reward + motion_table.cost_penalty *
                                         normalized_cost);
  if (transition_prim->arc_length < 0.001) {
    travel_cost = travel_cost_raw;
  } else {
    if (prim->left_turn == transition_prim->left_turn) {
      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    } else {
      travel_cost = travel_cost_raw * (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }
  if (child->isBackward()) {
    travel_cost *= motion_table.reverse_penalty;
  }
  return travel_cost;
}

float NodeLattice::getHeuristicCost(LatticeContext & context, const Coordinates & node_coords,
                                    const Coordinates & goal_coords)
{
  const float obstacle_heuristic = getObstacleHeuristic(context, node_coords, goal_coords,
                                                        context.motion_table.cost_penalty);
  const float distance_heuristic = getDistanceHeuristic(context, node_coords, goal_coords, obstacle_heuristic);
  return std::max(obstacle_heuristic, distance_heuristic);
}

float NodeLattice::getDistanceHeuristic(LatticeContext & context, const Coordinates & node_coords,
                                        const Coordinates & goal_coords, const float & obstacle_heuristic)
{
  auto & motion_table = context.motion_table;
  const TrigValues & trig_vals = motion_table.trig_values[goal_coords.theta];
  const float cos_th = trig_vals.first;
  const float sin_th = -trig_vals.second;
  const float dx = node_coords.x - goal_coords.x;
  const float dy = node_coords.y - goal_coords.y;

  double dtheta_bin = node_coords.theta - goal_coords.theta;
  if (dtheta_bin < 0) {
    dtheta_bin += motion_table.num_angle_quantization;
  }
  if (dtheta_bin > motion_table.num_angle_quantization) {
    dtheta_bin -= motion_table.num_angle_quantization;
  }

  Coordinates node_coords_relative(std::round(dx * cos_th - dy * sin_th), std::round(dx * sin_th + dy * cos_th),
                                   std::round(dtheta_bin));

  float motion_heuristic = 0.0;
  const int floored_size = std::floor(context.size_lookup / 2.0);
  const int ceiling_size = std::ceil(context.size_lookup / 2.0);
  const float mirrored_relative_y = std::abs(node_coords_relative.y);
  if (std::abs(node_coords_relative.x) < floored_size && mirrored_relative_y < floored_size) {
    int theta_pos;
    if (node_coords_relative.y < 0.0) {
      theta_pos = motion_table.num_angle_quantization - node_coords_relative.theta;
    } else {
      theta_pos = node_coords_relative.theta;
    }
    const int x_pos = node_coords_relative.x + floored_size;
    const int y_pos = static_cast<int>(mirrored_relative_y);
    const int index = x_pos * ceiling_size * motion_table.num_angle_quantization +
      y_pos * motion_table.num_angle_quantization + theta_pos;
    motion_heuristic = context.dist_heuristic_lookup_table[index];
  } else if (obstacle_heuristic == 0.0) {
    DubinsStateSpace::State from, to;
    to.x = goal_coords.x;
    to.y = goal_coords.y;
    to.yaw = motion_table.getAngleFromBin(goal_coords.theta);
    from.x = node_coords.x;
    from.y = node_coords.y;
    from.yaw = motion_table.getAngleFromBin(node_coords.theta);
    motion_heuristic = motion_table.state_space->distance(from, to);
  }
  return motion_heuristic;
}

void NodeLattice::precomputeDistanceHeuristic(LatticeContext & context, const float & lookup_table_dim,
                                              const unsigned int & dim_3_size, const SearchInfo & search_info)
{
  auto & motion_table = context.motion_table;
  motion_table.lattice_metadata = LatticeMotionTable::getLatticeMetadata(search_info.lattice_filepath);
  if (motion_table.lattice_metadata.motion_model == "omni" || search_info.allow_reverse_expansion) {
    throw std::runtime_error("the port supports the Dubins (forward-only, non-omni) lattice only");
  }
  motion_table.state_space = std::make_shared<DubinsStateSpace>(search_info.minimum_turning_radius);
  motion_table.motion_model = MotionModel::DUBIN;

  DubinsStateSpace::State from, to;
  to.x = 0.0;
  to.y = 0.0;
  to.yaw = 0.0;
  context.size_lookup = lookup_table_dim;
  float motion_heuristic = 0.0;
  unsigned int index = 0;
  int dim_3_size_int = static_cast<int>(dim_3_size);
  const float size_lookup = context.size_lookup;
  context.dist_heuristic_lookup_table.resize(size_lookup * std::ceil(size_lookup / 2.0) * dim_3_size_int);
  for (float x = std::ceil(-size_lookup / 2.0); x <= std::floor(size_lookup / 2.0); x += 1.0) {
    for (float y = 0.0; y <= std::floor(size_lookup / 2.0); y += 1.0) {
      for (int heading = 0; heading != dim_3_size_int; heading++) {
        from.x = x;
        from.y = y;
        from.yaw = motion_table.getAngleFromBin(heading);
        motion_heuristic = motion_table.state_space->distance(from, to);
        context.dist_heuristic_lookup_table[index] = motion_heuristic;
        index++;
      }
    }
  }
}

void NodeLattice::resetObstacleHeuristic(LatticeContext & context, const Costmap * costmap,
                                         const unsigned int & start_x, const unsigned int & start_y,
                                         const unsigned int & goal_x, const unsigned int & goal_y)
{
  context.costmap = costmap;
  auto & table = context.obstacle_heuristic_lookup_table;
  unsigned int size = 0u;
  unsigned int size_x = 0u;
  if (context.downsample_obstacle_heuristic) {
    size_x = std::ceil(static_cast<float>(costmap->getSizeInCellsX()) / 2.0f);
    size = size_x * std::ceil(static_cast<float>(costmap->getSizeInCellsY()) / 2.0f);
  } else {
    size_x = costmap->getSizeInCellsX();
    size = size_x * costmap->getSizeInCellsY();
  }
  if (table.size() == size) {
    std::fill(table.begin(), table.end(), 0.0f);
  } else {
    unsigned int obstacle_size = table.size();
    table.resize(size, 0.0f);
    std::fill_n(table.begin(), obstacle_size, 0.0f);
  }
  context.obstacle_heuristic_queue.clear();
  context.obstacle_heuristic_queue.reserve(size);

  unsigned int goal_index;
  if (context.downsample_obstacle_heuristic) {
    goal_index = std::floor(goal_y / 2.0f) * size_x + std::floor(goal_x / 2.0f);
  } else {
    goal_index = std::floor(goal_y) * size_x + std::floor(goal_x);
  }
  context.obstacle_heuristic_queue.emplace_back(distanceHeuristic2D(goal_index, size_x, start_x, start_y),
                                                goal_index);
  table[goal_index] = -0.00001f;
}

float NodeLattice::getObstacleHeuristic(LatticeContext & context, const Coordinates & node_coords,
                                        const Coordinates & /*goal_coords*/, const float & cost_penalty)
{
  const Costmap * costmap = context.costmap;
  auto & table = context.obstacle_heuristic_lookup_table;
  auto & queue = context.obstacle_heuristic_queue;
  unsigned int size_x = 0u;
  unsigned int size_y = 0u;
  if (context.downsample_obstacle_heuristic) {
    size_x = std::ceil(static_cast<float>(costmap->getSizeInCellsX()) / 2.0f);
    size_y = std::ceil(static_cast<float>(costmap->getSizeInCellsY()) / 2.0f);
  } else {
    size_x = costmap->getSizeInCellsX();
    size_y = costmap->getSizeInCellsY();
  }
  unsigned int start_y, start_x;
  const bool & downsample_H = context.downsample_obstacle_heuristic;
  if (downsample_H) {
    start_y = std::floor(node_coords.y / 2.0f);
    start_x = std::floor(node_coords.x / 2.0f);
  } else {
    start_y = std::floor(node_coords.y);
    start_x = std::floor(node_coords.x);
  }
  const unsigned int start_index = start_y * size_x + start_x;
  const float & requested_node_cost = table[start_index];
  if (requested_node_cost > 0.0f) {
    return downsample_H ? 2.0f * requested_node_cost : requested_node_cost;
  }

  for (auto & n : queue) {
    n.first = -table[n.second] + distanceHeuristic2D(n.second, size_x, start_x, start_y);
  }
  std::make_heap(queue.begin(), queue.end(), ObstacleHeuristicComparator{});

  const int size_x_int = static_cast<int>(size_x);
  const float sqrt2 = sqrtf(2.0f);
  float c_cost, cost, travel_cost, new_cost, existing_cost;
  unsigned int mx, my;
  unsigned int idx, new_idx = 0;
  const std::vector<int> neighborhood = {1, -1, size_x_int, -size_x_int, size_x_int + 1, size_x_int - 1,
                                         -size_x_int + 1, -size_x_int - 1};

  while (!queue.empty()) {
    idx = queue.front().second;
    std::pop_heap(queue.begin(), queue.end(), ObstacleHeuristicComparator{});
    queue.pop_back();
    c_cost = table[idx];
    if (c_cost > 0.0f) {
      continue;
    }
    c_cost = -c_cost;
    table[idx] = c_cost;

    for (unsigned int i = 0; i != neighborhood.size(); i++) {
      new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);
      if (new_idx < size_x * size_y) {
        if (downsample_H) {
          unsigned int y_offset = (new_idx / size_x) * 2;
          unsigned int x_offset = (new_idx - ((new_idx / size_x) * size_x)) * 2;
          cost = costmap->getCost(x_offset, y_offset);
          for (unsigned int k = 0; k < 2u; ++k) {
            unsigned int mxd = x_offset + k;
            if (mxd >= costmap->getSizeInCellsX()) {
              continue;
            }
            for (unsigned int j = 0; j < 2u; ++j) {
              unsigned int myd = y_offset + j;
              if (myd >= costmap->getSizeInCellsY()) {
                continue;
              }
              if (k == 0 && j == 0) {
                continue;
              }
              cost = std::min(cost, static_cast<float>(costmap->getCost(mxd, myd)));
            }
          }
        } else {
          cost = static_cast<float>(costmap->getCost(new_idx));
        }
        if (cost >= INSCRIBED) {
          continue;
        }
        my = new_idx / size_x;
        mx = new_idx - (my * size_x);
        if (mx >= size_x - 3 || mx <= 3) {
          continue;
        }
        if (my >= size_y - 3 || my <= 3) {
          continue;
        }
        existing_cost = table[new_idx];
        if (existing_cost <= 0.0f) {
          if (context.use_quadratic_cost_penalty) {
            travel_cost = (i <= 3 ? 1.0f : sqrt2) * (1.0f + (cost_penalty * cost * cost / 63504.0f));
          } else {
            travel_cost = ((i <= 3) ? 1.0f : sqrt2) * (1.0f + (cost_penalty * cost / 252.0f));
          }
          new_cost = c_cost + travel_cost;
          if (existing_cost == 0.0f || -existing_cost > new_cost) {
            table[new_idx] = -new_cost;
            queue.emplace_back(new_cost + distanceHeuristic2D(new_idx, size_x, start_x, start_y), new_idx);
            std::push_heap(queue.begin(), queue.end(), ObstacleHeuristicComparator{});
          }
        }
      }
    }
    if (idx == start_index) {
      break;
    }
  }
  return downsample_H ? 2.0f * requested_node_cost : requested_node_cost;
}

void NodeLattice::getNeighbors(NodeGetter & NeighborGetter, GridCollisionChecker * collision_checker,
                               const bool & traverse_unknown, NodeVector & neighbors)
{
  auto & motion_table = context->motion_table;
  uint64_t index = 0;
  bool backwards = false;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords, motion_projection;
  unsigned int direction_change_index = 0;
  MotionPrimitivePtrs motion_primitives = motion_table.getMotionPrimitives(this, direction_change_index);
  const float & grid_resolution = motion_table.lattice_metadata.grid_resolution;

  for (unsigned int i = 0; i != motion_primitives.size(); i++) {
    const MotionPose & end_pose = motion_primitives[i]->poses.back();
    motion_projection.x = this->pose.x + (end_pose._x / grid_resolution);
    motion_projection.y = this->pose.y + (end_pose._y / grid_resolution);
    motion_projection.theta = motion_primitives[i]->end_angle;
    backwards = false;
    if (i >= direction_change_index) {
      backwards = true;
      float opposite_heading_theta = motion_projection.theta - (motion_table.num_angle_quantization / 2);
      if (opposite_heading_theta < 0) {
        opposite_heading_theta += motion_table.num_angle_quantization;
      }
      if (opposite_heading_theta > motion_table.num_angle_quantization) {
        opposite_heading_theta -= motion_table.num_angle_quantization;
      }
      motion_projection.theta = opposite_heading_theta;
    }
    index = NodeLattice::getIndex(*context, static_cast<unsigned int>(motion_projection.x),
                                  static_cast<unsigned int>(motion_projection.y),
                                  static_cast<unsigned int>(motion_projection.theta));
    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      initial_node_coords = neighbor->pose;
      neighbor->setPose(Coordinates(motion_projection.x, motion_projection.y, motion_projection.theta));
      if (neighbor->isNodeValid(traverse_unknown, collision_checker, motion_primitives[i], backwards)) {
        neighbor->setMotionPrimitive(motion_primitives[i]);
        neighbor->backwards(backwards);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

bool NodeLattice::backtracePath(CoordinateVector & path)
{
  if (!this->parent) {
    return false;
  }
  NodePtr current_node = this;
  while (current_node->parent) {
    addNodeToPath(current_node, path);
    current_node = current_node->parent;
  }
  addNodeToPath(current_node, path);
  return true;
}

void NodeLattice::addNodeToPath(NodeLattice::NodePtr current_node, CoordinateVector & path)
{
  auto & motion_table = context->motion_table;
  Coordinates initial_pose, prim_pose;
  MotionPrimitive * prim = nullptr;
  const float & grid_resolution = motion_table.lattice_metadata.grid_resolution;
  prim = current_node->getMotionPrimitive();
  if (prim) {
    initial_pose.x = current_node->pose.x - (prim->poses.back()._x / grid_resolution);
    initial_pose.y = current_node->pose.y - (prim->poses.back()._y / grid_resolution);
    initial_pose.theta = motion_table.getAngleFromBin(prim->start_angle);
    for (auto it = prim->poses.crbegin(); it != prim->poses.crend(); ++it) {
      prim_pose.x = initial_pose.x + (it->_x / grid_resolution);
      prim_pose.y = initial_pose.y + (it->_y / grid_resolution);
      if (current_node->isBackward()) {
        prim_pose.theta = std::fmod(it->_theta + M_PI, 2.0 * M_PI);
      } else {
        prim_pose.theta = it->_theta;
      }
      path.push_back(prim_pose);
    }
  } else {
    path.push_back(current_node->pose);
    path.back().theta = motion_table.getAngleFromBin(path.back().theta);
  }
}

void NodeBasic::populateSearchNode(NodeLattice * & node)
{
  this->pose = node->pose;
  this->graph_node_ptr = node;
  this->prim_ptr = node->getMotionPrimitive();
  this->backward = node->isBackward();
}

void NodeBasic::processSearchNode()
{
  if (!this->graph_node_ptr->wasVisited()) {
    this->graph_node_ptr->pose = this->pose;
    this->graph_node_ptr->setMotionPrimitive(this->prim_ptr);
    this->graph_node_ptr->backwards(this->backward);
  }
}
}  // namespace mobile_manipulator_routing::smac
