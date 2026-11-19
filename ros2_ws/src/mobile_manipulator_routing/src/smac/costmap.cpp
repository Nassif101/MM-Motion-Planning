// Copyright (c) 2008, 2013, Willow Garage, Inc.; Copyright (c) 2015, Fetch Robotics, Inc.
// Licensed under the BSD 3-Clause License (nav2_costmap_2d); see third_party/NOTICE.md.
// Ported from navigation2 1.3.13 nav2_costmap_2d; the source files are named in costmap.hpp.
#include "mobile_manipulator_routing/smac/costmap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mobile_manipulator_routing::smac
{
Costmap::Costmap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                 unsigned char default_value)
: size_x_(size_x), size_y_(size_y), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y),
  costmap_(static_cast<std::size_t>(size_x)*size_y, default_value)
{
}

void Costmap::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool Costmap::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }
  mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
  return mx < size_x_ && my < size_y_;
}

bool Costmap::worldToMapContinuous(double wx, double wy, float & mx, float & my) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }
  mx = static_cast<float>((wx - origin_x_) / resolution_);
  my = static_cast<float>((wy - origin_y_) / resolution_);
  return mx < size_x_ && my < size_y_;
}

unsigned int Costmap::cellDistance(double world_dist) const
{
  double cells_dist = std::max(0.0, std::ceil(world_dist / resolution_));
  return static_cast<unsigned int>(cells_dist);
}

namespace
{
double sign0(double x) { return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0); }
double point_distance(double x0, double y0, double x1, double y1) { return std::hypot(x1 - x0, y1 - y0); }

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
{
  double A = pX - x0;
  double B = pY - y0;
  double C = x1 - x0;
  double D = y1 - y0;
  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;
  double xx, yy;
  if (param < 0) {
    xx = x0;
    yy = y0;
  } else if (param > 1) {
    xx = x1;
    yy = y1;
  } else {
    xx = x0 + param * C;
    yy = y0 + param * D;
  }
  return point_distance(pX, pY, xx, yy);
}

struct CellData
{
  CellData(unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) : x_(x), y_(y), src_x_(sx), src_y_(sy) {}
  unsigned int x_, y_, src_x_, src_y_;
};
}  // namespace

void padFootprint(Footprint & footprint, double padding)
{
  for (auto & pt : footprint) {
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}

std::pair<double, double> calculateMinAndMaxDistances(const Footprint & footprint)
{
  double min_dist = std::numeric_limits<double>::max();
  double max_dist = 0.0;
  if (footprint.size() <= 2) {
    return {min_dist, max_dist};
  }
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    double vertex_dist = point_distance(0.0, 0.0, footprint[i].x, footprint[i].y);
    double edge_dist = distanceToLine(0.0, 0.0, footprint[i].x, footprint[i].y, footprint[i + 1].x, footprint[i + 1].y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }
  double vertex_dist = point_distance(0.0, 0.0, footprint.back().x, footprint.back().y);
  double edge_dist = distanceToLine(0.0, 0.0, footprint.back().x, footprint.back().y, footprint.front().x,
                                    footprint.front().y);
  min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
  max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  return {min_dist, max_dist};
}

unsigned char inflation_cost(double distance, double resolution, double inscribed_radius, double cost_scaling_factor)
{
  unsigned char cost = 0;
  if (distance == 0) {
    cost = kLethalObstacle;
  } else if (distance * resolution <= inscribed_radius) {
    cost = kInscribedInflatedObstacle;
  } else {
    double factor = std::exp(-1.0 * cost_scaling_factor * (distance * resolution - inscribed_radius));
    cost = static_cast<unsigned char>((kInscribedInflatedObstacle - 1) * factor);
  }
  return cost;
}

void inflate(Costmap & master, double inflation_radius, double inscribed_radius, double cost_scaling_factor)
{
  const unsigned int cell_inflation_radius = master.cellDistance(inflation_radius);
  if (cell_inflation_radius == 0) {
    return;
  }
  const double resolution = master.getResolution();
  // computeCaches
  const unsigned int cache_length = cell_inflation_radius + 2;
  std::vector<double> cached_distances(cache_length * cache_length);
  std::vector<unsigned char> cached_costs(cache_length * cache_length);
  for (unsigned int i = 0; i < cache_length; ++i) {
    for (unsigned int j = 0; j < cache_length; ++j) {
      cached_distances[i * cache_length + j] = std::hypot(i, j);
    }
  }
  for (unsigned int i = 0; i < cache_length; ++i) {
    for (unsigned int j = 0; j < cache_length; ++j) {
      cached_costs[i * cache_length + j] =
        inflation_cost(cached_distances[i * cache_length + j], resolution, inscribed_radius, cost_scaling_factor);
    }
  }
  // generateIntegerDistances
  const int r = static_cast<int>(cell_inflation_radius) + 2;
  const int size = r * 2 + 1;
  std::vector<std::pair<int, int>> points;
  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      if (x * x + y * y <= r * r) {
        points.emplace_back(x, y);
      }
    }
  }
  std::sort(points.begin(), points.end(), [](const std::pair<int, int> & a, const std::pair<int, int> & b) {
      return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
    });
  std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
  std::pair<int, int> last = {0, 0};
  int level = 0;
  for (auto const & p : points) {
    if (p.first * p.first + p.second * p.second != last.first * last.first + last.second * last.second) {
      level++;
    }
    distance_matrix[p.first + r][p.second + r] = level;
    last = p;
  }
  std::vector<std::vector<CellData>> inflation_cells(level + 1);

  // updateCosts over the whole map (bounds are the full map on a first update)
  unsigned char * master_array = master.getCharMap();
  const unsigned int size_x = master.getSizeInCellsX(), size_y = master.getSizeInCellsY();
  std::vector<bool> seen(static_cast<std::size_t>(size_x) * size_y, false);
  const auto distanceLookup = [&](unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y) {
      unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
      unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
      return cached_distances[dx * cache_length + dy];
    };
  const auto costLookup = [&](unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y) {
      unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
      unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
      return cached_costs[dx * cache_length + dy];
    };
  const auto enqueue = [&](unsigned int index, unsigned int mx, unsigned int my, unsigned int src_x,
                           unsigned int src_y) {
      if (!seen[index]) {
        double distance = distanceLookup(mx, my, src_x, src_y);
        if (distance > cell_inflation_radius) {
          return;
        }
        const unsigned int rr = cell_inflation_radius + 2;
        const auto dist = distance_matrix[mx - src_x + rr][my - src_y + rr];
        inflation_cells[dist].emplace_back(mx, my, src_x, src_y);
      }
    };

  auto & obs_bin = inflation_cells[0];
  for (unsigned int j = 0; j < size_y; j++) {
    for (unsigned int i = 0; i < size_x; i++) {
      unsigned char cost = master_array[master.getIndex(i, j)];
      if (cost == kLethalObstacle) {  // inflate_around_unknown is false
        obs_bin.emplace_back(i, j, i, j);
      }
    }
  }
  for (auto & dist_bin : inflation_cells) {
    for (std::size_t i = 0; i < dist_bin.size(); ++i) {
      const CellData cell = dist_bin[i];  // a copy: the bin may grow while processing
      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;
      unsigned int index = master.getIndex(mx, my);
      if (seen[index]) {
        continue;
      }
      seen[index] = true;
      unsigned char cost = costLookup(mx, my, sx, sy);
      unsigned char old_cost = master_array[index];
      // inflate_unknown is false
      if (old_cost == kNoInformation && cost >= kInscribedInflatedObstacle) {
        master_array[index] = cost;
      } else {
        master_array[index] = std::max(old_cost, cost);
      }
      if (mx > 0) {
        enqueue(index - 1, mx - 1, my, sx, sy);
      }
      if (my > 0) {
        enqueue(index - size_x, mx, my - 1, sx, sy);
      }
      if (mx < size_x - 1) {
        enqueue(index + 1, mx + 1, my, sx, sy);
      }
      if (my < size_y - 1) {
        enqueue(index + size_x, mx, my + 1, sx, sy);
      }
    }
    dist_bin = std::vector<CellData>();
  }
}

LayeredCostmap build_costmap(const GridSnapshot & grid, const Footprint & footprint, const CostmapSettings & settings)
{
  if (footprint.size() < 3) {
    throw std::invalid_argument("the costmap needs a footprint polygon");
  }
  if (!std::isfinite(settings.footprint_padding) || !std::isfinite(settings.inflation_radius) ||
      !std::isfinite(settings.cost_scaling_factor) || settings.inflation_radius < 0.0) {
    throw std::invalid_argument("costmap settings must be finite");
  }
  LayeredCostmap out;
  out.padded_footprint = footprint;
  padFootprint(out.padded_footprint, settings.footprint_padding);
  std::tie(out.inscribed_radius, out.circumscribed_radius) = calculateMinAndMaxDistances(out.padded_footprint);
  out.inflation_radius = settings.inflation_radius;
  out.cost_scaling_factor = settings.cost_scaling_factor;
  const auto origin = grid.origin();
  // The static layer sizes the master costmap from the map message, whose resolution is a 32-bit float
  // (nav_msgs/MapMetaData): 0.05 becomes 0.0500000007. Every Smac quantity derived from the costmap inherits it.
  const double resolution = static_cast<double>(static_cast<float>(grid.resolution()));
  out.costmap = Costmap(static_cast<unsigned int>(grid.width()), static_cast<unsigned int>(grid.height()),
                        resolution, origin.x, origin.y, kFreeSpace);
  // static layer (trinary, track_unknown_space false: unknown is free) and observed obstacles, as lethal marks
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      if (grid.occupancy(x, y) == Cell::Occupied) {
        out.costmap.setCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y), kLethalObstacle);
      }
    }
  }
  inflate(out.costmap, out.inflation_radius, out.inscribed_radius, out.cost_scaling_factor);
  return out;
}
}  // namespace mobile_manipulator_routing::smac
