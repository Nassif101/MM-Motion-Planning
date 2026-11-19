#include "mobile_manipulator_routing/map_io.hpp"
#include "mobile_manipulator_routing/primitives.hpp"
#include "mobile_manipulator_routing/search.hpp"

#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

namespace r = mobile_manipulator_routing;
using Json = nlohmann::json;
namespace
{
r::Pose2 pose(const Json & value)
{
  if (!value.is_array() || value.size() != 3) throw std::invalid_argument("pose must have x, y, yaw");
  r::Pose2 result{value.at(0).get<double>(),value.at(1).get<double>(),value.at(2).get<double>()};
  if (!r::finite(result)) throw std::invalid_argument("pose must be finite");
  return result;
}
Json values(const r::Pose2 & value) { return {value.x,value.y,value.yaw}; }
std::size_t positive_size(const Json & value)
{
  if (!value.is_number_integer() || (!value.is_number_unsigned() && value.get<std::int64_t>() <= 0))
    throw std::invalid_argument("search bounds must be positive integers");
  const auto result=value.get<std::uint64_t>();
  if (result==0 || result>std::numeric_limits<std::size_t>::max())
    throw std::invalid_argument("search bound is outside the supported range");
  return static_cast<std::size_t>(result);
}
}

int main(int argc, char ** argv)
{
  try {
    if (argc != 2) throw std::invalid_argument("usage: route_check CASE.json (JSON result on stdout)");
    const std::filesystem::path input = argv[1];
    if (!std::filesystem::is_regular_file(input) || std::filesystem::file_size(input) > (1u << 20))
      throw std::invalid_argument("case must be a regular JSON file no larger than 1 MiB");
    Json data;
    std::ifstream stream(input);
    stream >> data;
    if (!data.is_object()) throw std::invalid_argument("case root must be an object");
    const std::set<std::string> supported{"map","primitives","footprint","start","goal","search"};
    for (auto it=data.begin(); it!=data.end(); ++it)
      if (!supported.count(it.key())) throw std::invalid_argument("unknown case field: "+it.key());
    auto grid = std::make_shared<const r::GridSnapshot>(r::load_map(input.parent_path()/data.at("map").get<std::string>()));
    const auto lattice = input.parent_path()/data.at("primitives").get<std::string>();
    r::PrimitiveSet::load(lattice,grid->resolution());  // validates the file and its resolution
    const auto & polygon = data.at("footprint");
    if (!polygon.is_array() || polygon.size() < 3 || polygon.size() > 256)
      throw std::invalid_argument("footprint must have 3..256 vertices");
    r::Polygon footprint;
    for (const auto & vertex : polygon) {
      if (!vertex.is_array() || vertex.size()!=2) throw std::invalid_argument("footprint vertices need x, y");
      footprint.push_back({vertex.at(0).get<double>(),vertex.at(1).get<double>()});
    }
    r::SearchConfiguration config;
    r::WorkBudget budget;
    if (data.contains("search")) {
      const auto & options=data.at("search");
      if (!options.is_object()) throw std::invalid_argument("search options must be an object");
      for (auto it=options.begin(); it!=options.end(); ++it) {
        const auto & key=it.key();
        if (key=="max_expansions") {
          const auto value=positive_size(it.value());
          if (value>static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::invalid_argument("max_expansions too large");
          config.lattice.max_iterations=static_cast<int>(value);
        } else if (key=="max_path_poses") config.max_path_poses=positive_size(it.value());
        else if (key=="expansions_per_tick") budget.expansions=positive_size(it.value());
        else if (key=="cell_tests_per_tick") budget.cell_tests=positive_size(it.value());
        else throw std::invalid_argument("unknown search option: "+key);
      }
    }
    if (!budget.expansions || !budget.cell_tests) throw std::invalid_argument("driver slice budgets must be positive");
    r::SearchRequest request{pose(data.at("start")),pose(data.at("goal")),{}};
    request.revisions.map=grid->revision();
    config.lattice.lattice_filepath=lattice.string();
    r::smac::Footprint smac_footprint;
    for (const auto & p:footprint) smac_footprint.push_back({p.x,p.y});
    auto planner=std::make_shared<r::RoutePlanner>(grid,smac_footprint,footprint,config);
    r::Search search(planner,request);
    std::size_t slices=0;
    // Bounds the standalone driver as well as the search; a broken cursor cannot spin forever.
    constexpr std::size_t kMaxSlices=10000000;
    while (search.result().status==r::SearchStatus::Pending && slices<kMaxSlices) {
      search.advance(budget);
      ++slices;
    }
    if (search.result().status==r::SearchStatus::Pending) throw std::runtime_error("driver slice limit exceeded");
    const auto & result=search.result();
    Json path=Json::array();
    for (const auto & p:result.poses) path.push_back(values(p));
    const auto & statistics=result.statistics;
    Json out={{"status",r::to_string(result.status)}, {"reason",result.reason},
      {"kind",result.kind==r::PathKind::Stationary ? "stationary" : "translating"},
      {"requested_goal",values(result.requested_goal)}, {"resolved_goal",values(result.resolved_goal)},
      {"planner",r::SearchResult::planner}, {"smoothed",result.smoothed}, {"poses",path},
      {"length_m",result.length_m}, {"slices",slices}, {"expansions",statistics.expansions},
      {"cell_tests",statistics.cell_tests}, {"generated",statistics.generated},
      {"peak_queue",statistics.peak_queue}};
    std::cout << out.dump(2) << '\n';
    return result.status==r::SearchStatus::Found ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "route_check: " << error.what() << '\n';
    return 1;
  }
}
