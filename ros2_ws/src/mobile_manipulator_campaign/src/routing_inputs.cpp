#include "mobile_manipulator_campaign/routing_inputs.hpp"

#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/polygon_ops.hpp"
#include "mobile_manipulator_routing/map_io.hpp"
#include "mobile_manipulator_routing/search.hpp"

namespace mobile_manipulator_campaign
{
namespace
{
namespace mr = mobile_manipulator_routing;
namespace mg = mobile_manipulator_geometry;
namespace fs = std::filesystem;

std::string digest(const char * data, std::size_t size)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> bytes{};
  unsigned count=0;
  if (EVP_Digest(data,size,bytes.data(),&count,EVP_sha256(),nullptr)!=1)
    throw std::runtime_error("routing input SHA-256 failed");
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned i=0; i<count; ++i) out << std::setw(2) << static_cast<unsigned>(bytes[i]);
  return out.str();
}
std::string file_digest(const fs::path & path)
{
  if (!fs::is_regular_file(path) || fs::file_size(path) > (64u << 20))
    throw std::invalid_argument("routing input file is missing or exceeds 64 MiB");
  std::ifstream stream(path,std::ios::binary);
  if (!stream) throw std::runtime_error("cannot read routing input fingerprint");
  std::ostringstream data;
  data << stream.rdbuf();
  if (stream.bad()) throw std::runtime_error("routing input fingerprint read failed");
  const auto value=data.str();
  return digest(value.data(),value.size());
}

bool intersects(const mr::Polygon & polygon, double x0, double y0, double x1, double y1)
{
  const double cx=(x0+x1)*0.5, cy=(y0+y1)*0.5, hx=(x1-x0)*0.5, hy=(y1-y0)*0.5;
  const auto separated=[&](double nx,double ny) {
    double lo=std::numeric_limits<double>::infinity(), hi=-lo;
    for (const auto & p:polygon) {
      const double v=nx*(p.x-cx)+ny*(p.y-cy);
      lo=std::min(lo,v); hi=std::max(hi,v);
    }
    const double reach=std::abs(nx)*hx+std::abs(ny)*hy;
    return hi < -reach-1e-12 || lo > reach+1e-12;
  };
  if (separated(1,0) || separated(0,1)) return false;
  for (std::size_t i=0; i<polygon.size(); ++i) {
    const auto & a=polygon[i]; const auto & b=polygon[(i+1)%polygon.size()];
    if (separated(a.y-b.y,b.x-a.x)) return false;
  }
  return true;
}

void rasterize(const mr::GridSnapshot & grid, std::vector<mr::Cell> & known,
               const mobile_manipulator_navigation::Box & box)
{
  if (!box.center.allFinite() || !box.size.allFinite() || (box.size.array()<=0).any() ||
      box.center.cwiseAbs().maxCoeff()>1e6 || box.size.maxCoeff()>1e6 || !std::isfinite(box.yaw))
    throw std::invalid_argument("invalid explicitly known routing box");
  if (box.center.z()+box.size.z()*0.5 < 0.02 || box.center.z()-box.size.z()*0.5 > 0.60) return;
  const double c=std::cos(box.yaw), s=std::sin(box.yaw);
  mr::Polygon polygon;
  for (const auto & sign:std::array<mr::Point2,4>{{{-1,-1},{1,-1},{1,1},{-1,1}}}) {
    const double x=sign.x*box.size.x()*0.5, y=sign.y*box.size.y()*0.5;
    polygon.push_back({box.center.x()+c*x-s*y,box.center.y()+s*x+c*y});
  }
  const auto origin=grid.origin(); const double r=grid.resolution();
  // Clamp in floating point before integer conversion; boxes may straddle map bounds.
  double lo_x=polygon[0].x, hi_x=lo_x, lo_y=polygon[0].y, hi_y=lo_y;
  for (const auto & p:polygon) {
    lo_x=std::min(lo_x,p.x); hi_x=std::max(hi_x,p.x);
    lo_y=std::min(lo_y,p.y); hi_y=std::max(hi_y,p.y);
  }
  if (hi_x<origin.x || hi_y<origin.y || lo_x>origin.x+grid.width()*r || lo_y>origin.y+grid.height()*r) return;
  const auto low=[](double v,int n) { return static_cast<int>(std::clamp(std::floor(v)-1.0,0.0,double(n-1))); };
  const auto high=[](double v,int n) { return static_cast<int>(std::clamp(std::floor(v)+1.0,0.0,double(n-1))); };
  for (int y=low((lo_y-origin.y)/r,grid.height()); y<=high((hi_y-origin.y)/r,grid.height()); ++y)
    for (int x=low((lo_x-origin.x)/r,grid.width()); x<=high((hi_x-origin.x)/r,grid.width()); ++x)
      if (intersects(polygon,origin.x+x*r,origin.y+y*r,origin.x+(x+1)*r,origin.y+(y+1)*r))
        known[static_cast<std::size_t>(y)*grid.width()+x]=mr::Cell::Occupied;
}
}  // namespace

RoutingInputs load_routing_inputs(const std::string & navigation_root, const std::string & urdf,
  const nlohmann::ordered_json & payload, const std::string & map_path, const std::string & primitive_path,
  const std::vector<mobile_manipulator_navigation::Box> & known_additions)
{
  if (known_additions.size()>4096) throw std::invalid_argument("too many explicitly known routing additions");
  const fs::path root=navigation_root;
  const auto dynamic_path=root/"config/dynamic_footprint.yaml";
  const auto global_path=root/"config/nav2_global_planning.yaml";
  const auto dynamic=YAML::LoadFile(dynamic_path.string())["dynamic_footprint_node"]["ros__parameters"];
  const auto global=YAML::LoadFile(global_path.string())["global_costmap"]["global_costmap"]["ros__parameters"];
  const double geometry_padding=dynamic["padding_m"].as<double>();
  const double map_padding=global["footprint_padding"].as<double>();
  if (!std::isfinite(geometry_padding) || geometry_padding<0 || !std::isfinite(map_padding) || map_padding<0)
    throw std::invalid_argument("invalid routing footprint padding");
  std::set<std::string> links;
  for (const auto & link:dynamic["disc"]["base_links"]) links.insert(link.as<std::string>());
  if (links.empty()) throw std::invalid_argument("routing needs an explicit base-link set");
  mg::FootprintProjector projector(urdf,mg::payload_from_json(payload));
  const auto points=projector.projected_points({},mg::ProjectionOptions{16,links,false});
  const auto costmap_polygon=mg::offset_outward(mg::convex_hull(points),geometry_padding);
  RoutingInputs out;
  // Spec 2 Amendment 3 (user, 2026-11-19): the swept validation checks the hull with the geometric padding only; Smac
  // checks its poses with the costmap's footprint_padding added. The smaller footprint offsets the checker's cell
  // dilation, which rejected a clear store_room route in Task 13 (the padded footprint was 8.6 mm clear).
  for (const auto & p:costmap_polygon) out.footprint.push_back({p[0],p[1]});
  for (const auto & p:costmap_polygon) out.costmap_footprint.push_back({p[0],p[1]});
  out.costmap.footprint_padding=map_padding;
  const fs::path map=map_path.empty() ? root/"maps/construction_site_v2.yaml" : fs::path(map_path);
  const fs::path primitive=primitive_path.empty()
    ? fs::path(ROUTING_DATA_DIR)/"diff_5cm.json"
    : fs::path(primitive_path);
  const mr::InflationParameters inflation{global["inflation_layer"]["inflation_radius"].as<double>(),
    global["inflation_layer"]["cost_scaling_factor"].as<double>()};
  out.costmap.inflation_radius=inflation.radius_m;
  out.costmap.cost_scaling_factor=inflation.scaling;
  auto grid=mr::load_map(map,inflation);
  if (std::abs(grid.resolution()-0.05)>1e-9) throw std::invalid_argument("the active routing primitive profile needs a 5 cm map");
  if (!known_additions.empty()) {
    auto known=grid.known();
    for (const auto & box:known_additions) rasterize(grid,known,box);
    out.grid=std::make_shared<const mr::GridSnapshot>(grid.width(),grid.height(),grid.resolution(),grid.origin(),
      std::move(known),grid.observed(),inflation,grid.revision()+1);
  } else out.grid=std::make_shared<const mr::GridSnapshot>(std::move(grid));
  out.primitives=std::make_shared<const mr::PrimitiveSet>(mr::PrimitiveSet::load(primitive,out.grid->resolution()));
  out.primitive_path=primitive.string();
  {
    const auto planner=YAML::LoadFile(global_path.string())["planner_server"]["ros__parameters"]["Lattice"];
    if (!planner || planner["plugin"].as<std::string>()!="nav2_smac_planner::SmacPlannerLattice")
      throw std::invalid_argument("the global planning configuration has no SmacPlannerLattice block");
    auto & l=out.lattice;
    const auto f=[&](const char * key, float & value) { if (planner[key]) value=static_cast<float>(planner[key].as<double>()); };
    const auto d=[&](const char * key, double & value) { if (planner[key]) value=planner[key].as<double>(); };
    const auto i=[&](const char * key, int & value) { if (planner[key]) value=planner[key].as<int>(); };
    const auto b=[&](const char * key, bool & value) { if (planner[key]) value=planner[key].as<bool>(); };
    l.lattice_filepath=out.primitive_path;
    f("tolerance",l.tolerance); b("allow_unknown",l.allow_unknown); i("max_iterations",l.max_iterations);
    i("max_on_approach_iterations",l.max_on_approach_iterations);
    d("analytic_expansion_max_length",l.analytic_expansion_max_length_m);
    f("analytic_expansion_ratio",l.analytic_expansion_ratio); f("analytic_expansion_max_cost",l.analytic_expansion_max_cost);
    b("analytic_expansion_max_cost_override",l.analytic_expansion_max_cost_override);
    f("reverse_penalty",l.reverse_penalty); f("change_penalty",l.change_penalty);
    f("non_straight_penalty",l.non_straight_penalty); f("cost_penalty",l.cost_penalty);
    f("rotation_penalty",l.rotation_penalty); f("retrospective_penalty",l.retrospective_penalty);
    d("lookup_table_size",l.lookup_table_size); b("cache_obstacle_heuristic",l.cache_obstacle_heuristic);
    b("allow_reverse_expansion",l.allow_reverse_expansion); b("smooth_path",l.smooth_path);
    if (const auto sm=planner["smoother"]) {
      if (sm["max_iterations"]) l.smoother.max_its_=sm["max_iterations"].as<int>();
      if (sm["w_smooth"]) l.smoother.w_smooth_=sm["w_smooth"].as<double>();
      if (sm["w_data"]) l.smoother.w_data_=sm["w_data"].as<double>();
      if (sm["tolerance"]) l.smoother.tolerance_=sm["tolerance"].as<double>();
      if (sm["do_refinement"]) l.smoother.do_refinement_=sm["do_refinement"].as<bool>();
      if (sm["refinement_num"]) l.smoother.refinement_num_=sm["refinement_num"].as<int>();
    }
    if (l.allow_reverse_expansion) throw std::invalid_argument("the Smac port supports forward-only lattices");
  }
  const mr::FootprintChecker validated_footprint(out.grid,out.footprint);
  (void)validated_footprint;
  const auto map_yaml=YAML::LoadFile(map.string());
  const auto image=map.parent_path()/map_yaml["image"].as<std::string>();
  const auto metadata=map.parent_path()/(map.stem().string()+".metadata.json");
  const auto image_hash=file_digest(image);
  if (fs::is_regular_file(metadata)) {
    if (fs::file_size(metadata)>(1u<<20)) throw std::invalid_argument("routing map metadata exceeds 1 MiB");
    std::ifstream input(metadata);
    nlohmann::json sidecar;
    input>>sidecar;
    if (sidecar.contains("pgmSha256") && sidecar.at("pgmSha256")!=image_hash)
      throw std::invalid_argument("routing map image does not match its exported metadata");
  }
  nlohmann::json footprint=nlohmann::json::array(), additions=nlohmann::json::array();
  for (const auto & p:out.footprint) footprint.push_back({p.x,p.y});
  for (const auto & box:known_additions) additions.push_back({{"name",box.name},
    {"center",{box.center.x(),box.center.y(),box.center.z()}},{"size",{box.size.x(),box.size.y(),box.size.z()}},{"yaw",box.yaw}});
  out.metadata={{"map_yaml_sha256",file_digest(map)},{"map_image_sha256",image_hash},
    {"map_metadata_sha256",fs::is_regular_file(metadata) ? nlohmann::json(file_digest(metadata)) : nlohmann::json(nullptr)},
    {"primitive_sha256",file_digest(primitive)},{"urdf_sha256",digest(urdf.data(),urdf.size())},
    {"dynamic_footprint_config_sha256",file_digest(dynamic_path)},{"global_planning_config_sha256",file_digest(global_path)},
    {"footprint",footprint},{"validation_footprint","geometry_padding"},{"geometry_padding_m",geometry_padding},
    {"map_padding_m",map_padding},
    {"known_additions",additions},{"known_raster_height_band_m",{0.02,0.60}},
    {"map_geometry",{{"width",out.grid->width()},{"height",out.grid->height()},
      {"resolution_m",out.grid->resolution()},{"origin",{out.grid->origin().x,out.grid->origin().y}}}},
    {"planner",mr::SearchResult::planner}};
  // spec 2 Task 9: the effective values the planner runs with, beside the hashes of the files they came from
  const auto & l=out.lattice;
  out.metadata["lattice"]={{"tolerance",l.tolerance},{"allow_unknown",l.allow_unknown},{"max_iterations",l.max_iterations},
    {"max_on_approach_iterations",l.max_on_approach_iterations},{"analytic_expansion_max_length_m",l.analytic_expansion_max_length_m},
    {"analytic_expansion_ratio",l.analytic_expansion_ratio},{"analytic_expansion_max_cost",l.analytic_expansion_max_cost},
    {"analytic_expansion_max_cost_override",l.analytic_expansion_max_cost_override},{"reverse_penalty",l.reverse_penalty},
    {"change_penalty",l.change_penalty},{"non_straight_penalty",l.non_straight_penalty},{"cost_penalty",l.cost_penalty},
    {"rotation_penalty",l.rotation_penalty},{"retrospective_penalty",l.retrospective_penalty},
    {"lookup_table_size",l.lookup_table_size},{"cache_obstacle_heuristic",l.cache_obstacle_heuristic},
    {"allow_reverse_expansion",l.allow_reverse_expansion},{"smooth_path",l.smooth_path},
    {"smoother",{{"max_iterations",l.smoother.max_its_},{"w_smooth",l.smoother.w_smooth_},{"w_data",l.smoother.w_data_},
      {"tolerance",l.smoother.tolerance_},{"do_refinement",l.smoother.do_refinement_},{"refinement_num",l.smoother.refinement_num_}}}};
  out.metadata["costmap"]={{"footprint_padding_m",out.costmap.footprint_padding},{"inflation_radius_m",out.costmap.inflation_radius},
    {"cost_scaling_factor",out.costmap.cost_scaling_factor}};
  nlohmann::json costmap_footprint=nlohmann::json::array();
  for (const auto & p:out.costmap_footprint) costmap_footprint.push_back({p.x,p.y});
  out.metadata["costmap_footprint"]=costmap_footprint;
  return out;
}
}  // namespace mobile_manipulator_campaign
