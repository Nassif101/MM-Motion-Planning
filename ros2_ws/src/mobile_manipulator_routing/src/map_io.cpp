#include "mobile_manipulator_routing/map_io.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace mobile_manipulator_routing
{
namespace
{
constexpr std::uintmax_t kMaxYamlBytes = 1024u * 1024u;
constexpr std::size_t kMaxHeaderBytes = 65536;
constexpr std::uint64_t kMaxDimension = 65536;
constexpr std::uint64_t kMaxCells = 16u * 1024u * 1024u;

class PgmHeader
{
public:
  explicit PgmHeader(std::istream & input) : input_(input) {}

  std::string token()
  {
    int c;
    for (;;) {
      c = get();
      if (c == EOF) throw std::runtime_error("truncated PGM header");
      if (std::isspace(static_cast<unsigned char>(c))) continue;
      if (c == '#') {
        while ((c = get()) != EOF && c != '\n' && c != '\r') {}
        continue;
      }
      break;
    }
    std::string result(1, static_cast<char>(c));
    for (;;) {
      c = input_.peek();
      if (c == EOF || c == '#' || std::isspace(static_cast<unsigned char>(c))) return result;
      if (result.size() >= 32) throw std::runtime_error("oversized PGM header token");
      result.push_back(static_cast<char>(get()));
    }
  }

  void raster_separator(std::size_t payload_bytes, std::uintmax_t file_bytes)
  {
    const int c = get();
    if (c == EOF || !std::isspace(static_cast<unsigned char>(c))) {
      throw std::runtime_error("PGM maximum value must end in one whitespace separator");
    }
    // Accept a CRLF line ending only when the exact payload length calls for it.
    // A legal CR separator followed by a first pixel of LF must keep that pixel.
    const auto at = input_.tellg();
    if (c == '\r' && input_.peek() == '\n' && at >= 0 &&
        static_cast<std::uintmax_t>(at) <= file_bytes &&
        file_bytes - static_cast<std::uintmax_t>(at) == payload_bytes + 1) get();
  }

private:
  int get()
  {
    if (++bytes_ > kMaxHeaderBytes) throw std::runtime_error("PGM header exceeds its size limit");
    return input_.get();
  }
  std::istream & input_;
  std::size_t bytes_ = 0;
};

std::uint64_t positive_integer(const std::string & text)
{
  std::uint64_t value = 0;
  if (text.empty() || !std::all_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; })) {
    throw std::runtime_error("PGM dimensions and maximum value must be positive integers");
  }
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value == 0) {
    throw std::runtime_error("PGM integer is zero or outside the supported range");
  }
  return value;
}

struct Image
{
  int width, height;
  std::vector<unsigned char> pixels;
};

Image read_pgm(const std::filesystem::path & path)
{
  if (!std::filesystem::is_regular_file(path)) throw std::runtime_error("map image is not a regular file");
  const auto file_bytes = std::filesystem::file_size(path);
  if (file_bytes > kMaxCells + kMaxHeaderBytes) throw std::runtime_error("map image exceeds its size limit");
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot read map image");
  PgmHeader header(input);
  if (header.token() != "P5") throw std::runtime_error("only binary P5 map images are supported");
  const std::uint64_t width = positive_integer(header.token());
  const std::uint64_t height = positive_integer(header.token());
  const std::uint64_t maximum = positive_integer(header.token());
  if (width > kMaxDimension || height > kMaxDimension || width > kMaxCells / height) {
    throw std::runtime_error("map image dimensions exceed their supported bounds");
  }
  if (maximum != 255) throw std::runtime_error("only 8-bit PGM maps with maximum value 255 are supported");
  const auto cells = static_cast<std::size_t>(width * height);
  header.raster_separator(cells, file_bytes);
  const auto raster_at = input.tellg();
  if (raster_at < 0 || static_cast<std::uintmax_t>(raster_at) > file_bytes ||
      file_bytes - static_cast<std::uintmax_t>(raster_at) != cells) {
    throw std::runtime_error("map image pixel length does not match its dimensions");
  }
  Image result{static_cast<int>(width), static_cast<int>(height), std::vector<unsigned char>(cells)};
  input.read(reinterpret_cast<char *>(result.pixels.data()), static_cast<std::streamsize>(cells));
  if (!input) throw std::runtime_error("map image was truncated while reading");
  return result;
}
}  // namespace

GridSnapshot load_map(const std::filesystem::path & yaml_path, InflationParameters inflation)
{
  if (!std::filesystem::is_regular_file(yaml_path) ||
      std::filesystem::file_size(yaml_path) > kMaxYamlBytes) {
    throw std::runtime_error("map YAML must be a regular file within its size limit");
  }
  try {
    const YAML::Node yaml = YAML::LoadFile(yaml_path.string());
    if (!yaml.IsMap()) throw std::invalid_argument("map YAML must contain a mapping");
    const std::set<std::string> supported{
      "image", "mode", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"};
    std::set<std::string> seen;
    for (const auto & entry : yaml) {
      const auto key = entry.first.as<std::string>();
      if (supported.count(key) == 0 || !seen.insert(key).second) {
        throw std::invalid_argument("map YAML contains an unsupported or repeated field");
      }
    }
    const std::string mode = yaml["mode"] ? yaml["mode"].as<std::string>() : "trinary";
    if (mode != "trinary") throw std::invalid_argument("only trinary map mode is supported");
    const std::string image = yaml["image"].as<std::string>();
    const double resolution = yaml["resolution"].as<double>();
    const YAML::Node origin = yaml["origin"];
    if (!origin.IsSequence() || origin.size() != 3) throw std::invalid_argument("map origin must have x, y, yaw");
    const Point2 start{origin[0].as<double>(), origin[1].as<double>()};
    const double yaw = origin[2].as<double>();
    const int negate = yaml["negate"].as<int>();
    const double occupied = yaml["occupied_thresh"].as<double>();
    const double free = yaml["free_thresh"].as<double>();
    if (image.empty() || !std::isfinite(resolution) || resolution <= 0.0 || !std::isfinite(start.x) ||
        !std::isfinite(start.y) || !std::isfinite(yaw) || yaw != 0.0 ||
        (negate != 0 && negate != 1) || !std::isfinite(free) || !std::isfinite(occupied) ||
        free < 0.0 || free >= occupied || occupied > 1.0) {
      throw std::invalid_argument("invalid map geometry, thresholds, negate value or unsupported origin yaw");
    }
    const Image raster = read_pgm(yaml_path.parent_path() / image);
    std::vector<Cell> cells(raster.pixels.size());
    for (int y = 0; y < raster.height; ++y) {
      for (int x = 0; x < raster.width; ++x) {
        const auto pixel = raster.pixels[static_cast<std::size_t>(raster.height - 1 - y) * raster.width + x];
        const double probability = (negate ? static_cast<double>(pixel) : 255.0 - pixel) / 255.0;
        cells[static_cast<std::size_t>(y) * raster.width + x] = probability > occupied ? Cell::Occupied :
          probability < free ? Cell::Free : Cell::Unknown;
      }
    }
    return GridSnapshot(raster.width, raster.height, resolution, start, std::move(cells), {}, inflation);
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string("invalid map YAML: ") + error.what());
  }
}
}  // namespace mobile_manipulator_routing
