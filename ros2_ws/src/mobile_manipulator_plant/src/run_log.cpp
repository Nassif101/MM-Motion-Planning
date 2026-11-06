#include "mobile_manipulator_plant/run_log.hpp"

#include <zlib.h>

#include <stdexcept>
#include <vector>

#include "mobile_manipulator_plant/frames.hpp"

namespace mobile_manipulator_plant
{
namespace
{
constexpr char kMagic[8] = {'M', 'M', 'R', 'U', 'N', 'L', 'G', '1'};
constexpr size_t kChunk = 1 << 20;

void put_u32(std::ofstream & out, uint32_t v)
{
  char b[4];
  for (int i = 0; i < 4; ++i) b[i] = static_cast<char>((v >> (8 * i)) & 0xff);
  out.write(b, 4);
}
bool get_u32(std::ifstream & in, uint32_t & v)
{
  unsigned char b[4];
  if (!in.read(reinterpret_cast<char *>(b), 4)) return false;
  v = static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) | (static_cast<uint32_t>(b[2]) << 16) |
      (static_cast<uint32_t>(b[3]) << 24);
  return true;
}
}  // namespace

RunLogWriter::RunLogWriter(const std::string & path, const nlohmann::json & header) : out_(path, std::ios::binary)
{
  if (!out_) throw std::runtime_error("run log: cannot write " + path);
  out_.write(kMagic, sizeof kMagic);
  open_ = true;
  write(encode(FrameType::Hello, header, {}));
}

RunLogWriter::~RunLogWriter()
{
  try {
    close();
  } catch (...) {
  }
}

void RunLogWriter::write(const std::string & frame)
{
  if (!open_) throw std::logic_error("run log: write after close");
  raw_ += frame;
  if (raw_.size() >= kChunk) flush();
}

void RunLogWriter::flush()
{
  if (raw_.empty()) return;
  uLongf size = compressBound(raw_.size());
  std::vector<Bytef> packed(size);
  if (compress2(packed.data(), &size, reinterpret_cast<const Bytef *>(raw_.data()), raw_.size(), 6) != Z_OK)
    throw std::runtime_error("run log: zlib failed");
  put_u32(out_, static_cast<uint32_t>(raw_.size()));
  put_u32(out_, static_cast<uint32_t>(size));
  out_.write(reinterpret_cast<const char *>(packed.data()), static_cast<std::streamsize>(size));
  raw_.clear();
}

void RunLogWriter::close()
{
  if (!open_) return;
  flush();
  out_.close();
  open_ = false;
}

RunLogReader::RunLogReader(const std::string & path) : in_(path, std::ios::binary)
{
  char magic[8];
  if (!in_ || !in_.read(magic, 8) || std::string(magic, 8) != std::string(kMagic, 8))
    throw std::runtime_error("run log: " + path + " is not a run log");
  std::string frame;
  FrameType type{};
  std::vector<std::vector<float>> blocks;
  size_t consumed = 0;
  if (!next(frame) || !decode(frame, type, header_, blocks, consumed) || type != FrameType::Hello)
    throw std::runtime_error("run log: " + path + " has no header");
}

bool RunLogReader::load_chunk()
{
  uint32_t raw_len = 0, packed_len = 0;
  if (!get_u32(in_, raw_len)) return false;
  if (!get_u32(in_, packed_len)) throw std::runtime_error("run log: truncated chunk");
  std::vector<Bytef> packed(packed_len);
  if (!in_.read(reinterpret_cast<char *>(packed.data()), packed_len)) throw std::runtime_error("run log: truncated chunk");
  std::string raw(raw_len, '\0');
  uLongf size = raw_len;
  if (uncompress(reinterpret_cast<Bytef *>(raw.data()), &size, packed.data(), packed_len) != Z_OK || size != raw_len)
    throw std::runtime_error("run log: corrupt chunk");
  raw_ = raw_.substr(at_) + raw;
  at_ = 0;
  return true;
}

bool RunLogReader::next(std::string & frame)
{
  for (;;) {
    FrameType type{};
    nlohmann::json h;
    std::vector<std::vector<float>> blocks;
    size_t consumed = 0;
    const std::string_view rest = std::string_view(raw_).substr(at_);
    if (decode(rest, type, h, blocks, consumed)) {
      frame = std::string(rest.substr(0, consumed));
      at_ += consumed;
      return true;
    }
    if (!load_chunk()) return false;
  }
}
}  // namespace mobile_manipulator_plant
