#include "mobile_manipulator_plant/frames.hpp"

#include <cstring>
#include <stdexcept>

namespace mobile_manipulator_plant
{
namespace
{
void put_u32(std::string & out, uint32_t v)
{
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<char>((v >> (8 * i)) & 0xff));
}
void put_u16(std::string & out, uint16_t v)
{
  out.push_back(static_cast<char>(v & 0xff));
  out.push_back(static_cast<char>((v >> 8) & 0xff));
}
uint32_t get_u32(std::string_view b, size_t at)
{
  uint32_t v = 0;
  for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(static_cast<unsigned char>(b[at + i])) << (8 * i);
  return v;
}
uint16_t get_u16(std::string_view b, size_t at)
{
  return static_cast<uint16_t>(static_cast<unsigned char>(b[at]) | (static_cast<unsigned char>(b[at + 1]) << 8));
}
bool known(uint16_t t) { return t == 1 || t == 2 || t == 3 || t == 10 || t == 11 || t == 12; }
}  // namespace

std::string encode(FrameType type, const nlohmann::json & header, const std::vector<std::vector<float>> & blocks)
{
  const std::string text = header.dump();
  std::string body;
  put_u16(body, static_cast<uint16_t>(type));
  put_u32(body, static_cast<uint32_t>(text.size()));
  body += text;
  put_u32(body, static_cast<uint32_t>(blocks.size()));
  for (const auto & block : blocks) {
    put_u32(body, static_cast<uint32_t>(block.size()));
    for (float f : block) {
      uint32_t bits;
      std::memcpy(&bits, &f, sizeof bits);
      put_u32(body, bits);
    }
  }
  std::string out;
  put_u32(out, static_cast<uint32_t>(body.size()));
  return out + body;
}

bool decode(std::string_view bytes, FrameType & type, nlohmann::json & header, std::vector<std::vector<float>> & blocks,
            size_t & consumed)
{
  if (bytes.size() < 4) return false;
  const size_t length = get_u32(bytes, 0);
  if (bytes.size() < 4 + length) return false;
  const std::string_view body = bytes.substr(4, length);
  auto need = [&](size_t at, size_t n) {
    if (at + n > body.size()) throw std::runtime_error("frame: lengths exceed the frame");
  };
  size_t at = 0;
  need(at, 6);
  const uint16_t t = get_u16(body, at);
  if (!known(t)) throw std::runtime_error("frame: unknown type " + std::to_string(t));
  const size_t json_len = get_u32(body, at + 2);
  at += 6;
  need(at, json_len);
  nlohmann::json h;
  try {
    h = nlohmann::json::parse(body.substr(at, json_len));
  } catch (const nlohmann::json::exception & e) {
    throw std::runtime_error(std::string("frame: bad JSON header: ") + e.what());
  }
  at += json_len;
  need(at, 4);
  const size_t n_blocks = get_u32(body, at);
  at += 4;
  std::vector<std::vector<float>> b;
  for (size_t k = 0; k < n_blocks; ++k) {
    need(at, 4);
    const size_t n = get_u32(body, at);
    at += 4;
    need(at, 4 * n);
    std::vector<float> block(n);
    for (size_t i = 0; i < n; ++i) {
      const uint32_t bits = get_u32(body, at + 4 * i);
      std::memcpy(&block[i], &bits, sizeof bits);
    }
    at += 4 * n;
    b.push_back(std::move(block));
  }
  if (at != body.size()) throw std::runtime_error("frame: trailing bytes");
  type = static_cast<FrameType>(t);
  header = std::move(h);
  blocks = std::move(b);
  consumed = 4 + length;
  return true;
}
}  // namespace mobile_manipulator_plant
