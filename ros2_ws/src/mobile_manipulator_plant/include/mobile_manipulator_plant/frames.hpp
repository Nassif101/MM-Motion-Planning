#pragma once
// Frames: one schema for the demo server's stream and the run log (MuJoCo plant spec Section 6.2). A frame is a JSON
// header plus float blocks (poses, plans, scan points), framed for a byte stream:
//   [u32 length][u16 type][u32 json_len][json][u32 n_blocks]{[u32 n_floats][float32 ...]}
// little-endian; `length` counts the bytes after itself.
#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace mobile_manipulator_plant
{
constexpr int kSchemaVersion = 1;

enum class FrameType : uint16_t
{
  Hello = 1,    // the session or log header
  Control = 2,  // a client's command (demo server)
  Event = 3,    // goal_reached, failed, contact, timeout, fallback
  State = 10,   // the plant's state each B5 cycle
  Plans = 11,   // a medium plan
  Scan = 12     // a lidar scan
};

std::string encode(FrameType type, const nlohmann::json & header, const std::vector<std::vector<float>> & blocks);

// The first frame of `bytes`. false (the outputs untouched) while `bytes` holds less than a whole frame;
// std::runtime_error for a malformed frame (unknown type, inconsistent lengths, bad JSON).
bool decode(std::string_view bytes, FrameType & type, nlohmann::json & header, std::vector<std::vector<float>> & blocks,
            size_t & consumed);
}  // namespace mobile_manipulator_plant
