#pragma once
// ROS-TCP-Connector wire protocol (v0.7.0), as spoken by Unity's ROS-TCP-Connector and
// the upstream ROS-TCP-Endpoint: every message is a frame of
//   uint32 little-endian destination length, destination (UTF-8),
//   uint32 little-endian payload length, payload.
// An empty destination is a keepalive; destinations starting with "__" are system
// commands whose payload is JSON followed by a NUL byte; any other destination is a topic
// and its payload is the CDR-serialized ROS 2 message, passed through unchanged.
#include <cctype>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace mobile_manipulator_control
{
constexpr const char * kProtocolVersion = "v0.7.0";

inline void append_u32(std::vector<uint8_t> & out, uint32_t value)
{
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(value >> (8 * i)));
}

inline uint32_t read_u32(const uint8_t * bytes)
{
  return static_cast<uint32_t>(bytes[0]) | static_cast<uint32_t>(bytes[1]) << 8 |
         static_cast<uint32_t>(bytes[2]) << 16 | static_cast<uint32_t>(bytes[3]) << 24;
}

inline std::vector<uint8_t> encode_frame(const std::string & destination, const uint8_t * payload, size_t size)
{
  std::vector<uint8_t> frame;
  frame.reserve(8 + destination.size() + size);
  append_u32(frame, static_cast<uint32_t>(destination.size()));
  frame.insert(frame.end(), destination.begin(), destination.end());
  append_u32(frame, static_cast<uint32_t>(size));
  frame.insert(frame.end(), payload, payload + size);
  return frame;
}

// System command to Unity: JSON payload (upstream sends it without a terminator).
inline std::vector<uint8_t> encode_command(const std::string & command, const std::string & json)
{
  return encode_frame(command, reinterpret_cast<const uint8_t *>(json.data()), json.size());
}

// System command payload from Unity: JSON followed by a NUL byte.
inline std::string command_json(const std::vector<uint8_t> & payload)
{
  size_t size = payload.size();
  while (size > 0 && payload[size - 1] == 0) --size;
  return std::string(payload.begin(), payload.begin() + static_cast<long>(size));
}

// Unity names messages "package/Type"; ROS 2 types are "package/msg/Type".
inline std::string ros_type(const std::string & unity_name, const std::string & kind = "msg")
{
  const auto slash = unity_name.find('/');
  if (slash == std::string::npos || unity_name.find('/', slash + 1) != std::string::npos) return unity_name;
  return unity_name.substr(0, slash) + "/" + kind + unity_name.substr(slash);
}

inline std::string unity_type(const std::string & ros_type_name)
{
  const auto marker = ros_type_name.find("/msg/");
  if (marker == std::string::npos) return ros_type_name;
  return ros_type_name.substr(0, marker) + ros_type_name.substr(marker + 4);
}

// Frame sync checks. ROS-TCP-Connector v0.7.0 builds queued system commands in one
// serializer shared by Unity's main thread and its connection thread, so registrations made
// while Unity connects can interleave on the wire: a command name paired with another
// command's JSON, or a JSON payload read as a destination. These checks recognise that.

// A topic name: letters, digits, '_', '/' and '~' only.
inline bool valid_topic_name(const std::string & name)
{
  if (name.empty()) return false;
  for (const unsigned char c : name) {
    if (!std::isalnum(c) && c != '_' && c != '/' && c != '~') return false;
  }
  return true;
}

// Unity's JsonUtility writes every field of a command, so each command has a fixed key set;
// commands without a known key set are left to the command handler.
inline bool command_fits(const std::string & command, const nlohmann::json & params)
{
  static const std::map<std::string, std::set<std::string>> keys = {
    {"subscribe", {"topic", "message_name"}},
    {"publish", {"topic", "message_name", "queue_size", "latch"}},
    {"ros_service", {"topic", "message_name"}},
    {"unity_service", {"topic", "message_name"}},
    {"topic_list", {}},
  };
  if (!params.is_object()) return false;
  const auto expected = keys.find(command);
  if (expected == keys.end()) return true;
  std::set<std::string> present;
  for (const auto & item : params.items()) present.insert(item.key());
  if (present != expected->second) return false;
  return !params.contains("topic") ||
         (params["topic"].is_string() && valid_topic_name(params["topic"].get<std::string>()));
}
}  // namespace mobile_manipulator_control
