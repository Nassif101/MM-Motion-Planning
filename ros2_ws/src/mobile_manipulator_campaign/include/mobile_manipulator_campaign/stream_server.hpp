#pragma once
// The demo stream's transport (MuJoCo plant spec Section 6.3): one TCP listener, one client at a time (a second waits
// until the first leaves), frames out with a non-blocking send behind a backlog cap (frames dropped and counted when the
// client is slow), Control frames in. POSIX sockets, no other dependency.
#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace mobile_manipulator_campaign
{
class StreamServer
{
public:
  explicit StreamServer(int port, size_t backlog_cap = 4u << 20);  // port 0: any free port
  ~StreamServer();
  StreamServer(const StreamServer &) = delete;
  StreamServer & operator=(const StreamServer &) = delete;
  int port() const { return port_; }
  // Accepts a waiting client when none is connected, reads the client's bytes and flushes the backlog, waiting at most
  // timeout_ms. True when a client connected during this call.
  bool poll(int timeout_ms);
  bool connected() const { return client_ >= 0; }
  // The client's Control frames (their JSON) since the last call. A malformed frame disconnects the client.
  std::vector<nlohmann::json> take_controls();
  // Queues a frame for the client: dropped (counted) without a client, or above the backlog cap unless it is a Hello,
  // Control or Event frame (those always queue). A client announcing a frame over 64 KiB is disconnected.
  void send(const std::string & frame);
  // Sends what it can of the backlog, then closes the client.
  void disconnect();
  long dropped() const { return dropped_; }

private:
  void flush();
  void drop_client();
  int listen_ = -1, client_ = -1, port_ = 0;
  size_t cap_;
  std::string in_, out_;
  std::vector<nlohmann::json> controls_;
  long dropped_ = 0;
};
}  // namespace mobile_manipulator_campaign
