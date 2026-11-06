#pragma once
// The plant's demo server (MuJoCo plant spec Section 6.3, plan Task 9): a Session behind the stream. At connect the
// client gets a Hello {schema, site_checksum, scenarios}; its first Control must be {"cmd": "hello", "schema": N}
// (another schema: an error event, then the server closes). Commands: list_scenarios, reset {scenario, bag?, start?,
// seed?}, set_goal {goal}, run {speed} (1 paced at real time, 0 unpaced), pause, step {n}, move_body {name, center,
// yaw}. The session's frames stream out; errors come back as error events. docs/experiments/mujoco-plant/stream-protocol.md.
#include <atomic>

#include "mobile_manipulator_campaign/session.hpp"
#include "mobile_manipulator_campaign/stream_server.hpp"

namespace mobile_manipulator_campaign
{
struct DemoOptions
{
  SessionConfig session;
  int port = 7700;
  int unpaced_cycles = 25;  // B5 cycles per poll when running unpaced
  double hello_timeout_s = 5.0;  // a client that has not said hello by then is dropped (the slot is the only one)
};

class DemoServer
{
public:
  explicit DemoServer(DemoOptions options);
  int port() const { return server_.port(); }
  // One poll (at most timeout_ms), the client's commands, the cycles due, the frames out.
  void iterate(int timeout_ms);
  void serve(const std::atomic<bool> & stop);
  long dropped() const { return server_.dropped(); }

private:
  void handle(const nlohmann::json & command);
  void error(const std::string & reason);
  void send_event(const nlohmann::json & event);
  void forward();

  DemoOptions options_;
  StreamServer server_;
  Session session_;
  bool greeted_ = false, running_ = false, has_run_ = false;
  double speed_ = 1.0, wall0_ = 0.0, sim0_ = 0.0;
  double connected_at_ = 0.0;  // when the client connected (the hello timeout)
  std::string site_checksum_;
};
}  // namespace mobile_manipulator_campaign
