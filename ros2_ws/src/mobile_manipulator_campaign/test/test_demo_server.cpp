// The demo server (MuJoCo plant plan Task 9, spec Section 6.3): a session behind one TCP client, frames out, control in.
#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iterator>
#include <thread>

#include "mobile_manipulator_campaign/demo_server.hpp"
#include "mobile_manipulator_campaign/goal_records.hpp"
#include "mobile_manipulator_campaign/stream_server.hpp"
#include "mobile_manipulator_plant/frames.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;
using json = nlohmann::json;

namespace
{
// The server on its own thread for the test's life
struct Running
{
  explicit Running(mmc::DemoOptions o) : server(std::move(o)), thread([this] { server.serve(stop); }) {}
  ~Running()
  {
    stop = true;
    thread.join();
  }
  mmc::DemoServer server;
  std::atomic<bool> stop{false};
  std::thread thread;
};

mmc::DemoOptions options()
{
  mmc::DemoOptions o;
  o.port = 0;  // any free port
  return o;
}

struct Client
{
  explicit Client(int port)
  {
    fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(static_cast<uint16_t>(port));
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    connected = connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof a) == 0;
  }
  ~Client() { close(fd); }
  void control(const json & c)
  {
    const std::string f = mmp::encode(mmp::FrameType::Control, c, {});
    ASSERT_EQ(::send(fd, f.data(), f.size(), 0), static_cast<ssize_t>(f.size()));
  }
  // The next frame within `timeout_s`; false on timeout or a closed connection (closed set)
  bool next(mmp::FrameType & type, json & header, double timeout_s = 20.0)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    for (;;) {
      std::vector<std::vector<float>> blocks;
      size_t consumed = 0;
      if (mmp::decode(buffer, type, header, blocks, consumed)) {
        buffer.erase(0, consumed);
        return true;
      }
      if (std::chrono::steady_clock::now() > end) return false;
      pollfd p{fd, POLLIN, 0};
      if (::poll(&p, 1, 50) <= 0) continue;
      char chunk[65536];
      const ssize_t n = recv(fd, chunk, sizeof chunk, 0);
      if (n <= 0) {
        closed = true;
        return false;
      }
      buffer.append(chunk, static_cast<size_t>(n));
    }
  }
  // The next frame of `wanted` type (skipping others)
  bool next_of(mmp::FrameType wanted, json & header, double timeout_s = 20.0)
  {
    mmp::FrameType type{};
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < end)
      if (next(type, header, 1.0) && type == wanted) return true;
      else if (closed) return false;
    return false;
  }
  int fd = -1;
  bool connected = false, closed = false;
  std::string buffer;
};
}  // namespace

TEST(DemoServer, HelloOnConnect)
{
  Running r(options());
  Client c(r.server.port());
  ASSERT_TRUE(c.connected);
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  EXPECT_EQ(h.at("schema"), mmp::kSchemaVersion);
  EXPECT_FALSE(h.at("site_checksum").get<std::string>().empty());
  EXPECT_FALSE(h.at("scenarios").empty());
}

TEST(DemoServer, ResetAndRunStreamsState)
{
  Running r(options());
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
  c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}, {"seed", 1}});
  c.control({{"cmd", "run"}, {"speed", 0.0}});  // unpaced
  int states = 0;
  while (states < 20 && c.next_of(mmp::FrameType::State, h)) ++states;
  EXPECT_EQ(states, 20);
  EXPECT_GT(h.at("t").get<double>(), 0.0);
}

TEST(DemoServer, SurvivesClientDisconnect)
{
  Running r(options());
  {
    Client c(r.server.port());
    json h;
    ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
    c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
    c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}, {"seed", 1}});
    c.control({{"cmd", "run"}, {"speed", 0.0}});
    ASSERT_TRUE(c.next_of(mmp::FrameType::State, h));
  }  // closes mid-stream
  Client second(r.server.port());
  json h;
  ASSERT_TRUE(second.next_of(mmp::FrameType::Hello, h));
  EXPECT_EQ(h.at("schema"), mmp::kSchemaVersion);
}

TEST(DemoServer, RejectsWrongSchema)
{
  Running r(options());
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion + 1}});
  ASSERT_TRUE(c.next_of(mmp::FrameType::Event, h));
  EXPECT_EQ(h.at("event"), "error");
  mmp::FrameType type{};
  EXPECT_FALSE(c.next(type, h, 5.0));  // the server closed the connection
  EXPECT_TRUE(c.closed);
}

TEST(DemoServer, CommandsBeforeHelloAreRejected)
{
  Running r(options());
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}});
  ASSERT_TRUE(c.next_of(mmp::FrameType::Event, h));
  EXPECT_EQ(h.at("event"), "error");
}

TEST(DemoServer, CommandsBeforeResetAreRejected)
{
  Running r(options());
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
  for (const json & cmd : {json{{"cmd", "set_goal"}, {"goal", {1.0, 2.0, 0.0}}},
                           json{{"cmd", "move_body"}, {"name", "reserved/0"}, {"center", {0.0, 0.0, 0.9}}},
                           json{{"cmd", "run"}}, json{{"cmd", "step"}}}) {
    c.control(cmd);
    ASSERT_TRUE(c.next_of(mmp::FrameType::Event, h)) << cmd.dump();
    EXPECT_EQ(h.at("event"), "error") << cmd.dump();
  }
  c.control({{"cmd", "list_scenarios"}});  // still serving
  ASSERT_TRUE(c.next_of(mmp::FrameType::Event, h));
  EXPECT_EQ(h.at("event"), "scenarios");
}

TEST(DemoServer, MoveBodyPlacesAReservedBody)
{
  Running r(options());
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
  c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}});
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  ASSERT_EQ(h.at("reserved").size(), 4u);  // the session Hello lists them with their sizes
  c.control({{"cmd", "move_body"}, {"name", "reserved/1"}, {"center", {-3.0, -4.0, 0.9}}, {"yaw", 0.5}});
  c.control({{"cmd", "step"}, {"n", 1}});
  mmp::FrameType type{};
  bool state = false;
  while (!state && c.next(type, h, 5.0)) {
    ASSERT_FALSE(type == mmp::FrameType::Event && h.at("event") == "error") << h.dump();
    state = type == mmp::FrameType::State;
  }
  ASSERT_TRUE(state);
  ASSERT_EQ(h.at("bodies").size(), 1u);
  EXPECT_EQ(h.at("bodies")[0][0], "reserved/1");
  EXPECT_NEAR(h.at("bodies")[0][1].get<double>(), -3.0, 1e-9);
}

TEST(DemoServer, StaysResponsiveWhenBehindRealTime)
{
  // a pace the host cannot hold (1000x): the server keeps answering instead of catching up for seconds
  Running r(options());
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
  c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}});
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "run"}, {"speed", 1000.0}});
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto t0 = std::chrono::steady_clock::now();
  c.control({{"cmd", "list_scenarios"}});
  bool answered = false;
  while (!answered && c.next_of(mmp::FrameType::Event, h, 10.0)) answered = h.at("event") == "scenarios";
  ASSERT_TRUE(answered);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 1.0);
}

TEST(DemoServer, DropsAClientThatNeverSaysHello)
{
  auto o = options();
  o.hello_timeout_s = 0.3;
  Running r(o);
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  mmp::FrameType type{};
  while (c.next(type, h, 3.0)) {
  }
  EXPECT_TRUE(c.closed);  // the slot is free for the next client
  Client next(r.server.port());
  EXPECT_TRUE(next.next_of(mmp::FrameType::Hello, h, 3.0));
}

// the next error event, skipping other events (a reset's)
bool next_error(Client & c, json & h)
{
  for (int k = 0; k < 50; ++k)
    if (!c.next_of(mmp::FrameType::Event, h, 5.0)) return false;
    else if (h.value("event", "") == "error") return true;
  return false;
}

TEST(DemoServer, ALatticeServerTakesGoalsThroughTheGoalManager)
{
  // Spec 2 Task 8: a generated-route server; a submitted goal is acknowledged, planned and reached, and the session
  // keeps running while idle (its state frames go on after the goal)
  auto o = options();
  o.session.generated_routes = true;
  o.session.auto_goal = false;
  Running r(o);
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
  c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}, {"seed", 1}});
  c.control({{"cmd", "set_goal"}, {"goal", {0.0, 0.0, 0.0}}});  // the recorded mode's command is refused here
  ASSERT_TRUE(next_error(c, h));
  // the open yard's start (-6, -3.5, -30 deg): 2 m ahead
  const double yaw = -0.523599;
  c.control({{"cmd", "submit_goal"}, {"client_request", 1},
             {"target", {-6.0 + 2.0 * std::cos(yaw), -3.5 + 2.0 * std::sin(yaw), yaw}}, {"timeout_s", 60.0}});
  c.control({{"cmd", "run"}, {"speed", 0.0}});
  bool acknowledged = false, reached = false;
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(240);
  while (!reached && std::chrono::steady_clock::now() < end && c.next_of(mmp::FrameType::Event, h, 5.0)) {
    if (h.value("event", "") == "goal_ack") {
      EXPECT_EQ(h.at("action"), "accepted");
      EXPECT_EQ(h.at("client_request"), 1);
      acknowledged = true;
    }
    reached = h.value("event", "") == "goal_reached";
  }
  EXPECT_TRUE(acknowledged);
  ASSERT_TRUE(reached);
  double t0 = -1.0;
  ASSERT_TRUE(c.next_of(mmp::FrameType::State, h));
  t0 = h.at("t").get<double>();
  for (int k = 0; k < 10; ++k) ASSERT_TRUE(c.next_of(mmp::FrameType::State, h));
  EXPECT_GT(h.at("t").get<double>(), t0);  // idle, still stepping
}

TEST(DemoServer, PacedAndUnpacedLatticeRunsReachTheSameDecisions)
{
  // Task 8: pacing changes the wall clock only; the same goal is reached at the same simulation time
  auto reached_at = [](double speed) {
    auto o = options();
    o.session.generated_routes = true;
    o.session.auto_goal = false;
    Running r(o);
    Client c(r.server.port());
    json h;
    EXPECT_TRUE(c.next_of(mmp::FrameType::Hello, h));
    c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
    c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}, {"seed", 1}});
    const double yaw = -0.523599;
    c.control({{"cmd", "submit_goal"}, {"client_request", 1},
               {"target", {-6.0 + 1.5 * std::cos(yaw), -3.5 + 1.5 * std::sin(yaw), yaw}}});
    c.control({{"cmd", "run"}, {"speed", speed}});
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(240);
    while (std::chrono::steady_clock::now() < end && c.next_of(mmp::FrameType::Event, h, 10.0))
      if (h.value("event", "") == "goal_reached") return h.at("t").get<double>();
    return -1.0;
  };
  const double unpaced = reached_at(0.0), paced = reached_at(8.0);
  ASSERT_GT(unpaced, 0.0);
  EXPECT_EQ(paced, unpaced);
}

TEST(DemoServer, PausingAndSteppingALatticeRunDoesNotChangeItsDecisions)
{
  // Spec 2 Task 14: pause, single steps and a resumed run change the wall clock only; the goal is reached at the same
  // simulation time as in an uninterrupted unpaced run
  auto reached_at = [](bool interrupt) {
    auto o = options();
    o.session.generated_routes = true;
    o.session.auto_goal = false;
    Running r(o);
    Client c(r.server.port());
    json h;
    EXPECT_TRUE(c.next_of(mmp::FrameType::Hello, h));
    c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
    c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}, {"seed", 1}});
    const double yaw = -0.523599;
    c.control({{"cmd", "submit_goal"}, {"client_request", 1},
               {"target", {-6.0 + 1.5 * std::cos(yaw), -3.5 + 1.5 * std::sin(yaw), yaw}}});
    if (interrupt) {
      c.control({{"cmd", "step"}, {"n", 10}});  // the search and the adoption run inside these steps
      EXPECT_TRUE(c.next_of(mmp::FrameType::Route, h, 30.0));
      c.control({{"cmd", "run"}, {"speed", 0.0}});
      EXPECT_TRUE(c.next_of(mmp::FrameType::State, h, 30.0));
      c.control({{"cmd", "pause"}});
      for (int k = 0; k < 4; ++k) c.control({{"cmd", "step"}, {"n", 7}});
    }
    c.control({{"cmd", "run"}, {"speed", 0.0}});
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(240);
    while (std::chrono::steady_clock::now() < end && c.next_of(mmp::FrameType::Event, h, 10.0))
      if (h.value("event", "") == "goal_reached") return h.at("t").get<double>();
    return -1.0;
  };
  const double straight = reached_at(false), interrupted = reached_at(true);
  ASSERT_GT(straight, 0.0);
  EXPECT_EQ(interrupted, straight);
}

TEST(DemoServer, AFractionalEpochIsRefusedAndTheActiveGoalStays)
{
  // Task 8 review I5: epoch 1.5 was truncated to the current epoch 1 and replaced the active goal
  auto o = options();
  o.session.generated_routes = true;
  o.session.auto_goal = false;
  Running r(o);
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
  c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}, {"seed", 1}});
  const json target = {-4.5, -4.0, -0.5};
  c.control({{"cmd", "submit_goal"}, {"client_request", 1}, {"target", target}});
  c.control({{"cmd", "submit_goal"}, {"client_request", 2}, {"target", target}, {"epoch", 1.5}});
  c.control({{"cmd", "schedule_goal"}, {"client_request", 3}, {"target", target}, {"epoch", 1.5}, {"at_s", 5.0}});
  c.control({{"cmd", "schedule_goal"}, {"client_request", 4}, {"target", target}, {"at_s", "soon"}});
  int acks = 0, errors = 0;
  bool superseded = false;
  for (int k = 0; k < 60 && (acks < 1 || errors < 3); ++k) {
    if (!c.next_of(mmp::FrameType::Event, h, 5.0)) break;
    const std::string e = h.value("event", "");
    acks += e == "goal_ack";
    errors += e == "error";
    superseded = superseded || e == "goal_superseded";
  }
  EXPECT_EQ(acks, 1);
  EXPECT_EQ(errors, 3);
  EXPECT_FALSE(superseded);
}

TEST(DemoServer, AReconnectingClientGetsTheSessionHeaderAndTheLatestGoalAndRoute)
{
  // Task 9 review I3: the goal/route records a client missed are replayed after its hello
  auto o = options();
  o.session.generated_routes = true;
  o.session.auto_goal = false;
  Running r(o);
  json h;
  {
    Client first(r.server.port());
    ASSERT_TRUE(first.next_of(mmp::FrameType::Hello, h));
    first.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
    first.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}, {"seed", 1}});
    const double yaw = -0.523599;
    first.control({{"cmd", "submit_goal"}, {"client_request", 1},
                   {"target", {-6.0 + 2.0 * std::cos(yaw), -3.5 + 2.0 * std::sin(yaw), yaw}}});
    first.control({{"cmd", "run"}, {"speed", 0.0}});
    ASSERT_TRUE(first.next_of(mmp::FrameType::Route, h, 60.0));
    first.control({{"cmd", "pause"}});
  }  // the first client leaves
  Client second(r.server.port());
  ASSERT_TRUE(second.next_of(mmp::FrameType::Hello, h));  // the server Hello
  second.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
  bool session_hello = false, goal = false, route = false;
  mmp::FrameType type{};
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (!(session_hello && goal && route) && std::chrono::steady_clock::now() < end && second.next(type, h, 1.0)) {
    session_hello = session_hello || (type == mmp::FrameType::Hello && h.contains("scenario"));
    goal = goal || type == mmp::FrameType::Goal;
    route = route || (type == mmp::FrameType::Route && h.at("goal_id") == 1);
  }
  EXPECT_TRUE(session_hello);
  EXPECT_TRUE(goal);
  EXPECT_TRUE(route);
}

TEST(DemoServer, GoalCommandsNeedALatticeServer)
{
  Running r(options());
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
  c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}, {"seed", 1}});
  c.control({{"cmd", "submit_goal"}, {"client_request", 1}, {"target", {0.0, 0.0, 0.0}}});
  ASSERT_TRUE(next_error(c, h));
  EXPECT_NE(h.at("reason").get<std::string>().find("lattice"), std::string::npos);
}

TEST(StreamServer, DropsAnOversizedControlFrame)
{
  // a client announcing a 4 GiB frame is cut off, not buffered
  mmc::StreamServer s(0);
  Client c(s.port());
  for (int i = 0; i < 20 && !s.connected(); ++i) s.poll(50);
  ASSERT_TRUE(s.connected());
  const uint32_t length = 0xFFFFFFF0u;
  std::string bytes(reinterpret_cast<const char *>(&length), 4);
  bytes += std::string(100000, 'x');
  ASSERT_EQ(::send(c.fd, bytes.data(), bytes.size(), 0), static_cast<ssize_t>(bytes.size()));
  for (int i = 0; i < 40 && s.connected(); ++i) s.poll(50);
  EXPECT_FALSE(s.connected());
}

TEST(StreamServer, KeepsHelloAndEventFramesOverTheCap)
{
  // a slow client loses State frames, never the Hello or an Event
  mmc::StreamServer s(0, 64u << 10);
  Client c(s.port());
  for (int i = 0; i < 20 && !s.connected(); ++i) s.poll(50);
  ASSERT_TRUE(s.connected());
  const std::vector<float> big(16000, 1.0f);
  for (int i = 0; i < 400; ++i) s.send(mmp::encode(mmp::FrameType::State, {{"i", i}}, {big}));
  ASSERT_GT(s.dropped(), 0);
  // fill the backlog to within one small frame of the cap: the Event below is larger than the room left
  for (long before = s.dropped(); s.dropped() == before;) s.send(mmp::encode(mmp::FrameType::State, {{"i", 0}}, {}));
  const long dropped = s.dropped();
  s.send(mmp::encode(mmp::FrameType::Goal, {{"event", "goal_accepted"}}, {}));  // Task 9 review I3: kept too
  s.send(mmp::encode(mmp::FrameType::Route, {{"route_id", 1}}, {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}}));
  s.send(mmp::encode(mmp::FrameType::Event, {{"event", "goal_reached"}, {"note", std::string(300, 'n')}}, {}));
  EXPECT_EQ(s.dropped(), dropped);  // queued over the cap
  bool got = false;
  mmp::FrameType type{};
  json h;
  int frames = 0;  // the frames queued before the Event (thousands of small ones fill the backlog), then the Event
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (!got && !c.closed && std::chrono::steady_clock::now() < end) {
    s.poll(0);
    if (c.next(type, h, 0.01)) {
      ++frames;
      got = type == mmp::FrameType::Event && h.at("event") == "goal_reached";
    }
  }
  EXPECT_TRUE(got) << "frames " << frames << " closed " << c.closed << " connected " << s.connected() << " dropped " << s.dropped();
}

TEST(DemoServer, ANonIntegerSchemaDisconnects)
{
  Running r(options());
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", "1"}});
  ASSERT_TRUE(c.next_of(mmp::FrameType::Event, h));
  EXPECT_EQ(h.at("event"), "error");
  mmp::FrameType type{};
  while (c.next(type, h, 2.0)) {
  }
  EXPECT_TRUE(c.closed);
}

TEST(DemoServer, AHugeStepIsRefused)
{
  // step n runs inside one command: a huge n would block the server for hours
  Running r(options());
  Client c(r.server.port());
  json h;
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  c.control({{"cmd", "hello"}, {"schema", mmp::kSchemaVersion}});
  c.control({{"cmd", "reset"}, {"scenario", "v2_open_yard_nav"}});
  ASSERT_TRUE(c.next_of(mmp::FrameType::Hello, h));
  const auto t0 = std::chrono::steady_clock::now();
  c.control({{"cmd", "step"}, {"n", 100000000}});
  bool error = false;
  while (!error && c.next_of(mmp::FrameType::Event, h, 5.0)) error = h.at("event") == "error";
  EXPECT_TRUE(error);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 5.0);
}

TEST(Frames, MatchTheCommittedFixture)
{
  // the wire's reference bytes (tools/plant/fixtures/frame_v1.bin): the Python codec checks itself against the same file
  const std::string frame = mmp::encode(mmp::FrameType::State, {{"t", 1.25}, {"name", "fixture"}, {"list", {1, 2, 3}}},
                                        {{1.5f, -2.0f}, {}, {3.25f}});
  const std::string path = std::string(REPO_ROOT) + "/tools/plant/fixtures/frame_v1.bin";
  if (std::getenv("UPDATE_FIXTURE")) std::ofstream(path, std::ios::binary) << frame;
  std::ifstream in(path, std::ios::binary);
  ASSERT_TRUE(in) << path;
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(in), {}), frame);
}

namespace
{
// Spec 2 Task 9: fixed inputs for the schema-3 record fixtures (tools/plant/fixtures/goal_v3.bin, route_v3.bin)
mmc::GoalEvent fixture_goal_event()
{
  mmc::GoalEvent e;
  e.sequence = 7;
  e.input_sequence = 3;
  e.kind = mmc::GoalEventKind::Accepted;
  e.applied_at = 2.5;
  e.scheduled_for = 2.5;
  e.client = "fixture";
  e.client_request = 11;
  e.epoch = 2;
  e.requested_epoch = 2;
  e.reason = "";
  mmc::GoalRecord g;
  g.submission = {2, "fixture", 11, {1.5, -2.25, 0.5}, 60.0};
  g.search_token.epoch = 2;
  g.search_token.goal = 4;
  g.search_token.request = 5;
  g.search_token.map = 1;
  g.route_id = 0;
  g.accepted_at = 2.5;
  g.deadline = 62.5;
  g.state = mmc::GoalState::Planning;
  e.goal = g;
  e.request = g.submission;
  return e;
}

// a request refused at submission: no goal record, the submitted target and timeout kept (Task 9 review I4)
mmc::GoalEvent fixture_rejected_event()
{
  mmc::GoalEvent e;
  e.sequence = 8;
  e.input_sequence = 4;
  e.kind = mmc::GoalEventKind::Rejected;
  e.applied_at = 3.0;
  e.client = "fixture";
  e.client_request = 12;
  e.epoch = 2;
  e.requested_epoch = 2;
  e.reason = "destination footprint is occupied, unknown, or outside the map";
  e.request = mmc::GoalSubmission{2, "fixture", 12, {101.0, -2.25, 0.5}, 20.0};
  return e;
}

mmc::RouteRecord fixture_route()
{
  mmc::RouteRecord r;
  r.t = 3.0;
  r.epoch = 2;
  r.goal_id = 4;
  r.request_id = 5;
  r.route_id = 1;
  r.requested_goal = {1.5, -2.25, 0.5};
  r.resolved_goal = {1.5, -2.25, 0.25};
  r.planner = "smac_lattice_1.3.13_port";
  r.path_kind = "translating";
  r.smoothed = true;
  r.turn_first = false;
  r.length_m = 2.5;
  r.expansions = 42;
  r.cell_tests = 1000;
  r.path = {{0.0, 0.0, 0.0}, {0.5, -0.5, -0.75}, {1.5, -2.25, 0.25}};
  r.route = {{0.0, 0.0}, {0.5, -0.5}, {1.5, -2.25}};
  return r;
}

void expect_fixture(const std::string & name, const std::string & frame)
{
  const std::string path = std::string(REPO_ROOT) + "/tools/plant/fixtures/" + name;
  if (std::getenv("UPDATE_FIXTURE")) std::ofstream(path, std::ios::binary) << frame;
  std::ifstream in(path, std::ios::binary);
  ASSERT_TRUE(in) << path;
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(in), {}), frame) << name;
}
}  // namespace

TEST(Frames, GoalAndRouteRecordsMatchTheCommittedFixtures)
{
  expect_fixture("goal_v3.bin", mmp::encode(mmp::FrameType::Goal, mmc::goal_record(fixture_goal_event(), 2.5), {}));
  expect_fixture("goal_rejected_v3.bin",
                 mmp::encode(mmp::FrameType::Goal, mmc::goal_record(fixture_rejected_event(), 3.0), {}));
  const auto [header, blocks] = mmc::route_record(fixture_route());
  expect_fixture("route_v3.bin", mmp::encode(mmp::FrameType::Route, header, blocks));
  // and they decode as written
  mmp::FrameType type{};
  json h;
  std::vector<std::vector<float>> b;
  size_t used = 0;
  const std::string route = mmp::encode(mmp::FrameType::Route, header, blocks);
  ASSERT_TRUE(mmp::decode(route, type, h, b, used));
  EXPECT_EQ(type, mmp::FrameType::Route);
  ASSERT_EQ(b.size(), 2u);
  EXPECT_EQ(b[0].size(), 9u);
  EXPECT_EQ(b[1].size(), 6u);
  EXPECT_EQ(h.at("path_poses"), 3);
}

TEST(GeneratedRecords, AGeneratedSessionWritesGoalAndRouteFrames)
{
  mmc::SessionConfig c;
  c.queue_frames = true;
  c.generated_routes = true;
  c.auto_goal = false;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto & x = s.start_state();
  ASSERT_EQ(s.submit_goal({s.goal_manager().epoch(), "test", 1,
                           {x(0) + 2.0 * std::cos(x(2)), x(1) + 2.0 * std::sin(x(2)), x(2)}, 60.0}).action,
            mmc::GoalAction::Accepted);
  bool hello = false, goal_accepted = false, route = false, reached = false;
  for (int k = 0; k < 3000 && !reached; ++k) {
    s.advance();
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      ASSERT_TRUE(mmp::decode(f, type, h, b, used));
      if (type == mmp::FrameType::Hello) {
        hello = true;
        EXPECT_EQ(h.at("schema"), 3);
        EXPECT_EQ(h.at("capabilities"), json::array({"goal_frames", "route_frames"}));
      }
      if (type == mmp::FrameType::Goal && h.at("event") == "goal_accepted") goal_accepted = true;
      if (type == mmp::FrameType::Goal && h.at("event") == "goal_reached") reached = true;
      if (type == mmp::FrameType::Route) {
        route = true;
        EXPECT_EQ(h.at("goal_id"), 1);
        EXPECT_EQ(b.at(0).size(), 3 * h.at("path_poses").get<size_t>());
        EXPECT_EQ(b.at(1).size(), 2 * h.at("route_points").get<size_t>());
      }
    }
  }
  EXPECT_TRUE(goal_accepted);
  EXPECT_TRUE(route);
  EXPECT_TRUE(reached);
  EXPECT_TRUE(hello);  // queued at the reset, taken with the first tick's frames
}
