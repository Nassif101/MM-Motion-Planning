// The demo server (MuJoCo plant plan Task 9, spec Section 6.3): a session behind one TCP client, frames out, control in.
#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iterator>
#include <thread>

#include "mobile_manipulator_campaign/demo_server.hpp"
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
