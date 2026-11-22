#include "mobile_manipulator_campaign/demo_server.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iterator>
#include <regex>

#include "mobile_manipulator_medium/bench_setup.hpp"
#include "mobile_manipulator_plant/frames.hpp"

namespace mobile_manipulator_campaign
{
namespace mmp = mobile_manipulator_plant;
using json = nlohmann::json;

namespace
{
constexpr int kMaxStepCycles = 500;  // 10 s of plant time per step command
constexpr double kLogFlushS = 5.0;   // spec 4C: a running take's log reaches the disk at least this often
double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

std::string site_checksum()
{
  std::ifstream in(mobile_manipulator_medium::package_share("mobile_manipulator_plant") + "/models/site_v2.xml");
  const std::string text(std::istreambuf_iterator<char>(in), {});
  std::smatch m;
  return std::regex_search(text, m, std::regex(R"re(<text name="site_checksum" data="([^"]*)")re")) ? m[1].str() : "";
}
}  // namespace

DemoServer::DemoServer(DemoOptions options)
: options_(std::move(options)), server_(options_.port), session_([&] {
    SessionConfig c = options_.session;
    c.queue_frames = true;  // the stream takes them
    c.number_logs = true;   // spec 4C: one run log per session, never overwritten
    return c;
  }()),
  site_checksum_(site_checksum())
{
}

void DemoServer::serve(const std::atomic<bool> & stop)
{
  while (!stop) iterate(running_ ? (speed_ > 0.0 ? 2 : 0) : 20);
}

void DemoServer::iterate(int timeout_ms)
{
  if (server_.poll(timeout_ms)) {  // a new client: greet it; it says hello before anything else
    greeted_ = false;
    running_ = false;
    connected_at_ = now_s();
    reported_dropped_ = server_.dropped();
    server_.send(mmp::encode(mmp::FrameType::Hello,
                             {{"schema", mmp::kSchemaVersion}, {"server", "plant_demo_server"},
                              {"mode", options_.session.mode == mmp::PlantMode::Realistic ? "realistic" : "ideal"},
                              {"site_checksum", site_checksum_}, {"scenarios", Session::list_scenarios()}},
                             {}));
  }
  for (const json & c : server_.take_controls()) {
    try {
      handle(c);
    } catch (const std::exception & e) {
      error(e.what());
    }
    if (!server_.connected()) return;
  }
  if (!server_.connected()) {
    if (running_ || log_dirty_) session_.flush_log();  // spec 4C: a take's log is complete when its client leaves
    log_dirty_ = false;
    running_ = false;
    return;
  }
  if (!greeted_ && now_s() - connected_at_ > options_.hello_timeout_s) {  // free the only slot
    server_.disconnect();
    return;
  }
  std::string stopped;  // why the run stopped on its own, reported after the frames that led to it
  if (running_ && has_run_) {
    try {  // a failing cycle stops the run, not the server
      if (speed_ > 0.0) {  // paced: the plant's clock follows the wall clock times the speed
        const double target = sim0_ + speed_ * (now_s() - wall0_);
        const double until = now_s() + options_.max_catchup_s;  // then answer the client again
        while (live() && session_.time() < target && now_s() < until) tick();
        if (session_.time() < target) {  // behind: re-anchor rather than owe the lag (it would only grow)
          // Task 8: the timestep stays; the client hears the speed achieved since the last anchor, at most once a second
          const double wall = now_s() - wall0_;
          if (wall > 0.0 && now_s() - behind_reported_at_ >= 1.0) {
            send_event({{"event", "behind"}, {"requested_speed", speed_}, {"achieved_speed", (session_.time() - sim0_) / wall},
                        {"t", session_.time()}});
            behind_reported_at_ = now_s();
          }
          wall0_ = now_s();
          sim0_ = session_.time();
        }
      } else {
        for (int k = 0; k < options_.unpaced_cycles && live(); ++k) tick();
      }
    } catch (const std::exception & e) {
      running_ = false;
      error(std::string("the run stopped: ") + e.what());
      stopped = "error";
    }
    if (running_ && !live()) {  // a recorded run's goals have ended
      running_ = false;
      stopped = "goals_done";
    }
  }
  forward();
  if (!stopped.empty()) run_stopped(stopped);
  // spec 4C (Task 1 review I1): the log on disk at a stop and every few seconds while running
  if (running_) log_dirty_ = true;
  if (log_dirty_ && (!running_ || now_s() - log_flushed_at_ >= kLogFlushS)) {
    session_.flush_log();
    log_flushed_at_ = now_s();
    log_dirty_ = false;
  }
}

// A generated-route session runs on while idle (scheduled goals arrive, the plant holds); a recorded one ends.
bool DemoServer::live() const { return generated() || session_.active(); }
bool DemoServer::generated() const { return options_.session.generated_routes; }
void DemoServer::tick() { generated() ? (void)session_.advance() : (void)session_.step_cycle(); }

void DemoServer::acknowledge(const std::string & command, const json & c, const GoalAcknowledgement & ack)
{
  json event = {{"event", "goal_ack"}, {"command", command}, {"action", to_string(ack.action)}, {"goal_id", ack.goal_id},
                {"reason", ack.reason}};
  if (c.contains("client_request")) event["client_request"] = c.at("client_request");
  send_event(event);
}

void DemoServer::handle(const json & c)
{
  const std::string cmd = c.value("cmd", std::string());
  if (cmd == "hello") {
    const bool integer = c.contains("schema") && c.at("schema").is_number_integer();
    if (!integer || c.at("schema").get<int>() != mmp::kSchemaVersion) {
      error("schema " + (c.contains("schema") ? c.at("schema").dump() : std::string("missing")) + " (the server speaks " +
            std::to_string(mmp::kSchemaVersion) + ")");
      server_.disconnect();
      return;
    }
    greeted_ = true;
    // spec 4B-2: Prediction frames only for a client that accepts them (older readers reject unknown frame types)
    accepts_prediction_ = false;
    if (c.contains("accepts") && c.at("accepts").is_array())
      for (const auto & a : c.at("accepts"))
        if (a.is_string() && a.get<std::string>() == "prediction") accepts_prediction_ = true;
    // a client joining a run gets its session header and the latest goal and route records (Task 9 review I3)
    if (has_run_) {
      session_.take_frames();  // older frames the client never saw are dropped as before; the resync replaces them
      session_.take_events();
      for (const auto & f : session_.resync_frames()) server_.send(f);
    }
    return;
  }
  if (!greeted_) throw std::invalid_argument("say hello with the schema first");
  if (!has_run_ && cmd != "list_scenarios" && cmd != "reset" && cmd != "pause")
    throw std::invalid_argument(cmd + " before a reset (choose a scenario first)");
  if (cmd == "list_scenarios") {
    send_event({{"event", "scenarios"}, {"scenarios", Session::list_scenarios()}});
  } else if (cmd == "reset") {
    std::optional<mmp::Pose2> start;
    if (c.contains("start")) start = mmp::Pose2{c["start"].at(0).get<double>(), c["start"].at(1).get<double>(), c["start"].at(2).get<double>()};
    session_.reset(c.at("scenario").get<std::string>(), c.value("bag", std::string()), start, c.value("seed", 1));
    running_ = false;
    has_run_ = true;
    log_dirty_ = true;  // spec 4C: the new session's log, its Hello first, reaches the disk at once
  } else if (cmd == "submit_goal" || cmd == "schedule_goal") {  // Task 8: through the session's goal manager
    if (!generated()) throw std::invalid_argument(cmd + " needs a generated-route server (--routes lattice)");
    const auto & g = c.at("target");
    if (!c.contains("client_request") || !c.at("client_request").is_number_unsigned())
      throw std::invalid_argument(cmd + " needs a nonnegative integer client_request");
    GoalSubmission submission{session_.goal_manager().epoch(), "demo", c.at("client_request").get<std::uint64_t>(),
                              {g.at(0).get<double>(), g.at(1).get<double>(), g.at(2).get<double>()},
                              c.value("timeout_s", 120.0)};
    if (c.contains("epoch")) {  // Task 8 review I5: an unsigned integer, never a truncated number
      if (!c.at("epoch").is_number_unsigned()) throw std::invalid_argument(cmd + ": epoch must be a nonnegative integer");
      submission.epoch = c.at("epoch").get<std::uint64_t>();
    }
    const auto ack = cmd == "submit_goal" ? session_.submit_goal(submission)
                                          : session_.schedule_goal(submission, finite_at(c));
    acknowledge(cmd, c, ack);
  } else if (cmd == "request_replan") {
    if (!generated()) throw std::invalid_argument("request_replan needs a generated-route server (--routes lattice)");
    acknowledge(cmd, c, session_.request_replan());
  } else if (cmd == "cancel_goal") {  // spec 4A
    if (!generated()) throw std::invalid_argument("cancel_goal needs a generated-route server (--routes lattice)");
    acknowledge(cmd, c, session_.cancel_goal());
  } else if (cmd == "set_goal") {
    if (generated()) throw std::invalid_argument("set_goal is the recorded mode's; use submit_goal");
    const auto & g = c.at("goal");
    session_.set_goal({g.at(0).get<double>(), g.at(1).get<double>(), g.at(2).get<double>()});
  } else if (cmd == "run") {
    speed_ = c.value("speed", 1.0);
    if (speed_ < 0.0) throw std::invalid_argument("speed must not be negative");
    running_ = true;
    wall0_ = now_s();
    sim0_ = session_.time();
  } else if (cmd == "pause") {
    running_ = false;
    session_.flush_log();  // spec 4C: a paused take's log is on disk
  } else if (cmd == "step") {
    running_ = false;
    const int n = c.value("n", 1);
    if (n < 1 || n > kMaxStepCycles)  // a step runs inside the command: the server answers nothing meanwhile
      throw std::invalid_argument("step: n from 1 to " + std::to_string(kMaxStepCycles) + " (use run to go on)");
    for (int k = 0; k < n && live(); ++k) tick();
    log_dirty_ = true;
  } else if (cmd == "move_body") {
    const auto & p = c.at("center");
    session_.move_body(c.at("name").get<std::string>(), {p.at(0).get<double>(), p.at(1).get<double>(), p.at(2).get<double>()},
                       c.value("yaw", 0.0));
  } else {
    throw std::invalid_argument("unknown command " + cmd);
  }
}

double DemoServer::finite_at(const json & c)
{
  if (!c.contains("at_s") || !c.at("at_s").is_number() || c.at("at_s").is_boolean() ||
      !std::isfinite(c.at("at_s").get<double>()))
    throw std::invalid_argument("schedule_goal needs a finite at_s");
  return c.at("at_s").get<double>();
}

void DemoServer::error(const std::string & reason) { send_event({{"event", "error"}, {"reason", reason}}); }

void DemoServer::send_event(const json & event) { server_.send(mmp::encode(mmp::FrameType::Event, event, {})); }

// Spec 4A Task 5 review I2: the client learns that the run stopped without a pause command
void DemoServer::run_stopped(const std::string & reason)
{
  send_event({{"event", "run_stopped"}, {"reason", reason}, {"t", session_.time()}});
}

// The frame type is the u16 after the u32 length (stream-protocol.md, Frames)
bool DemoServer::is_prediction(const std::string & frame)
{
  if (frame.size() < 6) return false;
  const auto type = static_cast<uint16_t>(static_cast<uint8_t>(frame[4]) | (static_cast<uint8_t>(frame[5]) << 8));
  return type == static_cast<uint16_t>(mmp::FrameType::Prediction);
}

void DemoServer::forward()
{
  if (!greeted_) {  // a client gets session frames only after its hello (left from the previous client: dropped)
    session_.take_frames();
    session_.take_events();
    return;
  }
  for (const std::string & f : session_.take_frames())
    if (accepts_prediction_ || !is_prediction(f)) server_.send(f);
  session_.take_events();  // already among the frames
  if (server_.dropped() > reported_dropped_ && now_s() - dropped_reported_at_ >= 1.0) {  // at most once a second
    send_event({{"event", "dropped"}, {"frames", server_.dropped() - reported_dropped_}, {"t", session_.time()}});
    reported_dropped_ = server_.dropped();
    dropped_reported_at_ = now_s();
  }
}
}  // namespace mobile_manipulator_campaign
