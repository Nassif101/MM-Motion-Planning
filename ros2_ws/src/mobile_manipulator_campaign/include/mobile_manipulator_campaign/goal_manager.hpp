#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mobile_manipulator_routing/types.hpp"

namespace mobile_manipulator_campaign
{
struct GoalEnvironment
{
  std::uint64_t map = 0, footprint = 0, configuration = 0;
  bool operator==(const GoalEnvironment & other) const
  {
    return map == other.map && footprint == other.footprint && configuration == other.configuration;
  }
};

struct GoalSubmission
{
  std::uint64_t epoch = 1;
  std::string client;
  std::uint64_t client_request = 0;
  mobile_manipulator_routing::Pose2 target;
  double timeout_s = 120.0;
};

enum class GoalState { Planning, Tracking, Reached, Failed, TimedOut, Superseded, Canceled };
enum class GoalAction { Applied, Accepted, Scheduled, Duplicate, Rejected, Stale, Backpressure, NoChange };
enum class GoalEventKind
{
  Queued, Accepted, Rejected, Superseded, ReplanRequested, RouteAccepted,
  Reached, Failed, TimedOut, Canceled, Reset, EnvironmentChanged
};

struct GoalRecord
{
  GoalSubmission submission;
  mobile_manipulator_routing::Revisions search_token;
  std::uint64_t input_sequence = 0, route_id = 0;  // Last accepted route; terminal states retain it for history only.
  double accepted_at = 0.0, deadline = 0.0;
  std::optional<double> scheduled_for;
  GoalState state = GoalState::Planning;
};

struct GoalAcknowledgement
{
  GoalAction action = GoalAction::Rejected;
  std::optional<GoalAction> duplicate_of;  // Preserve the recorded outcome when action is Duplicate.
  std::uint64_t goal_id = 0, input_sequence = 0, route_id = 0;
  std::optional<mobile_manipulator_routing::Revisions> search_token;
  std::string reason;
};

struct GoalEvent
{
  std::uint64_t sequence = 0, input_sequence = 0;
  GoalEventKind kind = GoalEventKind::Rejected;
  double applied_at = 0.0;
  std::optional<double> scheduled_for;
  std::optional<GoalRecord> goal;
  std::string client;
  std::uint64_t client_request = 0, epoch = 0, requested_epoch = 0;
  std::string reason;
};

struct GoalManagerLimits
{
  std::size_t max_scheduled = 128;
  std::size_t max_events = 512;  // Must fit reset's queued cancellations + active cancellation + reset.
  std::size_t max_clients = 32;
  std::size_t receipts_per_client = 64;
  std::size_t max_client_bytes = 64;
  std::size_t max_reason_bytes = 256;
};

struct GoalAdvance
{
  bool complete = true;
  double applied_at = 0.0;
  std::size_t scheduled_applied = 0;
};

// No ROS, plant or wall-clock dependency. The caller supplies simulation boundaries.
// advance(now) applies an expired current goal first, then due submissions in requested-time /
// input-sequence order, all at now (never a fabricated earlier application timestamp).
// Same-epoch time cannot move backwards. Only reset creates a new epoch and may restart time.
//
// A full journal causes explicit backpressure, never silent event loss. Drain take_events(), then
// retry advance at the same boundary before making other mutations or advancing physical execution.
// Per-client request IDs increase monotonically within an epoch. Recent identical duplicates return
// their recorded acknowledgement; expired/out-of-order IDs reject instead of starting another goal.
// Rejected requests reserve neither IDs nor receipt entries. Queued requests reserve their ID on intake.
// Calls are serialized by the owning session; validators must not re-enter the manager.
class GoalManager
{
public:
  using Validator = std::function<std::optional<std::string>(
    const mobile_manipulator_routing::Pose2 &, const GoalEnvironment &)>;

  explicit GoalManager(GoalManagerLimits limits = {}, Validator validator = {}, GoalEnvironment environment = {});
  ~GoalManager();
  GoalManager(GoalManager &&) noexcept;
  GoalManager & operator=(GoalManager &&) noexcept;
  GoalManager(const GoalManager &) = delete;
  GoalManager & operator=(const GoalManager &) = delete;

  double time() const;
  std::uint64_t epoch() const;
  const GoalEnvironment & environment() const;
  const std::optional<GoalRecord> & current() const;
  bool active() const;
  std::optional<mobile_manipulator_routing::Revisions> pending_search() const;
  bool matches(const mobile_manipulator_routing::Revisions & token) const;
  std::size_t scheduled_count() const;
  std::size_t journal_size() const;
  bool backpressured() const;

  GoalAcknowledgement submit(GoalSubmission request);
  GoalAcknowledgement schedule(GoalSubmission request, double requested_at);
  GoalAdvance advance(double now);
  GoalAcknowledgement request_replan();
  GoalAcknowledgement update_environment(GoalEnvironment environment);
  GoalAcknowledgement accept_route(const mobile_manipulator_routing::Revisions & token);
  GoalAcknowledgement fail_search(const mobile_manipulator_routing::Revisions & token, std::string reason);
  GoalAcknowledgement note_reached(std::uint64_t goal_id);
  GoalAcknowledgement note_failed(std::uint64_t goal_id, std::string reason);
  GoalAcknowledgement reset(double now = 0.0, GoalEnvironment environment = {});

  std::vector<GoalEvent> take_events(std::size_t maximum = std::numeric_limits<std::size_t>::max());
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace mobile_manipulator_campaign
