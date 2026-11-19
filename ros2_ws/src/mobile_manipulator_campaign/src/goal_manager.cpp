#include "mobile_manipulator_campaign/goal_manager.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <stdexcept>
#include <utility>

namespace mobile_manipulator_campaign
{
namespace routing = mobile_manipulator_routing;
namespace
{
bool live(GoalState state) { return state == GoalState::Planning || state == GoalState::Tracking; }

std::uint64_t next(std::uint64_t & value)
{
  if (value == std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("goal identifier exhausted");
  return ++value;
}

bool same_payload(const GoalSubmission & a, const GoalSubmission & b)
{
  return a.epoch == b.epoch && a.client == b.client && a.client_request == b.client_request &&
    a.target.x == b.target.x && a.target.y == b.target.y && a.target.yaw == b.target.yaw &&
    a.timeout_s == b.timeout_s;
}
}  // namespace

struct GoalManager::Impl
{
  struct Receipt
  {
    GoalSubmission request;
    std::optional<double> scheduled_for;
    GoalAcknowledgement acknowledgement;
    bool queued = false;
  };
  struct Client
  {
    std::uint64_t watermark = 0;
    std::map<std::uint64_t,Receipt> receipts;
  };
  struct Scheduled
  {
    GoalSubmission request;
    double at;
    std::uint64_t input_sequence;
  };

  GoalManagerLimits limits;
  Validator validator;
  GoalEnvironment environment;
  double now = 0.0;
  std::uint64_t epoch = 1, next_goal = 0, next_request = 0, next_route = 0, next_input = 0, next_event = 0;
  std::optional<GoalRecord> current;
  std::map<std::string,Client> clients;
  std::vector<Scheduled> scheduled;
  std::deque<GoalEvent> events;
  bool pressure = false;

  bool active() const { return current && live(current->state); }
  bool room(std::size_t required) const { return required <= limits.max_events-events.size(); }

  GoalAcknowledgement answer(GoalAction action, std::string reason = {}) const
  {
    GoalAcknowledgement result;
    result.action = action;
    result.reason = std::move(reason);
    if (current) {
      result.goal_id = current->search_token.goal;
      result.input_sequence = current->input_sequence;
      result.route_id = current->route_id;
      result.search_token = current->search_token;
    }
    return result;
  }

  std::string bounded(std::string reason) const
  {
    // A resized moved string could retain an arbitrarily large caller-owned capacity.
    return std::string(reason.data(),std::min(reason.size(),limits.max_reason_bytes));
  }

  void append(GoalEventKind kind, const std::optional<GoalRecord> & goal, std::string reason = {},
              const GoalSubmission * request = nullptr, std::optional<double> scheduled_for = {},
              std::uint64_t input_sequence = 0)
  {
    if (!room(1)) throw std::logic_error("goal journal reservation violated");
    GoalEvent event;
    event.sequence = next(next_event);
    event.kind = kind;
    event.applied_at = now;
    event.epoch = epoch;
    event.goal = goal;
    event.reason = bounded(std::move(reason));
    event.scheduled_for = scheduled_for;
    event.input_sequence = input_sequence;
    if (goal) {
      event.epoch = goal->search_token.epoch;
      event.client = goal->submission.client;
      event.client_request = goal->submission.client_request;
      event.requested_epoch = goal->submission.epoch;
      event.input_sequence = goal->input_sequence;
      event.scheduled_for = goal->scheduled_for;
    } else if (request) {
      event.client = request->client.substr(0,limits.max_client_bytes);
      event.client_request = request->client_request;
      event.requested_epoch = request->epoch;
    }
    events.push_back(std::move(event));
  }

  std::optional<std::string> basic(const GoalSubmission & request, double accepted_at,
                                   bool check_deadline = true) const
  {
    if (request.epoch != epoch) return "request epoch is obsolete";
    if (request.client.empty() || request.client.size() > limits.max_client_bytes ||
        !std::all_of(request.client.begin(),request.client.end(),[](unsigned char c) { return c >= 33 && c <= 126; }))
      return "client ID must be bounded printable ASCII";
    if (!request.client_request) return "client request ID must be positive";
    if (!routing::finite(request.target)) return "goal pose must be finite";
    if (!std::isfinite(request.timeout_s) || request.timeout_s <= 0.0)
      return "goal timeout must be positive and finite";
    if (check_deadline) {
      const double deadline = accepted_at+request.timeout_s;
      if (!std::isfinite(deadline) || deadline <= accepted_at)
        return "goal deadline must be finite and representable";
    }
    return std::nullopt;
  }

  std::optional<std::string> validate_pose(const GoalSubmission & request) const
  {
    if (!validator) return std::nullopt;
    try {
      if (auto reason = validator(request.target,environment))
        return reason->empty() ? std::string("goal rejected by validator") : bounded(*reason);
    } catch (const std::exception &) {
      return "goal validator failed";
    }
    return std::nullopt;
  }

  GoalAcknowledgement reject(const GoalSubmission & request, std::string reason)
  {
    if (!room(1)) return answer(GoalAction::Backpressure,"drain the goal journal before retrying");
    append(GoalEventKind::Rejected,std::nullopt,reason,&request);
    GoalAcknowledgement result;
    result.action = GoalAction::Rejected;
    result.reason = bounded(std::move(reason));
    return result;
  }

  // Exact recent duplicates use their immutable receipt; an evicted/older ID never reactivates.
  std::optional<GoalAcknowledgement> duplicate(const GoalSubmission & request, std::optional<double> at)
  {
    const auto client = clients.find(request.client);
    if (client == clients.end()) return std::nullopt;
    const auto receipt = client->second.receipts.find(request.client_request);
    if (receipt != client->second.receipts.end()) {
      if (!same_payload(receipt->second.request,request) || receipt->second.scheduled_for != at)
        return reject(request,"client request ID already has a different payload");
      auto result = receipt->second.acknowledgement;
      result.duplicate_of = result.action;
      result.action = GoalAction::Duplicate;
      return result;
    }
    if (request.client_request <= client->second.watermark)
      return reject(request,"client request ID expired or arrived out of order");
    return std::nullopt;
  }

  bool receipt_room(const GoalSubmission & request) const
  {
    const auto client = clients.find(request.client);
    if (client == clients.end()) return clients.size() < limits.max_clients;
    if (client->second.receipts.size() < limits.receipts_per_client) return true;
    return std::any_of(client->second.receipts.begin(),client->second.receipts.end(),
      [](const auto & item) { return !item.second.queued; });
  }

  void remember(GoalSubmission request, std::optional<double> at, GoalAcknowledgement acknowledgement, bool queued)
  {
    auto & client = clients[request.client];
    if (client.receipts.size() >= limits.receipts_per_client) {
      const auto oldest = std::find_if(client.receipts.begin(),client.receipts.end(),
        [](const auto & item) { return !item.second.queued; });
      if (oldest == client.receipts.end()) throw std::logic_error("goal receipt reservation violated");
      client.receipts.erase(oldest);
    }
    client.watermark = request.client_request;
    const auto id = request.client_request;
    client.receipts.emplace(id,Receipt{std::move(request),at,std::move(acknowledgement),queued});
  }

  routing::Revisions token()
  {
    return {epoch,current->search_token.goal,next(next_request),environment.map,
      environment.footprint,environment.configuration};
  }

  GoalAcknowledgement accept(const GoalSubmission & request, std::uint64_t input_sequence,
                             std::optional<double> at)
  {
    if (active()) {
      current->state = GoalState::Superseded;
      append(GoalEventKind::Superseded,current,"replaced by an accepted destination");
    }
    GoalRecord record;
    record.submission = request;
    record.input_sequence = input_sequence;
    record.accepted_at = now;
    record.deadline = now+request.timeout_s;
    record.scheduled_for = at;
    record.search_token = {epoch,next(next_goal),next(next_request),environment.map,
      environment.footprint,environment.configuration};
    current = std::move(record);
    append(GoalEventKind::Accepted,current);
    return answer(GoalAction::Accepted);
  }

  GoalAcknowledgement finish(GoalState state, GoalEventKind kind, std::string reason)
  {
    if (pressure || !room(1)) return answer(GoalAction::Backpressure,"drain the goal journal before retrying");
    current->state = state;
    append(kind,current,std::move(reason));
    return answer(GoalAction::Applied);
  }
};

GoalManager::GoalManager(GoalManagerLimits limits, Validator validator, GoalEnvironment environment)
: impl_(std::make_unique<Impl>())
{
  if (!limits.max_events || limits.max_events > 16384 || limits.max_scheduled > 4096 ||
      limits.max_scheduled > limits.max_events ||
      limits.max_events-limits.max_scheduled < 2 || !limits.max_clients || !limits.receipts_per_client ||
      limits.max_clients > 256 || limits.receipts_per_client > 256 ||
      limits.max_clients > 16384/limits.receipts_per_client ||
      !limits.max_client_bytes || limits.max_client_bytes > 256 || !limits.max_reason_bytes || limits.max_reason_bytes > 1024)
    throw std::invalid_argument("goal manager needs bounded, nonzero journal/client/receipt capacities");
  impl_->limits = limits;
  impl_->validator = std::move(validator);
  impl_->environment = environment;
  impl_->scheduled.reserve(limits.max_scheduled);
}

GoalManager::~GoalManager() = default;
GoalManager::GoalManager(GoalManager &&) noexcept = default;
GoalManager & GoalManager::operator=(GoalManager &&) noexcept = default;
double GoalManager::time() const { return impl_->now; }
std::uint64_t GoalManager::epoch() const { return impl_->epoch; }
const GoalEnvironment & GoalManager::environment() const { return impl_->environment; }
const std::optional<GoalRecord> & GoalManager::current() const { return impl_->current; }
bool GoalManager::active() const { return impl_->active(); }
std::size_t GoalManager::scheduled_count() const { return impl_->scheduled.size(); }
std::size_t GoalManager::journal_size() const { return impl_->events.size(); }
bool GoalManager::backpressured() const { return impl_->pressure; }

std::optional<routing::Revisions> GoalManager::pending_search() const
{
  if (!impl_->pressure && impl_->current && impl_->current->state == GoalState::Planning &&
      impl_->now < impl_->current->deadline) return impl_->current->search_token;
  return std::nullopt;
}

bool GoalManager::matches(const routing::Revisions & token) const
{
  const auto pending = pending_search();
  return pending && *pending == token;
}

GoalAcknowledgement GoalManager::submit(GoalSubmission request)
{
  auto & state = *impl_;
  if (state.pressure) return state.answer(GoalAction::Backpressure,"complete the pending advance first");
  if (auto reason = state.basic(request,state.now,false)) return state.reject(request,*reason);
  std::string(request.client.data(),request.client.size()).swap(request.client);
  request.target.yaw = routing::wrap_angle(request.target.yaw);
  if (auto result = state.duplicate(request,std::nullopt)) return *result;
  if (auto reason = state.basic(request,state.now)) return state.reject(request,*reason);
  if (auto reason = state.validate_pose(request)) return state.reject(request,*reason);
  if (!state.receipt_room(request) || !state.room(state.active() ? 2 : 1))
    return state.answer(GoalAction::Backpressure,"goal receipt or journal capacity reached");
  auto result = state.accept(request,next(state.next_input),std::nullopt);
  state.remember(std::move(request),std::nullopt,result,false);
  return result;
}

GoalAcknowledgement GoalManager::schedule(GoalSubmission request, double requested_at)
{
  auto & state = *impl_;
  if (state.pressure) return state.answer(GoalAction::Backpressure,"complete the pending advance first");
  if (!std::isfinite(requested_at)) return state.reject(request,"scheduled time must be finite");
  if (auto reason = state.basic(request,requested_at,false)) return state.reject(request,*reason);
  std::string(request.client.data(),request.client.size()).swap(request.client);
  request.target.yaw = routing::wrap_angle(request.target.yaw);
  if (auto result = state.duplicate(request,requested_at)) return *result;
  if (requested_at < state.now) return state.reject(request,"scheduled time is before the current boundary");
  if (auto reason = state.basic(request,requested_at)) return state.reject(request,*reason);
  if (auto reason = state.validate_pose(request)) return state.reject(request,*reason);
  if (state.scheduled.size() >= state.limits.max_scheduled || !state.receipt_room(request) || !state.room(1))
    return state.answer(GoalAction::Backpressure,"scheduled goal, receipt or journal capacity reached");
  GoalAcknowledgement result;
  result.action = GoalAction::Scheduled;
  result.input_sequence = next(state.next_input);
  Impl::Scheduled item{request,requested_at,result.input_sequence};
  const auto at = std::upper_bound(state.scheduled.begin(),state.scheduled.end(),item,
    [](const auto & a,const auto & b) { return a.at < b.at || (a.at == b.at && a.input_sequence < b.input_sequence); });
  state.scheduled.insert(at,std::move(item));
  state.append(GoalEventKind::Queued,std::nullopt,{},&request,requested_at,result.input_sequence);
  state.remember(std::move(request),requested_at,result,true);
  return result;
}

GoalAdvance GoalManager::advance(double now)
{
  auto & state = *impl_;
  if (!std::isfinite(now) || now < state.now || now < 0.0)
    throw std::invalid_argument("simulation time must be finite and monotonic within an epoch");
  state.now = now;
  state.pressure = false;
  GoalAdvance result{true,now,0};
  if (state.active() && now >= state.current->deadline) {
    if (!state.room(1)) {
      state.pressure = true;
      result.complete = false;
      return result;
    }
    state.current->state = GoalState::TimedOut;
    state.append(GoalEventKind::TimedOut,state.current,"goal deadline elapsed");
  }
  while (!state.scheduled.empty() && state.scheduled.front().at <= now) {
    const auto & item = state.scheduled.front();
    auto reason = state.basic(item.request,now);
    if (!reason) reason = state.validate_pose(item.request);
    const std::size_t required = reason ? 1 : (state.active() ? 2 : 1);
    if (!state.room(required)) {
      state.pressure = true;
      result.complete = false;
      break;
    }
    GoalAcknowledgement acknowledgement;
    if (reason) {
      state.append(GoalEventKind::Rejected,std::nullopt,*reason,&item.request,item.at,item.input_sequence);
      acknowledgement.action = GoalAction::Rejected;
      acknowledgement.input_sequence = item.input_sequence;
      acknowledgement.reason = *reason;
    } else {
      acknowledgement = state.accept(item.request,item.input_sequence,item.at);
    }
    auto & receipt = state.clients.at(item.request.client).receipts.at(item.request.client_request);
    receipt.queued = false;
    receipt.acknowledgement = std::move(acknowledgement);
    state.scheduled.erase(state.scheduled.begin());
    ++result.scheduled_applied;
  }
  return result;
}

GoalAcknowledgement GoalManager::request_replan()
{
  auto & state = *impl_;
  if (!state.active()) return state.answer(GoalAction::Stale,"no active goal to replan");
  if (state.pressure || !state.room(1)) return state.answer(GoalAction::Backpressure,"drain the goal journal before retrying");
  state.current->state = GoalState::Planning;
  state.current->search_token = state.token();
  state.append(GoalEventKind::ReplanRequested,state.current,"same-destination route update");
  return state.answer(GoalAction::Applied);
}

GoalAcknowledgement GoalManager::update_environment(GoalEnvironment environment)
{
  auto & state = *impl_;
  if (state.environment == environment) return state.answer(GoalAction::NoChange);
  if (state.pressure || !state.room(state.active() ? 2 : 1))
    return state.answer(GoalAction::Backpressure,"drain the goal journal before retrying");
  state.environment = environment;
  state.append(GoalEventKind::EnvironmentChanged,std::nullopt,"routing input revisions changed");
  if (state.active()) {
    state.current->state = GoalState::Planning;
    state.current->search_token = state.token();
    state.append(GoalEventKind::ReplanRequested,state.current,"routing input revisions changed");
  }
  return state.answer(GoalAction::Applied);
}

GoalAcknowledgement GoalManager::accept_route(const routing::Revisions & token)
{
  auto & state = *impl_;
  if (state.pressure) return state.answer(GoalAction::Backpressure,"complete the pending advance first");
  if (!matches(token)) return state.answer(GoalAction::Stale,"search result is not current");
  if (!state.room(1)) return state.answer(GoalAction::Backpressure,"drain the goal journal before retrying");
  state.current->route_id = next(state.next_route);
  state.current->state = GoalState::Tracking;
  state.append(GoalEventKind::RouteAccepted,state.current);
  return state.answer(GoalAction::Applied);
}

GoalAcknowledgement GoalManager::fail_search(const routing::Revisions & token, std::string reason)
{
  if (impl_->pressure) return impl_->answer(GoalAction::Backpressure,"complete the pending advance first");
  if (!matches(token)) return impl_->answer(GoalAction::Stale,"search result is not current");
  return impl_->finish(GoalState::Failed,GoalEventKind::Failed,reason.empty() ? "search failed" : std::move(reason));
}

GoalAcknowledgement GoalManager::note_reached(std::uint64_t goal_id)
{
  if (!impl_->active() || impl_->current->search_token.goal != goal_id || !impl_->current->route_id)
    return impl_->answer(GoalAction::Stale,"goal is not currently executing");
  return impl_->finish(GoalState::Reached,GoalEventKind::Reached,"destination reached");
}

GoalAcknowledgement GoalManager::note_failed(std::uint64_t goal_id, std::string reason)
{
  if (!impl_->active() || impl_->current->search_token.goal != goal_id)
    return impl_->answer(GoalAction::Stale,"goal is not current");
  return impl_->finish(GoalState::Failed,GoalEventKind::Failed,reason.empty() ? "execution failed" : std::move(reason));
}

GoalAcknowledgement GoalManager::reset(double now, GoalEnvironment environment)
{
  auto & state = *impl_;
  if (!std::isfinite(now) || now < 0.0) throw std::invalid_argument("reset time must be finite and nonnegative");
  const std::size_t required = state.scheduled.size()+(state.active() ? 2 : 1);
  if (!state.room(required)) return state.answer(GoalAction::Backpressure,"drain the journal before reset");
  if (state.active()) {
    state.current->state = GoalState::Canceled;
    state.append(GoalEventKind::Canceled,state.current,"session reset");
  }
  for (const auto & item : state.scheduled)
    state.append(GoalEventKind::Canceled,std::nullopt,"session reset",&item.request,item.at,item.input_sequence);
  next(state.epoch);
  state.now = now;
  state.environment = environment;
  state.current.reset();
  state.scheduled.clear();
  state.clients.clear();
  state.pressure = false;
  state.append(GoalEventKind::Reset,std::nullopt,"new session epoch");
  return state.answer(GoalAction::Applied);
}

std::vector<GoalEvent> GoalManager::take_events(std::size_t maximum)
{
  std::vector<GoalEvent> result;
  const auto count = std::min(maximum,impl_->events.size());
  result.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    result.push_back(std::move(impl_->events.front()));
    impl_->events.pop_front();
  }
  return result;
}
}  // namespace mobile_manipulator_campaign
