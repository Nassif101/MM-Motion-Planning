#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "mobile_manipulator_campaign/goal_manager.hpp"

namespace campaign = mobile_manipulator_campaign;
namespace routing = mobile_manipulator_routing;
namespace
{
campaign::GoalSubmission request(std::uint64_t id, double x = 1.0, double timeout = 120.0,
                                 std::string client = "script", std::uint64_t epoch = 1)
{
  return {epoch,std::move(client),id,{x,2.0,0.3},timeout};
}

std::vector<campaign::GoalEventKind> kinds(const std::vector<campaign::GoalEvent> & events)
{
  std::vector<campaign::GoalEventKind> result;
  for (const auto & event : events) result.push_back(event.kind);
  return result;
}
}  // namespace

TEST(GoalManager, AcceptedGoalHasBackendIdentityAndDeadline)
{
  campaign::GoalManager manager({}, {}, {3,4,5});
  ASSERT_TRUE(manager.advance(2.0).complete);
  const auto acknowledgement = manager.submit(request(1,1.0,10.0));
  ASSERT_EQ(acknowledgement.action,campaign::GoalAction::Accepted);
  ASSERT_TRUE(acknowledgement.search_token);
  EXPECT_EQ(*acknowledgement.search_token,(routing::Revisions{1,1,1,3,4,5}));
  ASSERT_TRUE(manager.current());
  EXPECT_DOUBLE_EQ(manager.current()->accepted_at,2.0);
  EXPECT_DOUBLE_EQ(manager.current()->deadline,12.0);
  EXPECT_EQ(manager.current()->state,campaign::GoalState::Planning);
  EXPECT_EQ(manager.current()->route_id,0u);
  EXPECT_TRUE(manager.matches(*acknowledgement.search_token));
  EXPECT_TRUE(manager.active());
}

TEST(GoalManager, InvalidSubmissionDoesNotPreemptOrReserveItsClientRequest)
{
  campaign::GoalManager manager({}, [](const routing::Pose2 & pose,const campaign::GoalEnvironment &)
    -> std::optional<std::string> { return pose.x < 0 ? std::optional<std::string>("outside map") : std::nullopt; });
  const auto a = manager.submit(request(1));
  ASSERT_TRUE(a.search_token);
  auto invalid = request(2);
  invalid.target.yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(manager.submit(invalid).action,campaign::GoalAction::Rejected);
  EXPECT_EQ(manager.submit(request(2,-1)).action,campaign::GoalAction::Rejected);
  EXPECT_EQ(manager.pending_search(),a.search_token);
  const auto corrected = manager.submit(request(2,3));
  EXPECT_EQ(corrected.action,campaign::GoalAction::Accepted);
  EXPECT_EQ(corrected.goal_id,a.goal_id+1);
  EXPECT_EQ(corrected.search_token->request,a.search_token->request+1);
}

TEST(GoalManager, RapidReplacementsArchiveOldGoalsAndRejectTheirResults)
{
  campaign::GoalManager manager;
  const auto a = manager.submit(request(1,1));
  const auto b = manager.submit(request(2,2));
  const auto c = manager.submit(request(3,3));
  ASSERT_TRUE(a.search_token && b.search_token && c.search_token);
  EXPECT_EQ(manager.accept_route(*a.search_token).action,campaign::GoalAction::Stale);
  EXPECT_EQ(manager.fail_search(*b.search_token,"late no_path").action,campaign::GoalAction::Stale);
  const auto route = manager.accept_route(*c.search_token);
  EXPECT_EQ(route.action,campaign::GoalAction::Applied);
  EXPECT_EQ(route.route_id,1u);
  EXPECT_EQ(manager.current()->state,campaign::GoalState::Tracking);
  EXPECT_FALSE(manager.pending_search());
  const auto events = manager.take_events();
  ASSERT_EQ(events.size(),6u);
  EXPECT_EQ(kinds(events),(std::vector<campaign::GoalEventKind>{campaign::GoalEventKind::Accepted,
    campaign::GoalEventKind::Superseded,campaign::GoalEventKind::Accepted,
    campaign::GoalEventKind::Superseded,campaign::GoalEventKind::Accepted,campaign::GoalEventKind::RouteAccepted}));
  ASSERT_TRUE(events[1].goal);
  EXPECT_EQ(events[1].goal->state,campaign::GoalState::Superseded);
  EXPECT_EQ(events[1].goal->search_token.goal,a.goal_id);
  EXPECT_EQ(events[3].goal->search_token.goal,b.goal_id);
  for (std::size_t i = 1; i < events.size(); ++i) EXPECT_LT(events[i-1].sequence,events[i].sequence);
}

TEST(GoalManager, SameGoalReplanPreservesDeadlineAndDistinguishesNewDestination)
{
  campaign::GoalManager manager;
  const auto a = manager.submit(request(1,1,10));
  ASSERT_EQ(manager.accept_route(*a.search_token).action,campaign::GoalAction::Applied);
  manager.advance(2.0);
  const auto replanned = manager.request_replan();
  ASSERT_TRUE(replanned.search_token);
  EXPECT_EQ(replanned.goal_id,a.goal_id);
  EXPECT_GT(replanned.search_token->request,a.search_token->request);
  EXPECT_EQ(manager.current()->route_id,1u);
  EXPECT_DOUBLE_EQ(manager.current()->accepted_at,0.0);
  EXPECT_DOUBLE_EQ(manager.current()->deadline,10.0);
  EXPECT_EQ(manager.accept_route(*a.search_token).action,campaign::GoalAction::Stale);
  EXPECT_EQ(manager.accept_route(*replanned.search_token).route_id,2u);
  const auto b = manager.submit(request(2,1,10));  // Same coordinates but a new accepted destination request.
  EXPECT_NE(b.goal_id,a.goal_id);
  EXPECT_EQ(manager.current()->route_id,0u);
  EXPECT_DOUBLE_EQ(manager.current()->deadline,12.0);
}

TEST(GoalManager, EveryEnvironmentRevisionParticipatesInSearchIdentity)
{
  campaign::GoalManager manager({}, {}, {1,2,3});
  const auto a = manager.submit(request(1));
  ASSERT_TRUE(a.search_token);
  for (int field = 0; field < 6; ++field) {
    auto altered = *a.search_token;
    auto * value = field == 0 ? &altered.epoch : field == 1 ? &altered.goal : field == 2 ? &altered.request :
      field == 3 ? &altered.map : field == 4 ? &altered.footprint : &altered.configuration;
    ++*value;
    EXPECT_FALSE(manager.matches(altered));
  }
  EXPECT_EQ(manager.update_environment({1,2,3}).action,campaign::GoalAction::NoChange);
  const auto changed = manager.update_environment({9,8,7});
  ASSERT_TRUE(changed.search_token);
  EXPECT_EQ(changed.search_token->map,9u);
  EXPECT_EQ(changed.search_token->footprint,8u);
  EXPECT_EQ(changed.search_token->configuration,7u);
  EXPECT_FALSE(manager.matches(*a.search_token));
  EXPECT_EQ(manager.fail_search(*a.search_token,"stale").action,campaign::GoalAction::Stale);
  EXPECT_EQ(manager.accept_route(*changed.search_token).action,campaign::GoalAction::Applied);
}

TEST(GoalManager, EqualTimeScheduledGoalsUseInputOrderAtActualBoundary)
{
  campaign::GoalManager manager;
  const auto a = manager.schedule(request(1,1,2),5.0);
  const auto b = manager.schedule(request(2,2,2),5.0);
  EXPECT_EQ(a.action,campaign::GoalAction::Scheduled);
  EXPECT_EQ(b.action,campaign::GoalAction::Scheduled);
  EXPECT_LT(a.input_sequence,b.input_sequence);
  EXPECT_FALSE(manager.current());
  EXPECT_EQ(manager.advance(4.99).scheduled_applied,0u);
  const auto result = manager.advance(5.2);
  EXPECT_TRUE(result.complete);
  EXPECT_EQ(result.scheduled_applied,2u);
  ASSERT_TRUE(manager.current());
  EXPECT_DOUBLE_EQ(manager.current()->submission.target.x,2.0);
  EXPECT_DOUBLE_EQ(manager.current()->accepted_at,5.2);
  EXPECT_DOUBLE_EQ(manager.current()->deadline,7.2);
  EXPECT_EQ(manager.current()->scheduled_for,std::optional<double>(5.0));
  EXPECT_EQ(manager.scheduled_count(),0u);
  const auto events = manager.take_events();
  ASSERT_EQ(events.size(),5u);
  EXPECT_EQ(events[2].kind,campaign::GoalEventKind::Accepted);
  EXPECT_EQ(events[3].kind,campaign::GoalEventKind::Superseded);
  EXPECT_EQ(events[4].kind,campaign::GoalEventKind::Accepted);
  for (std::size_t i = 2; i < events.size(); ++i) EXPECT_DOUBLE_EQ(events[i].applied_at,5.2);
}

TEST(GoalManager, DeadlineIsAppliedBeforeDueInputsAtTheSameBoundary)
{
  campaign::GoalManager manager;
  const auto a = manager.submit(request(1,1,3));
  manager.schedule(request(2,2,2),3.0);
  manager.take_events();
  const auto result = manager.advance(10.0);
  EXPECT_TRUE(result.complete);
  EXPECT_EQ(result.scheduled_applied,1u);
  const auto events = manager.take_events();
  ASSERT_EQ(events.size(),2u);
  EXPECT_EQ(events[0].kind,campaign::GoalEventKind::TimedOut);
  EXPECT_EQ(events[0].goal->search_token.goal,a.goal_id);
  EXPECT_EQ(events[1].kind,campaign::GoalEventKind::Accepted);
  EXPECT_DOUBLE_EQ(events[0].applied_at,10.0);
  EXPECT_DOUBLE_EQ(events[1].applied_at,10.0);
  EXPECT_DOUBLE_EQ(manager.current()->accepted_at,10.0);
  EXPECT_DOUBLE_EQ(manager.current()->deadline,12.0);
  EXPECT_EQ(manager.current()->scheduled_for,std::optional<double>(3.0));
}

TEST(GoalManager, ScheduleOrderUsesRequestedTimeBeforeInputSequence)
{
  campaign::GoalManager manager;
  manager.schedule(request(1,10),10.0);
  manager.schedule(request(2,5),5.0);
  manager.take_events();
  EXPECT_EQ(manager.advance(11).scheduled_applied,2u);
  const auto events = manager.take_events();
  ASSERT_EQ(events.size(),3u);
  EXPECT_DOUBLE_EQ(events.front().goal->submission.target.x,5.0);
  EXPECT_DOUBLE_EQ(events.back().goal->submission.target.x,10.0);
}

TEST(GoalManager, ScheduledDuplicateAfterApplicationReturnsRecordedReceipt)
{
  campaign::GoalManager manager;
  const auto original = request(1);
  manager.schedule(original,2.0);
  manager.advance(3.0);
  const auto goal = manager.current()->search_token.goal;
  manager.take_events();
  const auto duplicate = manager.schedule(original,2.0);
  EXPECT_EQ(duplicate.action,campaign::GoalAction::Duplicate);
  EXPECT_EQ(duplicate.duplicate_of,std::optional<campaign::GoalAction>(campaign::GoalAction::Accepted));
  EXPECT_EQ(duplicate.goal_id,goal);
  EXPECT_EQ(manager.journal_size(),0u);
  EXPECT_EQ(manager.scheduled_count(),0u);
  EXPECT_EQ(manager.schedule(request(2),2.0).action,campaign::GoalAction::Rejected);
}

TEST(GoalManager, RecentDuplicateIsIdempotentAndConflictingPayloadRejects)
{
  campaign::GoalManager manager;
  const auto original = request(1);
  const auto accepted = manager.submit(original);
  manager.take_events();
  manager.advance(1.0);
  EXPECT_EQ(manager.submit(original).action,campaign::GoalAction::Duplicate);
  EXPECT_EQ(manager.journal_size(),0u);
  EXPECT_EQ(manager.submit(request(1,99)).action,campaign::GoalAction::Rejected);
  EXPECT_EQ(manager.current()->search_token.goal,accepted.goal_id);
  EXPECT_DOUBLE_EQ(manager.current()->submission.target.x,1.0);
  EXPECT_EQ(manager.schedule(original,2.0).action,campaign::GoalAction::Rejected);
}

TEST(GoalManager, ExpiredReceiptNeverReactivatesAnOldRequest)
{
  campaign::GoalManagerLimits limits;
  limits.receipts_per_client = 2;
  campaign::GoalManager manager(limits);
  manager.submit(request(1,1));
  manager.submit(request(2,2));
  const auto current = manager.submit(request(3,3));
  EXPECT_EQ(manager.submit(request(1,1)).action,campaign::GoalAction::Rejected);
  EXPECT_EQ(manager.submit(request(3,3)).action,campaign::GoalAction::Duplicate);
  EXPECT_EQ(manager.current()->search_token.goal,current.goal_id);
  EXPECT_DOUBLE_EQ(manager.current()->submission.target.x,3.0);
}

TEST(GoalManager, FailedValidationDoesNotPoisonFutureClientIds)
{
  campaign::GoalManager manager;
  auto invalid = request(100);
  invalid.timeout_s = -1;
  EXPECT_EQ(manager.submit(invalid).action,campaign::GoalAction::Rejected);
  EXPECT_EQ(manager.submit(request(1)).action,campaign::GoalAction::Accepted);
  EXPECT_EQ(manager.submit(request(100)).action,campaign::GoalAction::Accepted);
}

TEST(GoalManager, BoundsClientsScheduledQueueAndPendingReceipts)
{
  campaign::GoalManagerLimits limits;
  limits.max_clients = 1;
  limits.max_scheduled = 2;
  limits.receipts_per_client = 1;
  campaign::GoalManager manager(limits);
  EXPECT_EQ(manager.schedule(request(1),2).action,campaign::GoalAction::Scheduled);
  EXPECT_EQ(manager.schedule(request(2),3).action,campaign::GoalAction::Backpressure);
  EXPECT_EQ(manager.submit(request(1,1,120,"another")).action,campaign::GoalAction::Backpressure);
  EXPECT_EQ(manager.scheduled_count(),1u);
  manager.advance(2);
  EXPECT_EQ(manager.schedule(request(2),3).action,campaign::GoalAction::Scheduled);

  limits.max_scheduled = 1;
  limits.receipts_per_client = 64;
  campaign::GoalManager queue(limits);
  EXPECT_EQ(queue.schedule(request(1),2).action,campaign::GoalAction::Scheduled);
  EXPECT_EQ(queue.schedule(request(2),3).action,campaign::GoalAction::Backpressure);
  queue.advance(2);
  EXPECT_EQ(queue.schedule(request(2),3).action,campaign::GoalAction::Scheduled);
}

TEST(GoalManager, JournalBackpressurePreservesStateAndCanResumeAtSameBoundary)
{
  campaign::GoalManagerLimits limits;
  limits.max_scheduled = 0;
  limits.max_events = 2;
  campaign::GoalManager manager(limits);
  const auto a = manager.submit(request(1,1,2));
  ASSERT_EQ(manager.accept_route(*a.search_token).action,campaign::GoalAction::Applied);
  EXPECT_EQ(manager.journal_size(),2u);
  EXPECT_EQ(manager.submit(request(2)).action,campaign::GoalAction::Backpressure);
  EXPECT_EQ(manager.current()->search_token.goal,a.goal_id);
  EXPECT_EQ(manager.take_events(1).size(),1u);
  EXPECT_EQ(manager.submit(request(2)).action,campaign::GoalAction::Backpressure);  // replacement needs two events
  manager.take_events();
  const auto b = manager.submit(request(2,2,2));
  ASSERT_EQ(b.action,campaign::GoalAction::Accepted);
  EXPECT_EQ(manager.journal_size(),2u);
  EXPECT_FALSE(manager.advance(3.0).complete);
  EXPECT_TRUE(manager.backpressured());
  EXPECT_FALSE(manager.pending_search());
  EXPECT_EQ(manager.accept_route(*b.search_token).action,campaign::GoalAction::Backpressure);
  const auto prior = manager.take_events();
  EXPECT_EQ(kinds(prior),(std::vector<campaign::GoalEventKind>{campaign::GoalEventKind::Superseded,
    campaign::GoalEventKind::Accepted}));
  EXPECT_TRUE(manager.advance(3.0).complete);
  EXPECT_FALSE(manager.backpressured());
  EXPECT_EQ(manager.current()->state,campaign::GoalState::TimedOut);
  EXPECT_EQ(manager.take_events().front().kind,campaign::GoalEventKind::TimedOut);
}

TEST(GoalManager, PartialScheduledAdvanceNeverDropsOrReordersDueInputs)
{
  campaign::GoalManagerLimits limits;
  limits.max_scheduled = 2;
  limits.max_events = 4;
  campaign::GoalManager manager(limits);
  manager.schedule(request(1,1),2);
  manager.schedule(request(2,2),2);
  // Two Queued events leave room for one acceptance but not its subsequent replacement pair.
  EXPECT_FALSE(manager.advance(2).complete);
  EXPECT_EQ(manager.scheduled_count(),1u);
  EXPECT_DOUBLE_EQ(manager.current()->submission.target.x,1.0);
  const auto first = manager.take_events();
  ASSERT_EQ(first.size(),3u);
  EXPECT_TRUE(manager.advance(2).complete);
  const auto second = manager.take_events();
  ASSERT_EQ(second.size(),2u);
  EXPECT_EQ(second[0].kind,campaign::GoalEventKind::Superseded);
  EXPECT_EQ(second[1].kind,campaign::GoalEventKind::Accepted);
  EXPECT_GT(second.front().sequence,first.back().sequence);
  EXPECT_DOUBLE_EQ(manager.current()->submission.target.x,2.0);
}

TEST(GoalManager, ResetPreservesUndrainedHistoryAndRejectsOldEpochInputs)
{
  campaign::GoalManager manager;
  manager.advance(10);
  const auto a = manager.submit(request(1));
  manager.schedule(request(2),20);
  EXPECT_EQ(manager.reset(0,{7,8,9}).action,campaign::GoalAction::Applied);
  EXPECT_EQ(manager.epoch(),2u);
  EXPECT_DOUBLE_EQ(manager.time(),0.0);
  EXPECT_FALSE(manager.current());
  EXPECT_EQ(manager.scheduled_count(),0u);
  EXPECT_EQ(manager.accept_route(*a.search_token).action,campaign::GoalAction::Stale);
  const auto events = manager.take_events();
  EXPECT_EQ(kinds(events),(std::vector<campaign::GoalEventKind>{campaign::GoalEventKind::Accepted,
    campaign::GoalEventKind::Queued,campaign::GoalEventKind::Canceled,campaign::GoalEventKind::Canceled,
    campaign::GoalEventKind::Reset}));
  EXPECT_EQ(events.front().epoch,1u);
  EXPECT_EQ(events.back().epoch,2u);
  EXPECT_DOUBLE_EQ(events[2].applied_at,10.0);
  EXPECT_DOUBLE_EQ(events.back().applied_at,0.0);
  EXPECT_EQ(manager.submit(request(1)).action,campaign::GoalAction::Rejected);
  EXPECT_EQ(manager.submit(request(1,1,120,"script",2)).action,campaign::GoalAction::Accepted);
  EXPECT_EQ(manager.pending_search()->map,7u);
}

TEST(GoalManager, ResetBackpressureDoesNotPretendToInvalidateTheOldEpoch)
{
  campaign::GoalManagerLimits limits;
  limits.max_scheduled = 0;
  limits.max_events = 2;
  campaign::GoalManager manager(limits);
  const auto a = manager.submit(request(1));
  EXPECT_EQ(manager.reset().action,campaign::GoalAction::Backpressure);
  EXPECT_EQ(manager.epoch(),1u);
  EXPECT_TRUE(manager.matches(*a.search_token));
  manager.take_events();
  EXPECT_EQ(manager.reset().action,campaign::GoalAction::Applied);
  EXPECT_EQ(manager.epoch(),2u);
  EXPECT_FALSE(manager.matches(*a.search_token));
}

TEST(GoalManager, NewGoalAfterReachedFailedOrTimeoutGetsItsOwnInterval)
{
  campaign::GoalManager manager;
  const auto a = manager.submit(request(1,1,1));
  manager.accept_route(*a.search_token);
  manager.advance(0.5);
  EXPECT_EQ(manager.note_reached(a.goal_id).action,campaign::GoalAction::Applied);
  manager.advance(20);
  const auto b = manager.submit(request(2,2,10));
  EXPECT_DOUBLE_EQ(manager.current()->deadline,30.0);
  manager.advance(25);
  EXPECT_EQ(manager.current()->state,campaign::GoalState::Planning);
  EXPECT_EQ(manager.fail_search(*b.search_token,"no_path").action,campaign::GoalAction::Applied);
  EXPECT_FALSE(manager.active());
  EXPECT_EQ(manager.request_replan().action,campaign::GoalAction::Stale);
  const auto c = manager.submit(request(3,3,2));
  EXPECT_EQ(c.action,campaign::GoalAction::Accepted);
  manager.advance(27);
  EXPECT_EQ(manager.current()->state,campaign::GoalState::TimedOut);
  const auto d = manager.submit(request(4,4,5));
  EXPECT_EQ(d.action,campaign::GoalAction::Accepted);
  EXPECT_DOUBLE_EQ(manager.current()->accepted_at,27.0);
  EXPECT_DOUBLE_EQ(manager.current()->deadline,32.0);
}

TEST(GoalManager, QueuedGoalIsValidatedAgainAtApplicationWithoutPreemptingOnRejection)
{
  bool blocked = false;
  campaign::GoalManager manager({}, [&](const routing::Pose2 &,const campaign::GoalEnvironment &)
    -> std::optional<std::string> { return blocked ? std::optional<std::string>("now blocked") : std::nullopt; });
  const auto a = manager.submit(request(1));
  manager.schedule(request(2),2);
  blocked = true;
  manager.advance(2);
  EXPECT_EQ(manager.current()->search_token.goal,a.goal_id);
  const auto duplicate = manager.schedule(request(2),2);
  EXPECT_EQ(duplicate.action,campaign::GoalAction::Duplicate);
  EXPECT_EQ(duplicate.duplicate_of,std::optional<campaign::GoalAction>(campaign::GoalAction::Rejected));
  EXPECT_EQ(duplicate.goal_id,0u);
  EXPECT_EQ(duplicate.reason,"now blocked");
}

TEST(GoalManager, SameEpochClockRejectsBackwardsOrNonfiniteTime)
{
  campaign::GoalManager manager;
  manager.advance(2);
  EXPECT_THROW(manager.advance(1.999),std::invalid_argument);
  EXPECT_THROW(manager.advance(std::numeric_limits<double>::infinity()),std::invalid_argument);
  EXPECT_THROW(manager.advance(std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
  EXPECT_DOUBLE_EQ(manager.time(),2.0);
  EXPECT_THROW(manager.reset(-1),std::invalid_argument);
  EXPECT_THROW(manager.reset(std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
  EXPECT_EQ(manager.reset(0).action,campaign::GoalAction::Applied);
  EXPECT_NO_THROW(manager.advance(0));
}

TEST(GoalManager, DiagnosticsAndCapacityConfigurationAreBounded)
{
  campaign::GoalManagerLimits limits;
  limits.max_reason_bytes = 8;
  campaign::GoalManager manager(limits);
  const auto a = manager.submit(request(1));
  manager.fail_search(*a.search_token,std::string(10000,'x'));
  EXPECT_EQ(manager.take_events().back().reason.size(),8u);
  limits.max_clients = 0;
  EXPECT_THROW(campaign::GoalManager{limits},std::invalid_argument);
  limits.max_clients = 1;
  limits.max_events = 2;  // insufficient room to reset the configured scheduled queue
  EXPECT_THROW(campaign::GoalManager{limits},std::invalid_argument);
  limits.max_events = 1000000000;
  EXPECT_THROW(campaign::GoalManager{limits},std::invalid_argument);
}

TEST(GoalManager, DeadlineRepresentationDoesNotBreakOldDuplicateReceipts)
{
  campaign::GoalManager manager;
  const auto original = request(1);
  manager.submit(original);
  manager.advance(1e308);
  EXPECT_EQ(manager.submit(original).action,campaign::GoalAction::Duplicate);
  EXPECT_EQ(manager.submit(request(2)).action,campaign::GoalAction::Rejected);
}

// Spec 4A Task 1: a client cancels the active goal and any scheduled submissions without a new epoch
TEST(GoalManager, CancelEndsTheActiveGoal)
{
  campaign::GoalManager manager;
  const auto submitted = manager.submit(request(1));
  ASSERT_EQ(submitted.action,campaign::GoalAction::Accepted);
  const auto epoch = manager.epoch();
  manager.take_events();
  const auto ack = manager.cancel("client");
  EXPECT_EQ(ack.action,campaign::GoalAction::Applied);
  EXPECT_EQ(ack.goal_id,submitted.goal_id);
  EXPECT_FALSE(manager.active());
  ASSERT_TRUE(manager.current());
  EXPECT_EQ(manager.current()->state,campaign::GoalState::Canceled);
  EXPECT_EQ(manager.epoch(),epoch);
  const auto events = manager.take_events();
  ASSERT_EQ(kinds(events),std::vector<campaign::GoalEventKind>{campaign::GoalEventKind::Canceled});
  EXPECT_EQ(events[0].reason,"client");
  ASSERT_TRUE(events[0].goal);
  EXPECT_EQ(events[0].goal->search_token.goal,submitted.goal_id);
  EXPECT_FALSE(manager.pending_search());
}

TEST(GoalManager, CancelRemovesScheduledSubmissions)
{
  campaign::GoalManager manager;
  ASSERT_EQ(manager.schedule(request(1),5.0).action,campaign::GoalAction::Scheduled);
  manager.take_events();
  EXPECT_EQ(manager.cancel("client").action,campaign::GoalAction::Applied);
  EXPECT_EQ(manager.scheduled_count(),0u);
  const auto events = manager.take_events();
  ASSERT_EQ(kinds(events),std::vector<campaign::GoalEventKind>{campaign::GoalEventKind::Canceled});
  EXPECT_FALSE(events[0].goal);
  ASSERT_TRUE(events[0].request);
  EXPECT_DOUBLE_EQ(events[0].request->target.x,1.0);
  ASSERT_TRUE(manager.advance(6.0).complete);
  EXPECT_FALSE(manager.current());
  EXPECT_FALSE(manager.active());
}

TEST(GoalManager, CancelWithoutGoalIsRejected)
{
  campaign::GoalManager manager;
  const auto ack = manager.cancel("client");
  EXPECT_EQ(ack.action,campaign::GoalAction::Rejected);
  EXPECT_EQ(ack.reason,"no_goal");
  EXPECT_EQ(manager.journal_size(),0u);
  ASSERT_EQ(manager.submit(request(1)).action,campaign::GoalAction::Accepted);
  ASSERT_EQ(manager.note_failed(manager.current()->search_token.goal,"test").action,campaign::GoalAction::Applied);
  manager.take_events();
  EXPECT_EQ(manager.cancel("client").action,campaign::GoalAction::Rejected);  // a finished goal is not canceled
  EXPECT_EQ(manager.current()->state,campaign::GoalState::Failed);
}

TEST(GoalManager, CancelUnderBackpressureAnswersBackpressure)
{
  campaign::GoalManagerLimits limits;
  limits.max_events = 2;
  limits.max_scheduled = 0;
  campaign::GoalManager manager(limits);
  ASSERT_EQ(manager.submit(request(1)).action,campaign::GoalAction::Accepted);
  ASSERT_EQ(manager.request_replan().action,campaign::GoalAction::Applied);
  ASSERT_EQ(manager.journal_size(),2u);  // full
  EXPECT_EQ(manager.cancel("client").action,campaign::GoalAction::Backpressure);
  EXPECT_TRUE(manager.active());
  manager.take_events();
  EXPECT_EQ(manager.cancel("client").action,campaign::GoalAction::Applied);
}

// Task 1 review I1: canceled scheduled submissions release their client receipt slots; their IDs stay used
TEST(GoalManager, CancelReleasesScheduledReceipts)
{
  campaign::GoalManagerLimits limits;
  limits.receipts_per_client = 2;
  campaign::GoalManager manager(limits);
  std::uint64_t id = 0;
  for (int cycle = 0; cycle < 4; ++cycle) {  // repeated schedule/cancel never exhausts the client's receipts
    ASSERT_EQ(manager.schedule(request(++id),50.0).action,campaign::GoalAction::Scheduled) << cycle;
    ASSERT_EQ(manager.schedule(request(++id),60.0).action,campaign::GoalAction::Scheduled) << cycle;
    ASSERT_EQ(manager.cancel("client").action,campaign::GoalAction::Applied) << cycle;
    manager.take_events();
  }
  EXPECT_EQ(manager.submit(request(++id)).action,campaign::GoalAction::Accepted);
  EXPECT_NE(manager.submit(request(id-1)).action,campaign::GoalAction::Accepted);  // a canceled ID stays spent
  ASSERT_TRUE(manager.advance(61.0).complete);
  EXPECT_EQ(manager.scheduled_count(),0u);
}
