# Spec 2 to spec 4: backend contracts for the viewer design

Date: 2026-11-20. Input to [spec 4 plan](../../superpowers/plans/2026-11-18-plant-unity-viewer.md) Task 0 (the viewer
technical design). Spec 2 Task 14 hands over what exists and has been tested on branch `plant-routing-unity-viewer`.
The viewer consumes these contracts. It must not create a second backend state machine: goals, routes, their states
and outcome reasons belong to the session.

## What the backend does

- **One session per run** (`mobile_manipulator_campaign::Session`): MuJoCo plant, B5, F2 and, in generated-route
  mode, the route planner and goal manager. It runs headless at simulation time; pacing is the server's.
- **Two route modes, never pooled.**
  - *Recorded:* a bag's Nav2 routes, replayed.
  - *Generated (`--routes lattice`):* the session plans each destination with the Smac Lattice port.
- **Goals in generated mode** go through the goal manager:
  - submitted, scheduled at a simulation time, replaced by a newer goal, or replanned;
  - each has an id, an epoch, and the submitting client's own request number;
  - every change is a typed event with a reason.
- **What the planner knows.** Generated routes are planned on the static map plus explicitly declared known
  obstacles. Lidar obstacles reach B5, not the route planner, until the perception phase (spec 2 Amendment 2).

## Contracts

| Contract | Where | Tested by |
|---|---|---|
| Stream and run-log protocol, schema 3: frames, Hello, Control commands, events | [stream-protocol.md](../mujoco-plant/stream-protocol.md) | `test_demo_server`, `tools/plant/test_plant_stream.py` |
| Goal frames (type 20): goal records with `request`, `client`, `client_request`, `state`, `revisions` | `goal_records.{hpp,cpp}` | fixtures `goal_v3.bin`, `goal_rejected_v3.bin` (C++ and Python) |
| Route frames (type 21): adopted path (x, y, yaw), route points, planner, iterations, `smoothed`, `turn_first` | `goal_records.{hpp,cpp}`, `plant_stream.route_path()` | fixture `route_v3.bin` |
| Replay to a late or reconnecting client: the latest Hello, Goal and Route frames after its hello | `Session::resync_frames()`, `demo_server.cpp` | `test_demo_server` |
| F2 plans (type 11): each medium plan's predicted samples (x, y, yaw and the six arm joints) with `plan_id`, `route_id`, `status`, `homotopy`, `capped` | `session.cpp`, stream-protocol.md "Plans" | `view_run.py` draws them |
| Backpressure: State, Plans and Scan frames may be dropped for a slow client; Hello, Event, Goal and Route never are; a client whose kept backlog passes 16x the cap is disconnected | `stream_server` | `test_demo_server` over-cap tests |
| Goal lifecycle and acknowledgements (`accepted`, `rejected`, `duplicate`, `stale`, `backpressure`, `scheduled`, ...) | `goal_manager.hpp`, stream-protocol.md `submit_goal` | `test_goal_manager`, `test_generated_session` |
| Lidar observations at acquisition time (no labels) | `mobile_manipulator_routing/observation.hpp`, `Session::take_observations()` | `test_observation`, `test_observations` |
| Routing fingerprint (what a lattice run's routes depend on) | summary `routing`; `plant_campaign --print-routing-fingerprint` | `test_routing_inputs`, `tools/plant/test_plant_campaign.py` |

The viewer reads Hello, State, Plans, Scan, Event, Goal and Route frames. It sends Control commands: `hello`, `reset`,
`run`, `pause`, `step`, `submit_goal`, `schedule_goal`, `request_replan`, `cancel_goal` (spec 4A Task 1), `move_body`. A destination clicked on the minimap is
a `submit_goal` with the viewer's own increasing `client_request`.

## Running it

```bash
# container: a generated-route demo server (port 7700, published on this host only)
ros2_ws/install/mobile_manipulator_campaign/lib/mobile_manipulator_campaign/plant_demo_server --mode realistic --routes lattice [--lidar] [--log FILE]
```

A minimal client is `tools/plant/plant_stream.Client` (Python, host or container):

```python
import plant_stream as ps
c = ps.Client("localhost", 7700)                      # reads the server Hello, sends hello
c.command("reset", scenario="v2_open_yard_nav", seed=1)
c.command("submit_goal", target=[10.0, 2.0, 0.0], client_request=1)
c.command("run", speed=1.0)
kind, header, blocks = c.next()                       # Hello, State, Plans, Scan, Event, Goal, Route frames
```

`tools/plant/view_run.py --live` drives recorded-route servers only: it resets with a bag and moves goals with
`set_goal`. It does not drive a generated-route server. Its replay of run logs (`view_run.py RUN.mmlog`) draws
generated routes from the Route frames.

## Measured costs for the viewer workload

Spec 2 Task 14 measured these on the MacBook Air M4 (fanless), Unity closed. See
[implementation.md](implementation.md), "Task 14".

| Quantity | Value |
|---|---|
| Route planner setup at reset (inputs, costmap) | about 150 ms |
| Search wall time per tick at 4,096 iterations | p50 5-6 ms, max 20.8 ms; at most 50 ticks (1 s) per search on V2 |
| Headless real-time factor, one run, realistic plant | 3.9-6.3 lidar off, 2.7-3.8 lidar on |
| Paced 1x demo server, lidar on, longest V2 search | 0.997x achieved over 60 s, no `behind` event |
| Frame rates at 1x | State 50 Hz, Scan 10 Hz |

## Gaps the viewer design must decide

1. **Cancel (resolved).** Spec 4A Task 1 added `cancel_goal` (`f837960`, `a1de482`): it ends the active goal and
   every scheduled submission of the epoch; the robot holds.
2. **Display data.** F2's plans are already streamed (Plans frames: predicted base and arm samples). B5's own
   prediction, its margins and diagnostics, and the costmap are not streamed; spec 4 adds what the viewer needs
   (corrected 2026-11-20, Task 14 review M2: the first version said no F2 predictions were carried).
3. **Staleness.** Frames carry simulation time `t`; the server sends `behind` events when paced runs fall behind.
   The design defines the displayed-state age limit (spec 4 Task 0).
4. **Known behaviour to expect in demos.** The hierarchy can stall and switch sides near tight goals (Task 13,
   `v2_store_room_nav` seed 3; the route's contribution is unresolved); the diagnosis follows E5. A worker that stops in the lane usually ends a run as
   stalled or timed out (`v2_worker_stops_in_lane_nav`).
