# Unity -> ROS stream pauses at the ROS-TCP endpoint

Date: 2026-09-28/29 (Unity 6000.5.2f1, ROS 2 Jazzy, Fast DDS via `rmw_fastrtps_cpp`).

## Problem

During the Phase 1 controller comparison, 3 of 45 runs could not start because the ROS
arm hardware had faulted with "Unity feedback stale": `/clock` and `/arm/state` reached
ROS with pauses of 0.25-0.4 s about once a minute and occasionally 0.6-1.0 s, clustered
around Nav2 launches and shutdowns. The arm hardware trips at 0.5 s.

## Method

Diagnostics were run in the container against a fresh simulation epoch each time. They
were ad hoc scripts, not part of the repository:

- a copy of `unity_control_endpoint.py` that timed every socket read and every
  `publish()` call, logged Python garbage-collection pauses, and recorded the `/clock` sim
  times that followed each gap;
- a ROS-side subscriber logging wall-clock arrival gaps of `/clock` and `/arm/state`;
- a 50 Hz sampler of the endpoint's threads (`/proc/<pid>/task/*/schedstat`: CPU time,
  run-queue wait, state, wait channel) and of VM-wide CPU use.

Phases per run: 3 min idle; 30 start/stop cycles of a small node subscribing `/clock`,
`/tf`, and `/livox/lidar`, stopped with SIGINT, then 30 stopped with SIGKILL; Nav2
start/stop cycles; 1 min idle. The phases were repeated with the endpoint's DDS
transport changed. One set was discarded because the host went into low-battery sleep
for 21 minutes during it; later runs used `caffeinate`.

## Results

| Condition | `/clock` gaps on the ROS side |
|---|---|
| Idle (4 runs x 3 min) | 0 |
| 60 `ros2 lifecycle get` calls on an idle system | 0 |
| Node stopped cleanly (SIGINT), 30 cycles | 0-4, about 0.3 s |
| Node killed (SIGKILL), 30 cycles | 5-7, repeatable 0.31-0.34 s |
| Nav2 start/stop, 5 cycles, clean state | 3, max 0.35 s |
| Nav2 start/stop with CLI polling, after 30 SIGKILLed nodes | 289 (20 of 0.5 s or more), max 2.9 s |

Endpoint transport variants on the same phases (Nav2 cycles): default shared memory 20
gaps of 0.5 s or more (max 2.9 s); `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS` 31 (max
2.8 s); `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 12 but max 9.0 s, with lidar publishes of up
to 0.44 s. No transport setting removes the pauses.

## Findings

- The pauses are inside the endpoint's `publish()` calls. At all but one pause per run,
  Unity's next message was already waiting on the socket, and the `/clock` stamps after
  each gap show that Unity kept simulating in real time. Unity, the Docker network, and
  Python garbage collection are not the source.
- It is not CPU starvation. During a 0.31 s blocked `publish()` of `/clock` the reading
  thread was asleep the whole time (0 ms CPU, 0 ms run-queue wait) while the VM was
  40-60 % busy and Fast DDS's own threads used about 4 ms each.
- The block is process-wide: while a node that subscribed only `/clock`, `/tf`, and
  `/livox/lidar` was being killed, publishes of `/arm/state`, `/odom`, and
  `/joint_states` also blocked for up to 0.3 s. Per-topic threads in the endpoint would
  therefore not isolate the control streams.
- The trigger is DDS graph change, above all processes that exit uncleanly. Nav2's
  planner, controller, and BT navigator servers regularly ignore SIGINT for 5 s during
  shutdown and are then terminated with SIGTERM.
- Because the upstream endpoint reads Unity's socket and publishes each message on one
  thread, one blocked `publish()` delays every Unity stream at once, and bursts of such
  blocks add up to pauses of 0.5-3 s.
- Effect on the arm: the controller manager runs on sim time, so when `/clock` pauses
  with the state the control loop simply pauses. It faults when `/clock` resumes a moment
  before `/arm/state` and the state is then more than 0.5 s old.
- Effect on navigation results: during the comparison runs themselves the worst gap was
  about 0.1 s per minute; the pauses cluster at transitions between runs.

Not established: which Fast DDS mechanism performs the roughly 0.3 s timed wait.

## Consequences

- The 0.5 s arm feedback timeout is kept (ADR 0005 amendment). Stale feedback now
  deactivates the arm hardware without latching, and `arm_recovery_supervisor.py`
  re-activates it; a simulation-epoch change stays latched.
- Next: compare Cyclone DDS and Zenoh (both installed) with the same phases (below).

## 2026-09-29 Nav2 shutdown and recovery check

Five Nav2 start/stop cycles each, stopped either by SIGINT to the launch (as the runner
does) or by an orderly lifecycle SHUTDOWN of both managers followed by SIGINT:

| Stop | Duration | SIGTERM escalations | Stops with a `/clock` gap of 0.5 s or more | Max gap |
|---|---|---|---|---|
| SIGINT | 8-10 s | 0 | 1 of 5 | 2.0 s |
| Orderly SHUTDOWN, then SIGINT | 15-24 s | 4 (one cycle) | 3 of 5 | 2.0 s |

On a healthy graph Nav2 already exits cleanly on SIGINT; the escalations seen earlier
came after the SIGKILL phases. The orderly shutdown is slower and no better, so the
runner keeps SIGINT. Clean exits still produce pauses of about 2 s in some stops, so
shutdown cleanliness is not the lever.

During these cycles the arm hardware tripped three times (state 1.4-2.0 s old, or
`/clock` 0.5 s behind the state) and `arm_recovery_supervisor` restored arm control each
time within 0.6-1.7 s, without a restart.

## 2026-09-29/30 middleware comparison

Every ROS process in the container (description launch, endpoint, arm control, Nav2,
probes) was restarted with the middleware under test; Zenoh also ran its router. Phases:
3 min idle, 30 SIGINT and 30 SIGKILL churn cycles, 8 Nav2 start/stop cycles (25 s up, no
CLI polling), 1 min idle. Counts are `/clock` gaps on the ROS side; `/arm/state` matched.

| Middleware | Idle | Churn SIGINT | Churn SIGKILL | Nav2 cycles | Arm faults |
|---|---|---|---|---|---|
| Fast DDS (`rmw_fastrtps_cpp`, current) | 0 | 0 | 1 over 0.25 s (max 0.34 s) | 2 over 0.25 s (max 0.34 s) | 0 |
| Cyclone DDS (`rmw_cyclonedds_cpp`) | 0 | 0 | 0 | 0 | 0 |
| Zenoh (`rmw_zenoh_cpp`) | 1 of 0.83 s | 0 | 0 | 0 | 1, recovered |

Nav2 brought up all its nodes in all 8 cycles with each middleware. This Fast DDS run,
from a clean state, was milder than earlier ones (20 gaps of 0.5 s or more in the same
phases), so the difference is if anything understated. Cyclone DDS produced no pause
over 0.15 s in about 12 minutes that included the SIGKILL churn which reliably blocks
Fast DDS. One run per middleware; a switch would need a longer confirmation run and a
re-check of the arm transport measurements (ADR 0005).
