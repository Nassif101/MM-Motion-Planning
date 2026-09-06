# Physics clock and payload qualification for planner research

## Decision

Keep the six-joint arm geometry and finite force drives. Qualify the 3 kg reference
panel with explicit trajectories and base commands, without MoveIt. Use compact
vertical carry for narrow passages and wrist-compensated level extension for reach.

Publish `/clock` in `FixedUpdate` before arm transport/actuation. Accumulate integer
nanoseconds with the configured physics period rounded to microsecond precision.
Clock and arm feedback use this time; command-age validation uses the same epoch.
Unity's configured 20 ms step was observed as 0.0199999921 s. Passing that directly
to ROS can make an exact 20 ms timer wait for another tick. Tests cover one hour of
exact timestamp progression, step changes and new-session reset.

Use the project-owned `unity_control_endpoint.py` entry point on the existing TCP
port. It retains upstream protocol, sensor publishing and services, but uses DDS
depth one for `/arm/command` and `/cmd_vel`, two executor workers, and TCP_NODELAY
on accepted sockets to flush small command packets without Nagle coalescing. Suppress
duplicate-time writes in the ros2_control hardware plugin. Keep the 0.5 s watchdog
and 0.25 s maximum packet age; do not mask delivery failures by widening them.

## Evidence and alternatives

Raw floating-point physics timestamps yielded approximately 25 command packets/s.
Integer clock steps improved this to approximately 47/s. The upstream endpoint
still produced occasional stale-command watchdog events. With the endpoint change,
the subsequent 30 s observation measured about 49.7 command packets/s, with no
non-increasing command stamps. These are measurements, not hard real-time guarantees.

Subsequent 15/30 FPS trials exposed intermittent watchdog events despite correct
50 Hz physics timestamps. The upstream sockets did not disable Nagle coalescing.
The project endpoint now sets TCP_NODELAY without editing the Unity package cache.
The final low-frame-rate reruns and their observed delivery rates are reported in
the qualification evidence; rendering still batches Connector callback dispatch.

A larger pedestal is unnecessary for the selected poses. Higher drive gains would
not fix panel/fence contact, which contaminated one extended-arm disturbance trial.
That trial is retained and excluded from contact-free qualification. Stability tests
now start in the open area at Unity `(0, 0.21, 12)`; the gate test uses the actual
1.05 m scene opening. Test placement is a runtime fixture, not claimed navigation.

Compiled Pipeline commands replace runtime C# evaluation during measurement because
Roslyn compilation can stall physics and trip watchdogs. The recorder observes
physical joint error, drive contribution, panel penetration, floor clearance, base
tilt and the entire robot's world-space collider bounds. Any detected panel
penetration disqualifies contact-free acceptance.

## Consequences and revisit conditions

ROS still owns trajectories/control policy; Unity owns physics and finite actuation.
Rendering and network delivery remain asynchronous. A future MPC must account for
the measured delivery delay and tracking error and perform its own collision and
stability checks. Feedback faults and clock resets require explicit controller
recovery. Pauses freeze physics; fresh feedback may permit normal HOLD on resume,
otherwise recover the faulted manager. Requalify changes to payload mass/COM, arm geometry, actuator
gains, timestep, operating velocities or transport packages.

See the [qualification report](../experiments/arm-controller/qualification/README.md)
for final measurements, reproducible commands and operating limits.
