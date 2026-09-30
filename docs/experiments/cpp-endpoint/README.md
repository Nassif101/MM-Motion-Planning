# C++ ROS-TCP endpoint

The Python `unity_control_endpoint.py` (upstream ROS-TCP-Endpoint with latest-only
command subscriptions) was replaced by the C++ `unity_control_endpoint` in
`mobile_manipulator_control`. It speaks the same ROS-TCP-Connector v0.7.0 protocol on the
same port, so Unity is unchanged:

- Unity -> ROS topics are forwarded as serialized CDR through `rclcpp` generic
  publishers. Each topic publishes from its own worker thread with a bounded queue (the
  Unity `queue_size`), so the socket reader never waits on DDS.
- ROS -> Unity topics come from generic subscriptions through one sender thread;
  `/arm/command` and `/cmd_vel` use DDS depth 1, as before.
- `__topic_list` is answered; ROS services are not supported (none are used by Unity
  in this project) and are answered with an error.
- TCP_NODELAY on accepted sockets, as before. SIGINT/SIGTERM stop it cleanly.

## Transport load

Method as in [lidar-transport](../lidar-transport/README.md): Unity out of Play,
`tools/run_nav_scenario.py --new-epoch open_space` with the endpoint under test,
15 s settling, then two 30 s windows of `sensor_transport_probe.py`. Both endpoints were
measured in the same session on Cyclone DDS (ADR 0008), so the Python numbers here
replace the Fast DDS figures in that report for comparison.

| Endpoint | Lidar MB/s (Hz) | Endpoint CPU | `/clock` gap p50 / p99 / max (ms) | `/arm/state` Hz, gap p99 / max (ms) |
|---|---:|---:|---|---|
| C++ 1 | 3.20 (10.0) | 5.9 % | 19.9 / 23.9 / 48.4 | 50.0, 24.1 / 49.0 |
| C++ 2 | 3.20 (10.0) | 5.6 % | 20.0 / 24.0 / 62.2 | 50.0, 24.1 / 63.7 |
| Python 1 | 3.21 (10.0) | 14.9 % | 20.1 / 23.7 / 52.9 | 50.0, 23.9 / 52.9 |
| Python 2 | 3.20 (10.0) | 15.1 % | 20.1 / 24.3 / 59.7 | 50.0, 24.2 / 59.9 |

The C++ endpoint uses about 40 % of the Python endpoint's CPU. Delivery rates and jitter
are the same: with Cyclone DDS the receiver-side gaps are set by Unity's frame timing,
not by the endpoint. (Python CPU was 42-48 % on Fast DDS; Cyclone alone cut it to 15 %.)

## Registration race in ROS-TCP-Connector

Bringing the C++ endpoint up against Unity exposed a Unity-side race. ROS-TCP-Connector
v0.7.0 builds queued system commands in one `MessageSerializer` shared by the main
thread and its connection thread, and the connection thread re-sends every registration
as soon as TCP connects. With `ConnectOnStart`, the connection came up while components
were still registering topics in `OnEnable`/`Start`, and registrations interleaved on
the wire: a command name paired with another command's JSON, duplicated or lost
registrations, and a JSON payload read as a destination. It occurred on 3 of 11 Play
starts. The race is in Unity and does not depend on the endpoint.

Two fixes:

- Unity connects one frame after Play starts, once every component has registered
  (`RosConnectAfterStart`, applied by `configure_mobile_manipulator`). Afterwards 8 of 8
  Play starts connected once with every topic registered exactly once.
- The endpoint checks frame sync: destination and payload lengths, topic-name
  characters, the exact JSON key set of each system command (Unity's `JsonUtility`
  writes every field), and data on an unregistered topic. On a failure it discards the
  connection's registrations and drops the connection; Unity reconnects after about 1 s
  and re-registers every topic from its connection thread alone. Before the Unity fix
  this recovered every corrupted start (one publisher per topic, no stray
  registrations).

## Functional check

- `tools/run_nav_scenario.py --new-epoch open_space_nav narrow_gate_vertical_carry_nav
  static_obstacle_detour_nav` (RPP): 3 of 3 succeeded without contact, including the arm
  move to vertical carry (`/arm/command`), the 1.05 m gate, and the unmapped-box detour
  (`/cmd_vel`, lidar). One connection, no frame-sync loss, no stale-feedback fault.
- Telemetry window after the runs: 1195 messages received, none rejected, not stale.
- Arm transport, 30 s of `measure_transport.py` in HOLD
  ([arm-transport.json](arm-transport.json)): `/arm/command`, `/arm/state` and `/clock`
  at 50.0 /s, largest wall-clock gap 0.058 s, no non-increasing stamps. This matches the
  Cyclone DDS confirmation with the Python endpoint (gaps up to 0.061 s).

- Stall phases from [ros-tcp-stalls](../ros-tcp-stalls/README.md) (3 min idle, 30 SIGINT
  and 30 SIGKILL graph-churn cycles, 8 Nav2 start/stop cycles, 1 min idle) on Cyclone DDS:
  no `/clock` gap of 0.15 s or more in any phase (worst per minute 0.044-0.102 s), Nav2
  active in all 8 cycles, no arm fault. The Python endpoint on Cyclone DDS also had
  none over 0.15 s.
- SIGINT and SIGTERM each stop the endpoint without escalation, including with Unity
  connected. (The Python endpoint ignored both and needed SIGKILL.)
