# Unity telemetry window: cost and verification

Date: 2026-09-28. `TelemetryWindow` (on `SimulationROS`, wired by
`configure_mobile_manipulator`) is toggled by an on-screen button (top-right) or the `T`
key, or by `unity command telemetry_window --visible true|false`. It shows Run, Path,
Motion, Safety, Arm, and Link sections and a merged ROS/Unity event log, and draws the
planned path, goal ring, and executed trail in the scene while visible.

Data sources: base motion, applied command, watchdog, arm state and joint error, panel
clearance, contacts, and clock are read in Unity. ROS-side navigation state comes from
`nav_telemetry`: `/mm/telemetry` (JSON, 5 Hz, 638 bytes measured) and
`/mm/telemetry/path` (downsampled plan, on change and every 5 s; 34 poses for a 4 m plan).

## Cost

Each condition: fresh endpoint and Play epoch (`run_nav_scenario.py --new-epoch
open_space_nav`), full navigation stack, 10 s settle, two 30 s windows of
`sensor_transport_probe.py`, Unity frame rate from the frame counter over the same windows.

| Condition | Unity FPS | Endpoint CPU | `/clock` gap p50 / p99 / max (ms) |
|---|---:|---:|---|
| E: no window | 204-206 | 50-53 % | 19.7 / 39-42 / 73-687 |
| F: window present, hidden | 205-216 | 29-30 % | 19.9 / 24.5 / 70-680 |
| G: window visible with overlay | 194-196 | 29 % | 19.9 / 25.7 / 49-55 |

The window adds no measurable transport cost. The drop in endpoint CPU and jitter from E to
F is not caused by the window (it adds traffic); epoch-to-epoch variation of the endpoint
is larger than these effects, as in the lidar-transport measurements. Visible, the window
costs about 5 % frame rate (IMGUI text redrawn each frame, rebuilt at 5 Hz, and three line
renderers); hidden, the cost is within noise. The Editor Game view was 784 x 880 at 257 dpi.

## Live verification

During an `open_space_nav` run the window showed: goal executing, 7.3 s elapsed, ETA 7.4 s,
2.07 m remaining, plan 4.00 m / 66 points, cross-track 0.002 m, actual 0.27 m/s against an
applied 0.30 m/s, watchdog "commanded", collision monitor clear, no contacts; the planned
path and goal ring were drawn. Fixes found during verification: the toggle overlapped the
ROS-TCP-Connector HUD (moved top-right), an idle base was flagged as "STOPPED" (now flagged
only while a goal executes), and text from the previous opening was shown for one refresh
(now rebuilt when shown).

Note: a runtime `unity command eval` (Roslyn) during a run stalled the Editor long enough for
the arm hardware interface to report stale feedback and deactivate, as documented for
qualification runs; use compiled commands such as `telemetry_window` during runs.

## 2026-10-01 C++ node

`nav_telemetry` is now a C++ (`rclcpp`) node with the same topics, QoS and JSON fields. It
was run next to the previous Python node (remapped) over `open_space_nav`,
`static_obstacle_detour_nav` and `narrow_gate_home_nav`. In 719 snapshots taken while both
tracked the same goal, the goal status, recoveries, path point count and collision-monitor
state were identical. The numeric fields agreed within the 0.1 s offset between the two
timers, and both published the same downsampled paths. One difference is a fix: the Python
`path_length` measured `(x, y, yaw)` points, so heading changes inflated the plan length on
curved paths (22.0 m reported for the 8.6 m detour plan). The C++ node measures x and y
only. Only the window's display and its "new plan" events were affected; the scenario
results compute path lengths separately.
