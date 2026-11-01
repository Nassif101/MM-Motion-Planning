# Phase 5A campaign B: every run

F2 (`--medium bezier`, 2.5 Hz profile) above B5 at 50 Hz with the base friction feedforward on, campaign 2's exit set, 2026-11-01 20:05-22:38Z at 0b6ba0a (`campaign_b.sh`). In run order. Mover clearance: the footprint's closest approach to a worker (Nav2's 2D footprint, not the 3D model).

| # | Cell | Run | Result | Time (s) | 3D clearance (m) | Closest obstacle | Mover clearance (m) | MPC p99 (ms) | Misses | Medium p99 (s) | Fallback (cycles of B5's) | Arm offset max (rad) | Sides | Switches |
|---:|---|---|---|---:|---:|---|---:|---:|---:|---:|---|---:|---|---:|
| 1 | constrained_reconfiguration_mission | 20261101T200543Z | success | 13.6 | 0.0854 | tall_box | - | 3.6 | 0 | 0.163 | 9 of 676 (no_plan 9) | 0.037 | - | 0 |
| 2 | constrained_reconfiguration_mission | 20261101T200919Z | success | 13.7 | 0.0857 | tall_box | - | 3.0 | 0 | 0.158 | 9 of 683 (no_plan 9) | 0.038 | - | 0 |
| 3 | constrained_reconfiguration_mission | 20261101T201257Z | success | 13.6 | 0.0779 | tall_box | - | 3.8 | 0 | 0.212 | 11 of 679 (no_plan 11) | 0.043 | - | 0 |
| 4 | constrained_reconfiguration_mission | 20261101T201635Z | success | 13.6 | 0.0861 | tall_box | - | 2.9 | 0 | 0.160 | 9 of 676 (no_plan 9) | 0.036 | - | 0 |
| 5 | constrained_reconfiguration_mission | 20261101T202012Z | success | 13.3 | 0.0886 | tall_box | - | 3.8 | 0 | 0.191 | 10 of 663 (no_plan 10) | 0.036 | - | 0 |
| 6 | narrow_gate_home_nav | 20261101T202350Z | success | 20.6 | 0.0458 | ManipulationRequiredGate_1p05m/GatePostLeft | - | 4.9 | 0 | 0.281 | 8 of 1028 (no_plan 6, route_id 2) | 0.029 | right | 0 |
| 7 | narrow_gate_home_nav | 20261101T202741Z | success | 20.5 | 0.0451 | ManipulationRequiredGate_1p05m/GatePostRight | - | 4.8 | 0 | 0.285 | 7 of 1024 (no_plan 5, route_id 2) | 0.023 | - | 0 |
| 8 | overhead_course_nav | 20261101T203131Z | success | 51.3 | 0.0454 | gate_post_north | - | 4.9 | 0 | 0.133 | 5 of 2563 (no_plan 5) | 0.038 | left > right | 1 |
| 9 | overhead_course_nav | 20261101T203604Z | success | 51.5 | 0.0468 | gate_post_south | - | 5.6 | 0 | 0.132 | 5 of 2571 (no_plan 5) | 0.037 | left | 0 |
| 10 | overhead_course_worker_nav | 20261101T204040Z | success | 51.5 | 0.0471 | gate_post_north | 0.126 | 5.1 | 0 | 0.136 | 5 of 2571 (no_plan 5) | 0.029 | left | 0 |
| 11 | overhead_course_worker_nav | 20261101T204515Z | success | 57.4 | 0.0407 | gate_post_south | 0.151 | 6.1 | 0 | 0.284 | 5 of 2869 (no_plan 5) | 0.040 | left | 0 |
| 12 | v2_chicane_nav | 20261101T204957Z | success | 96.9 | 0.1823 | Chicane4_1/proxy_0 | - | 7.0 | 0 | 0.125 | 3 of 4831 (no_plan 3) | 0.009 | - | 0 |
| 13 | v2_door_105_mission | 20261101T205518Z | success | 46.0 | 0.0469 | DoorFrame105/proxy_1 | - | 4.0 | 0 | 0.167 | 3 of 2300 (no_plan 3) | 0.027 | right | 0 |
| 14 | v2_door_105_nav | 20261101T205940Z | success | 46.3 | 0.0421 | DoorFrame105/proxy_1 | - | 3.9 | 0 | 0.107 | 3 of 2312 (no_plan 3) | 0.018 | - | 0 |
| 15 | v2_door_130_nav | 20261101T210401Z | success | 51.3 | 0.0496 | LaneDoor130_West/proxy_0 | - | 4.6 | 0 | 0.129 | 4 of 2562 (no_plan 4) | 0.018 | - | 0 |
| 16 | v2_gate_right_swing_nav | 20261101T210831Z | success | 33.0 | 0.0519 | approach_shelf | - | 4.7 | 0 | 0.268 | 4 of 1650 (no_plan 4) | 0.025 | left | 0 |
| 17 | v2_goal_near_skip_nav | 20261101T211241Z | success | 36.3 | 1.4020 | Row1_03/proxy_0 | - | 2.9 | 0 | 0.049 | 3 of 1814 (no_plan 3) | 0.003 | - | 0 |
| 18 | v2_open_yard_nav | 20261101T211657Z | success | 40.0 | 1.7333 | Row1_04/proxy_0 | - | 3.1 | 0 | 0.039 | 3 of 2000 (no_plan 3) | 0.004 | - | 0 |
| 19 | v2_room_r1_nav | 20261101T212115Z | success | 51.7 | 0.0487 | LaneDoor130_West/proxy_0 | - | 4.5 | 0 | 0.128 | 3 of 2582 (no_plan 3) | 0.018 | - | 0 |
| 20 | v2_room_r4_nav | 20261101T212553Z | success | 66.2 | 0.0464 | DoorR4Frame/proxy_1 | - | 7.0 | 0 | 0.181 | 11 of 3305 (no_plan 4, route_id 7) | 0.025 | - | 0 |
| 21 | v2_room_r4_nav | 20261101T213046Z | success | 65.7 | 0.0444 | DoorFrame130/proxy_0 | - | 7.2 | 0 | 0.172 | 12 of 3280 (no_plan 4, route_id 8) | 0.028 | right | 0 |
| 22 | v2_room_r4_nav | 20261101T213540Z | success | 66.1 | 0.0423 | DoorR4Frame/proxy_1 | - | 7.3 | 0 | 0.283 | 3 of 3302 (no_plan 3) | 0.019 | left | 0 |
| 23 | v2_scaffold_tunnel_mission | 20261101T214035Z | success | 71.7 | 0.1247 | H1Ledger/proxy_1 | - | 4.8 | 0 | 0.097 | 3 of 3575 (no_plan 3) | 0.004 | - | 0 |
| 24 | v2_scaffold_tunnel_nav | 20261101T214540Z | success | 72.6 | 0.1244 | H1Ledger/proxy_1 | - | 5.3 | 0 | 0.107 | 4 of 3618 (no_plan 4) | 0.004 | - | 0 |
| 25 | v2_services_deadend_nav | 20261101T215044Z | success | 71.4 | 0.1867 | PipesSouth/proxy_2 | - | 6.4 | 0 | 0.142 | 5 of 3545 (no_plan 5) | 0.006 | - | 0 |
| 26 | v2_spine_nav | 20261101T215546Z | success | 129.1 | 2.4097 | LaneDoor130_West/proxy_0 | - | 5.2 | 0 | 0.060 | 4 of 6451 (no_plan 4) | 0.004 | - | 0 |
| 27 | v2_store_room_nav | 20261101T220146Z | success | 92.3 | 0.0441 | DoorFrame105/proxy_0 | - | 7.5 | 0 | 0.283 | 64 of 4597 (no_plan 4, route_id 60) | 0.023 | right | 0 |
| 28 | v2_store_room_nav | 20261101T220707Z | aborted | 157.7 | 0.0419 | DoorSFrame/proxy_0 | - | 12.5 | 0 | 0.282 | 434 of 7319 (no_plan 4, route_id 430) | 0.018 | left | 0 |
| 29 | v2_store_room_nav | 20261101T221336Z | success | 67.7 | 0.0432 | DoorSFrame/proxy_0 | - | 8.0 | 0 | 0.284 | 16 of 3366 (no_plan 4, route_id 12) | 0.030 | right > left | 1 |
| 30 | v2_through_building_mission | 20261101T221833Z | success | 17.8 | 3.0187 | LaneDoor130_West/proxy_0 | - | 4.4 | 0 | 0.090 | 5 of 891 (no_plan 5) | 0.004 | - | 0 |
| 31 | v2_worker_crossing_nav | 20261101T222236Z | success | 31.5 | 3.0978 | Chicane1_6/proxy_0 | 0.334 | 3.5 | 0 | 0.057 | 4 of 1574 (no_plan 4) | 0.002 | - | 0 |
| 32 | v2_worker_late_crossing_nav | 20261101T222654Z | success | 33.9 | 2.1770 | LaneDoor130_West/proxy_0 | 0.000 | 6.3 | 0 | 0.053 | 4 of 1691 (no_plan 3, route_id 1) | 0.007 | - | 0 |
| 33 | v2_worker_stops_in_lane_nav | 20261101T223116Z | success | 32.3 | 3.0201 | PanelsEast_1/proxy_0 | 0.025 | 5.0 | 0 | 0.063 | 5 of 1615 (no_plan 4, route_id 1) | 0.004 | - | 0 |
| 34 | v2_yard_store_nav | 20261101T223536Z | success | 98.0 | 0.1824 | Row2_05/proxy_0 | - | 6.1 | 0 | 0.090 | 4 of 4897 (no_plan 4) | 0.005 | - | 0 |
