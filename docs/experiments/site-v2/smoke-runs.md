# V2 smoke runs and real-time factor

Measured 2026-10-17 on the Apple M4 host (Unity 6000.5.2f1, container aarch64, 10 CPUs), site
`construction_site_v2` unless noted. Controllers run with current defaults (`--controller rpp` for
B1, `--controller wholebody` for B5). Run directories (under `experiment_runs/`, git-ignored) are
referenced by name.

## Real-time factor

| Scenario | Site | Controller | Real-time factor | Run |
|---|---|---|---|---|
| `open_space_nav` | V1 | rpp | 1.000 | `20261017T114538Z-open_space_nav-rpp` |
| `v2_open_yard_nav` | V2 | rpp | 1.004 | `20261017T114719Z-v2_open_yard_nav-rpp` |

V2 is at least 1.00, so no baked-lighting attempt was needed. Every later V2 run recorded 0.999 to
1.001. Both open-space runs succeeded with no contact.

## Smoke runs

All eight runs were sequential, one runner at a time, with the V1 to V2 scene switch happening once
(in the first V2 run). No run started a mission, so none was a B3 mission start.

| Scenario | Controller | Outcome | Expected | Contact | RTF | MPC p99 | Notes | Run |
|---|---|---|---|---|---|---|---|---|
| `v2_scaffold_tunnel_nav` | rpp | aborted, 45 s, 6.1 m, final error 11.7 m, 15 recoveries | pass | none | 1.001 | n/a | Unexpected: B1 stopped at y 6.1 in front of the H1 ledger because the local costmap marks it lethal (see Findings 1). | `20261017T114925Z-v2_scaffold_tunnel_nav-rpp` |
| `v2_scaffold_tunnel_nav` | wholebody | succeeded, 77.7 s, 20.0 m, final error 0.143 m | pass | none | 1.000 | 49.0 ms | Held carry throughout; panel 3D clearance 0.047 m to H1Ledger; 15 deadline misses, 31 near-misses. | `20261017T115836Z-v2_scaffold_tunnel_nav-wholebody` |
| `v2_door_105_nav` | rpp | aborted, 59 s, path 0.0 m, 13 recoveries | fail (carry wider than door) | none | 1.000 | n/a | As expected (could not plan). | `20261017T115119Z-v2_door_105_nav-rpp` |
| `v2_door_105_nav` | wholebody | aborted, 142 s, 6.9 m, final error 4.2 m | pass (reconfigure to vertical carry) | none | 1.000 | 52.2 ms | Unexpected: B5 reconfigured to vertical carry but stalled 0.3 m before the door (see Findings 3); wheel link 0.107 m from DoorFrame105 at closest. | `20261017T120125Z-v2_door_105_nav-wholebody` |
| `v2_chicane_nav` | rpp | aborted, 139 s, 29.0 m, final error 0.149 m / 0.839 rad, 11 recoveries | pass | none | 1.000 | n/a | Reached the goal position; the RPP rotate deadlock blocked the last 0.84 rad turn (see Findings 2); min clearance 0.171 m; 11 monitor stops. | `20261017T115344Z-v2_chicane_nav-rpp` |
| `v2_chicane_nav` | wholebody | succeeded, 150 s, 26.4 m, final error 0.102 m | pass | none | 1.001 | 46.5 ms | Panel 3D clearance 0.046 m to Chicane1_6; 21 deadline misses. | `20261017T120506Z-v2_chicane_nav-wholebody` |
| `v2_spine_rough_nav` | rpp | succeeded, 128.0 s, 34.87 m, final error 0.145 m | pass | none | 1.035 | n/a | Rerun after the contact-monitor fix (first run, `20261017T115724Z-v2_spine_rough_nav-rpp`, was cancelled by the strip contact). `disturbances: true`. | `20261017T121945Z-v2_spine_rough_nav-rpp` |
| `v2_spine_rough_nav` | wholebody | succeeded, 129.0 s, 34.86 m, final error 0.148 m | pass | none | 1.002 | 50.4 ms | Rerun after the fix (first run `20261017T120915Z-v2_spine_rough_nav-wholebody` cancelled by the strip contact). Held carry throughout; costmap check below ran during this run. | `20261017T122303Z-v2_spine_rough_nav-wholebody` |

### Contact monitor fix for `v2_spine_rough_nav`

The first runs of both `v2_spine_rough_nav` rows were cancelled by the runner's contact monitor after
about 1 to 2 m, at the rumble strips (x -19.8..-18.6): a wheel against a `proxy_N` at 2 to 3 mm
penetration. That is the intended Tier 2 disturbance, so `RobotContactMonitor` now ignores a contact
when the robot collider is under a `*wheel*` link and the other collider is under `Disturbances`
(any other robot link touching a disturbance still counts; classifier `ContactClassifier`, EditMode
test `ContactLedgerTests.IgnoresOnlyWheelContactsWithDisturbances`). Both reruns completed with
`disturbances: true` and no contact.

## Costmap check on the road plates (disturbances on)

Disturbances on (`disturbances: true` in the summary), the global costmap up during the full
wholebody `v2_spine_rough_nav` rerun (`20261017T122303Z-v2_spine_rough_nav-wholebody`, which drives
x -21 to 14, across the plates at x 9..11). A probe in the container subscribed to
`/global_costmap/costmap` for 240 s and checked the 4000 cells under x 9..11, y -2.5..2.5 on each of
102 messages.

Result: maximum cost 0 and 0 occupied cells (threshold 65) at the plates on all 102 messages. No
costmap marks on the plates. An earlier manual drive (robot driven with `/cmd_vel` 0.3 m/s from
x 3.7 to 13.5, 28 messages) gave the same result. The rpp rerun was not sampled; the costmap is the
same for both controllers. The costmap has no other cells to compare: the check only shows the
plates add none.

## Findings on the unexpected outcomes (diagnosed, not fixed)

1. **B1 rpp `v2_scaffold_tunnel_nav`: not the RPP deadlock, a perception/expectation mismatch.** The
   robot drove straight north (yaw 1.57, no turn involved) to y 6.1 and stopped, "Controller
   patience exceeded" (error 104). In the last `/local_costmap/costmap` of the bag, the cells for
   y 6.6..7.4 across the whole lane x -7.5..-5.5 are 99 (lethal): the H1 ledger (1.42 m high) is
   marked because `nav2_local_costmap.yaml` takes obstacle points up to `max_obstacle_height: 2.0`,
   and RPP's collision detection then refuses to move into it. `/cmd_vel_nav` is zero after
   t = 1792237856. The global planner and `check_site_plans.py` use the 2D static map without the
   ledger, so planning passes. So the scenario description's "B1/B2/B4-static expected to pass" does
   not hold for a 2D-costmap baseline: the ledger is an obstacle to them by construction. Not a start
   or goal yaw problem, so no scenario change; the expected outcome for B1/B2 (and any controller
   that uses this local costmap) is a failure at the ledger. Controller decision needed: change the
   expectation text, or accept this as the intended overhead-blind result.
2. **B1 rpp `v2_chicane_nav`: (a) known RPP rotate deadlock.** The chicane was driven fully (29.0 m);
   the robot arrived at the goal position (-18.1, 16.89) with yaw 0.73 against a goal yaw of 1.57
   and sat still for the last 15 s of the trajectory. `/cmd_vel_nav` in the last 20 s is
   angular.z 0.043 to 0.045 rad/s, linear 0 (the 0.04 rad/s clamp below the 0.07 breakaway); the
   error was "Failed to make progress" after nine "0 poses" plan errors. wholebody on the same goal
   succeeded. No change.
3. **B5 wholebody `v2_door_105_nav`: (b) MPC behaviour, a Phase 4B item.** The arm trace shows the
   reconfiguration worked: carry until t = 21 s, a 5 s transit, then vertical_carry held for 104 s.
   The robot reached (16.18, 5.27), yaw 1.51, centred on door B (x 16.25) about 0.3 m short of the
   wall plane (y about 5.5), and stayed there for about 100 s. The global plan is valid (202 poses
   ending at (16.25, 9.5)) and the door frame is in the MPC's known boxes (`DoorFrame105/proxy_0`
   appears in the 3D clearance at 0.107 m for a wheel). MPC status was `ok` on all 2629 cycles:
   no infeasible, no fallback, no hold, slack about 1e-11 m, `active_pairs` 168, `tightness` 1.0,
   `manipulability` 0.044 at the last cycle. So the MPC does not report a failure; it commands
   (near) zero motion at the door approach. Not a geometry defect found in the artifacts (the 0.77 m
   vertical-carry footprint in a 1.05 m door leaves about 0.14 m a side); B5 at a 1.05 m door from a
   carry start is a Phase 4B item.

## MPC timing and MoveIt

- MPC cycle p99 on the tunnel run (wholebody): 49.0 ms (deadline misses 15, near-misses 31). Other
  wholebody rows: door 52.2 ms, chicane 46.5 ms, spine-rough 88.9 ms (7.5 s run, arm restore).
- MoveIt planning-box count: the only MoveIt start was the arm restore in the wholebody
  `v2_spine_rough_nav` run, "Loaded 600 static and 0 scenario boxes, the floor (top 0.15 m) and the
  attached panel" (`/tmp/mm_moveit.log` in the container). No run was a B3 mission.
