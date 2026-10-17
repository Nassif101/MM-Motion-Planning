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
| `v2_scaffold_tunnel_nav` | rpp | aborted, 45 s, 6.1 m, final error 11.7 m, 15 recoveries | pass | none | 1.001 | n/a | Unexpected: B1 should pass. Failed in the tunnel/pocket region; not diagnosed here. | `20261017T114925Z-v2_scaffold_tunnel_nav-rpp` |
| `v2_scaffold_tunnel_nav` | wholebody | succeeded, 77.7 s, 20.0 m, final error 0.143 m | pass | none | 1.000 | 49.0 ms | Held carry throughout; panel 3D clearance 0.047 m to H1Ledger; 15 deadline misses, 31 near-misses. | `20261017T115836Z-v2_scaffold_tunnel_nav-wholebody` |
| `v2_door_105_nav` | rpp | aborted, 59 s, path 0.0 m, 13 recoveries | fail (carry wider than door) | none | 1.000 | n/a | As expected (could not plan). | `20261017T115119Z-v2_door_105_nav-rpp` |
| `v2_door_105_nav` | wholebody | aborted, 142 s, 6.9 m, final error 4.2 m | pass (reconfigure to vertical carry) | none | 1.000 | 52.2 ms | Unexpected: B5 did not get through door B; wheel link 0.107 m from DoorFrame105 at closest. | `20261017T120125Z-v2_door_105_nav-wholebody` |
| `v2_chicane_nav` | rpp | aborted, 139 s, 29.0 m, final error 0.149 m / 0.839 rad, 11 recoveries | pass | none | 1.000 | n/a | Reached the goal position, not the heading; min clearance 0.171 m; 11 monitor stops. | `20261017T115344Z-v2_chicane_nav-rpp` |
| `v2_chicane_nav` | wholebody | succeeded, 150 s, 26.4 m, final error 0.102 m | pass | none | 1.001 | 46.5 ms | Panel 3D clearance 0.046 m to Chicane1_6; 21 deadline misses. | `20261017T120506Z-v2_chicane_nav-wholebody` |
| `v2_spine_rough_nav` | rpp | canceled by contact after 6.6 s, 1.05 m | pass | Cylinder (wheel) with `proxy_1`, 2.1 mm | 1.001 | n/a | `disturbances: true` recorded. See the contact note below. | `20261017T115724Z-v2_spine_rough_nav-rpp` |
| `v2_spine_rough_nav` | wholebody | canceled by contact after 11.8 s, 1.9 m | pass | Cylinder (wheel) with `proxy_1` (2.6 mm) and `proxy_0` (1.6 mm) | 0.999 | 88.9 ms (short run) | `disturbances: true` recorded. MoveIt was started to restore the arm pose: "Loaded 600 static and 0 scenario boxes". | `20261017T120915Z-v2_spine_rough_nav-wholebody` |

### Contact note for `v2_spine_rough_nav`

Both rows were cancelled by the runner's contact monitor after about 1 to 2 m of travel from the
start at x = -21, which is where the rumble strips lie (x -19.8..-18.6). The reported contact is a
robot wheel against a `proxy_N` at 2 to 3 mm penetration. This is the intended Tier 2 disturbance
(a chamfered strip of at most 35 mm that the wheels drive over), but the contact monitor treats it
like an obstacle hit and cancels. As a result, neither rough row can complete, and the "no contact
on B5 rows" check fails for this scenario with disturbances on. The runner or the contact monitor
needs to ignore `Environment/Disturbances` proxies (or the scenario to expect it); this task did
not change either. The disturbance rows also never reached the road plates, hence the separate
costmap check below.

## Costmap check on the road plates (disturbances on)

Separate check, as the ruling allows: V2 in Play, `site_disturbances` enabled (confirmed `enabled:
true, count: 5`), Nav2 global costmap up (from a `--plan-only` run of `v2_spine_rough_nav`), robot
driven with `/cmd_vel` linear.x 0.3 m/s from x = 3.7 to x = 13.5, across the plates at x 9..11. A
probe in the container subscribed to `/global_costmap/costmap` for 40 s and checked the 4000 cells
under x 9..11, y -2.5..2.5 on each of 28 costmap messages.

Result: the maximum cost in that region was 0 on every message, with 0 occupied cells (threshold 65)
at the plates. No costmap marks on the plates. A first pass (79 messages, robot driven from the
gate to x = 10.8) gave the same result, but the disturbances had been switched off again by the
plan-only run at the end, so it is not counted.

Caveat: the whole costmap reports 320800 cells at or above 65 (outside the mapped site and the
static obstacles), and the check only shows the plates add none.

## MPC timing and MoveIt

- MPC cycle p99 on the tunnel run (wholebody): 49.0 ms (deadline misses 15, near-misses 31). Other
  wholebody rows: door 52.2 ms, chicane 46.5 ms, spine-rough 88.9 ms (7.5 s run, arm restore).
- MoveIt planning-box count: the only MoveIt start was the arm restore in the wholebody
  `v2_spine_rough_nav` run, "Loaded 600 static and 0 scenario boxes, the floor (top 0.15 m) and the
  attached panel" (`/tmp/mm_moveit.log` in the container). No run was a B3 mission.
