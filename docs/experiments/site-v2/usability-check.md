# ConstructionSiteV2 usability check (2026-10-17)

Pre-merge check of the V2 site on branch `site-overhaul` after the MPC collision prefilter (ee83ae3) and the
door approach lanes (8fc036c, 94c6ebb). Every run: one repeat, Unity real-time factor 1.00 (from the bag's
/clock against the recording time; 0.996 to 1.002), no robot-environment contact in any run, `experiment_runs/`
directories named by the run ids below (git-ignored). B5 = `--controller wholebody --mpc-rate-hz 50`
(comfort pose carry, slack gate on, the current defaults), B1 = `--controller rpp`, B3 = `--controller rpp` on a
`_mission` scenario (scripted reconfigure steps with MoveIt).

An earlier first attempt of this campaign was discarded: the description/TF stack was dead (only a stale
`map_to_odom` publisher from 07:00 was left, so `demo.py up` believed the stack was running), the lidar filter
could not transform and the collision monitor held "invalid source" with the robot at rest. After killing the
stale publisher and restarting `simulation.launch.py` all numbers below were recorded. `demo.py up` should not
treat the lone `map_to_odom` publisher as the whole stack (see Findings, item 5).

## B5 on the 20 V2 scenarios (50 Hz)

MPC p99 is the node's total cycle time; misses are cycles over the 20 ms period.

| Scenario | Outcome | Expected | MPC total p99 / misses | Class | Evidence |
|---|---|---|---|---|---|
| v2_open_yard_nav | succeeded 39.7 s, 10.2 m | pass | 21.2 ms / 32 | | |
| v2_door_130_nav | succeeded 74.7 s, 13.4 m | pass | 23.6 ms / 74 | | |
| v2_door_105_nav | succeeded 66.1 s, 11.8 m | pass | 19.3 ms / 25 | | the earlier stop-zone stall at door B is gone: lane approach, no StopZone event |
| v2_door_105_mission | succeeded 66.2 s, 11.8 m | pass | 21.6 ms / 39 | | |
| v2_through_building_mission | succeeded 21.9 s, 5.2 m | pass (final goal only) | 19.3 ms / 9 | | |
| v2_room_r1_nav | succeeded 77.0 s, 13.2 m | pass | 29.9 ms / 164 | | |
| v2_room_r4_nav | **aborted** 186.7 s, 15.5 m, 10 recoveries | pass | 33.4 ms / 522 | (a) with (c) | see Failure 1 |
| v2_store_room_nav | **aborted** 146.3 s, 11.6 m, 33 recoveries | pass | 41.1 ms / 753 | (a) with (c) | see Failure 2 |
| v2_yard_store_nav | succeeded 115.9 s, 26.3 m | pass | 30.5 ms / 222 | | |
| v2_scaffold_tunnel_nav | succeeded 77.7 s, 19.9 m | pass (B5) | 27.1 ms / 99 | | |
| v2_scaffold_tunnel_mission | succeeded 79.5 s, 20.0 m | pass (B5) | 26.9 ms / 112 | | |
| v2_chicane_nav | succeeded 151.4 s, 26.6 m | pass | 13.2 ms / 16 | | 18 monitor events (slowdown), no stop |
| v2_services_deadend_nav | succeeded 86.9 s, 19.4 m | pass | 20.5 ms / 51 | | |
| v2_goal_near_skip_nav | succeeded 35.9 s, 9.2 m | pass | 20.3 ms / 22 | | |
| v2_worker_crossing_nav | succeeded 32.5 s, 7.9 m | pass | 25.2 ms / 43 | | |
| v2_worker_late_crossing_nav | **canceled** at the 90 s timeout, 3.3 m | pass | 29.5 ms / 143 | (c) | see Failure 3 |
| v2_worker_stops_in_lane_nav | succeeded 42.8 s, 8.3 m | pass | 20.1 ms / 22 | | worker waits in the lane; robot stops and resumes |
| v2_spine_nav | succeeded 128.7 s, 34.9 m | pass | 24.6 ms / 111 | | |
| v2_spine_rough_nav | succeeded 129.2 s, 34.9 m | pass | 23.5 ms / 116 | | |
| v2_scaffold_tunnel_rough_nav | succeeded 76.9 s, 19.8 m | pass | 23.3 ms / 68 | | |

B5: 17 of 20 succeed. Final position error 0.09 to 0.15 m on every success. No contact, no MPC holding or
fallback cycles except 25 holding cycles in the late-crossing run (the worker 0.5 m ahead).

Run ids: `20261017T1800..T1930Z-<scenario>-wholebody-50hz` (the newest `<scenario>-wholebody-50hz` of each scenario).

## B1 (RPP) and B3 (RPP missions)

| Scenario | Ctrl | Outcome | Expected | Class | Evidence |
|---|---|---|---|---|---|
| v2_open_yard_nav | B1 | succeeded 39.2 s | pass | | |
| v2_spine_nav | B1 | succeeded 127.4 s | pass | | |
| v2_door_130_nav | B1 | aborted, path 0 m, error 208 (no plan), 10 recoveries | fail to plan (carry 1.24 m in a 1.30 m door) | (e) | `plan-expectations.yaml`: carry false |
| v2_door_105_nav | B1 | aborted, path 0 m, error 208, 11 recoveries | fail to plan (carry wider than the 1.05 m door) | (e) | same |
| v2_through_building_mission | B3 | succeeded 247.5 s (reconfigure 6.6 s, whole loop) | pass | | MoveIt carry to vertical carry worked |
| v2_scaffold_tunnel_mission | B3 | succeeded 301.0 s (reconfigure to home in the tunnel) | the text expects B1 to B4 to stop before H1 | (e) not reproduced | step 2 drove 56 m: the global planner detoured round the lane-blocking H1 cells to reach y 9.5; no panel penetration |
| v2_door_105_mission | B3 | **failed at step 1** (navigate to 16.275, 3.8), error 104 patience exceeded, 11 recoveries | pass or door fail (the text expects only B4-static to fail) | (c) | see Failure 4 |

B3 runs carry no MPC statistics. Real-time factor 1.00 on all.

## Failures

1. **v2_room_r4_nav, B5 (a, with c).** Run `20261017T181529Z`. The base reaches the hallway H (y 11.65..13.65)
   and tries the 1.05 m door of R4 (x 7.825, north wall) from the side: at t 90 s it sits at (7.62, 13.18)
   with yaw 68 deg (0.20 m off the door axis, 22 deg off the 90 deg heading). `/cmd_vel_smoothed` (MPC output) keeps
   0.24 to 0.30 m/s and yaw rates up to 0.4 rad/s, `/cmd_vel` after the monitor goes to 0.00; the monitor
   alternates StopZone/SlowdownZone from 93 s, ends in FootprintApproach at 115.7 s and then publishes nothing
   (`stop_pub_timeout` 2 s) for the remaining 70 s while the MPC is `ok` (max slack 0, no hold) and commands
   0.29 m/s / 0.39 rad/s. The base never moves again. The hallway is 2.0 m wide, so the route must turn 90 deg into
   a 1.05 m door within about 1 m; the B5 stop zone (hull + 0.05 m, about 0.87 m wide) is violated by the oblique entry.
   Same mechanism as the old door-B stall; the interior doors have no straight approach lane (only the two
   south doors got one).
2. **v2_store_room_nav, B5 (a, with c).** Run `20261017T182014Z`. Door of store S (1.05 m, x 15.05, y 9.025) is
   entered from the 3.2 m wide east bay going north: the stall is at (15.50, 8.85), yaw 151 deg (0.18 m off the
   axis, 29 deg off the 180 deg heading), StopZone at 83.6 s. MPC `ok` and commanding 0.29 m/s and 0.39 rad/s for
   the next 70 s, `/cmd_vel` silent. Slack 0, so the MPC does not see itself blocked: its own margin is smaller
   than the monitor's stop zone.
3. **v2_worker_late_crossing_nav, B5 (c).** Run `20261017T190013Z`. The robot stops at (4.52, -0.04), yaw -19 deg
   (the nose at the walkway W2 edge) at 21.6 s with the worker 0.5 m ahead in the StopZone and 25 MPC holding cycles
   (slack up to 0.21 m). The mover reports `walked 2.48 m, finished false, blockedSeconds 91.0` at (5.275, 0.52): the
   scenario's worker pauses while the robot is in its path, the robot's StopZone waits while the worker is in
   its path. Neither moves until the 90 s timeout: a deadlock of the scenario design (the worker triggers when the
   robot is already at the crossing), not a controller fault. With `stop_pub_timeout` the monitor is silent after
   2 s, the MPC commands 0.29 m/s and -0.39 rad/s into it.
4. **v2_door_105_mission, B3 (c, with d-like behaviour).** Run `20261017T192427Z`. Step 1 navigates to
   (16.275, 3.8) in carry (1.24 m wide) but that point now lies inside door B's new 1.6 m approach lane (barriers from
   y 3.0). RPP arrives at the lane mouth (16.11, 2.37) at yaw 51 deg, `/cmd_vel_nav` falls to 0 after 20 s (6
   messages in 4 s) and the controller ends with 104 (patience exceeded); the monitor only slowed (no stop). The mission
   step was written before the lanes existed; it should wait on the spine (y at most 2.0) and reconfigure there.
   Not run by B5 (one navigate task, reconfigures itself) which passes.

## Free drive (B5, 50 Hz, no scenario, start at the V2 spawn)

`demo.py drive --controller wholebody --site construction_site_v2`, robot at (-21.0, 0.0, yaw 0) in carry, goals
published on `/goal_pose` (PoseStamped, frame map) like RViz; MPC warnings counted from `/tmp/mm_nav.log` ("MPC
cycle took") over the leg; no FollowPath failure or abort in the log for any leg.

| Goal | Reached | Time | Path | MPC "cycle took" warnings (median / p90 / max) |
|---|---|---|---|---|
| (-14.21, 17.58, 0): tunnel turnaround pocket (the user's failed goal) | yes, 0.15 m | 160.6 s | 35.0 m | 52 / 23.4 / 33.3 / 60.8 ms |
| (5.275, 10.4, 90 deg): north end of C1 through door A | yes, 0.14 m | 181.4 s | 38.8 m | 68 / 22.8 / 32.3 / 50.0 ms |
| (3.0, -8.5, -30 deg): delivery yard, back out of the building | yes, 0.15 m | 109.1 s | 19.7 m | 16 / 22.8 / 30.5 / 30.7 ms |

Warnings are one per overrun cycle; legs have 8000 to 9000 cycles, so over 99.4 per cent of cycles meet the period.

## Summary: what blocks usability

Nothing blocks the user's three drives: the spine, tunnel, door A, C1 and the yard all work at 50 Hz in real time.
The MPC runs at the full rate with a per-cycle p99 of 13 to 41 ms and deadline misses of 0.5 to 2 per cent. What remains:

1. Interior 1.05 m doors entered obliquely from a narrow corridor (R4 from hallway H, store S from the east
   bay) stall the base at the collision monitor's StopZone while the MPC keeps commanding. This is the user's earlier
   failure, now at the two interior doors that have no approach lane. Options (not done here): straight lanes for
   the interior doors or a wider stop zone margin discussion for B5.
2. After a stop the collision monitor goes silent after `stop_pub_timeout` and nothing recovers the base: the MPC
   still reports `ok`; Nav2 recoveries do not move the base. Free drive has no timeout, so an RViz user would see a
   frozen robot with an `ok` MPC.
3. `v2_worker_late_crossing_nav` deadlocks by design (mover waits for the robot); retime its trigger.
4. `v2_door_105_mission` step 1 targets a point inside the new lane (B3 only); move it onto the spine.
5. `demo.py up` accepts a lone stale `map_to_odom` publisher as the running description stack
   (`DESCRIPTION` matches `static_transform_publisher.*map_to_odom`); a dead stack gives "invalid source" stops.
