## Investigation of the differences (realistic mode)

Revised 2026-11-07 after the Task 11 review showed the first version wrong in substance (store_room, room_r4,
gate_right_swing); CP4's ruling is re-opened. "Unity's stack" is the Nav2 layer Unity's runs have around B5 and F2
(route replans, recoveries, the task's abort, the collision monitor); the plant has none of it.

**Versions.** Unity's F2 runs are pooled from campaign A (503cab7) and campaign B (0b6ba0a), which differ in F2: B
plans a new goal at B5's first cycle without hysteresis back to a slowing plan (7dd01a0) and labels a swing side only
from a real swing (f1853c4). The plant runs the newer code. Cells with runs from both: door_105, gate_right_swing,
room_r4, store_room.

**Outcomes**

- **v2_store_room_nav, B5 0/20 (Unity: 4 of its 5 B5 runs that day reached, 2/3 of them in campaign A) and F2
  5/20 (Unity 4/6 in the campaigns).** A plant effect of the wheel-contact base, by its turning sideslip:
  - The replicas follow the route of Unity run 162204Z, which failed on Unity in the same way: at the door entry
    (x 15.32) B5's plan asked 5.08 mm of slack on a pair outside its margin, B5 held for 474 cycles; it entered
    inward and less turned (y 8.929, yaw 2.817) than Unity's successful runs (y 8.96-9.00, yaw 2.88-2.99).
  - On that route the wheeled plant trips the same way in every run (B5 0/20). Its gate logged per cycle (3 seeds):
    the plan asks 5.1-5.4 mm of slack on a pair outside the margin (the allowance is 5 mm), B5 holds; the hold
    releases for 0.3-0.5 s and the robot creeps on, then re-trips just inside the margin (violation 0.1-1.7 mm, the
    plan asking 2-3 mm), where the allowance is the violation + 0.1 mm: from then on no plan passes and the hold is
    permanent. The planar and ideal bases never trip there (gate slack 0.01 mm).
  - On the routes of Unity's three successful B5 runs (172203Z, 183322Z, 102446Z) the wheeled plant fails too, 0 of
    15 (5 seeds each): it reaches the door entry (x 15.46) 2-3 cm inward (y 8.93) and already 17-31 mm inside a door
    pair's margin, then holds (10) or stalls in `ok` without progress (5). So the difference from Unity is not the
    route.
  - The common cause: inward sideslip in the turn into the door. At x 15.3 the wheeled base is 17-20 mm cross-track
    toward the inner door post (about 55 mm of inward body-frame sideslip built up in the turn; planar about 1 mm,
    ideal none) and 50-60 mrad less turned. The lateral offset tracks the outcome across grip: at mu 0.8 it halves and
    every run passes the entry (the failures move inside the room, x 14.4-14.6); at mu 0.4 it grows and the trip comes
    earlier. B5 does not model sideslip, so its plans start from a base that has drifted into the margin.
  - The planar ablation (B5 20/20) changes more than the base contact: profile_unity.yaml also has Unity's arm servo
    gains, base lags and skid map; the arm state at the door matches within about 5 mrad, so the arm is an unlikely
    cause, but not excluded. The arm command delay is excluded (0/20 without it).
  - F2's store-room failures (15/20) stop inside the room (x 14.6-14.9), not at the entry: not explained here.
  Findings: (1) the wheel-contact base sideslips into tight doors where Unity's base mostly does not (plant model:
  how much sideslip is realistic is the accepted effective-track deviation's open question); (2) B5 ignores sideslip
  (execution fixes: tracking-aware B5); (3) the slack gate's allowance steps from 5 mm outside the margin to 0.1 mm
  inside it, so a hold at the margin locks (execution fixes).
- **v2_gate_right_swing_nav, F2 17/20 against B5 20/20 (Unity F2 4/4, B5 3/3): the one realistic cell where the
  ranking does not hold.** The failures follow F2's first side: runs that start left fail 3 of 4, runs that start
  right pass 16 of 16 (ideal: left 1 of 8 fails, right 12 of 12 pass); the plant's "right" runs never swing
  (|pan| <= 0.53 rad). Unity by shoulder pan: three F2 runs swung left (+0.97 to +1.02 rad; one of them campaign B's,
  the plant's F2 code) and passed, one never swung; none switched (campaign A's side labels are the ones f1853c4
  fixed). The replicas follow B5 run 163100Z's route. None of these differences is significant at these counts (Fisher:
  left starts failing plant 3/4 against Unity 0/3, p 0.11; left preference 12/40 against 3/4, p 0.08; realistic-left
  3/4 against ideal-left 1/8, p 0.07). Two failures end in
  `ok` with zero commands (s4, s19: just past the posts, turned right), one in `holding` with 685 executed slack plans
  (s7); the ideal failure (s14) ends `holding` a little further on. Open: why the plant's F2 prefers the right side
  (the F2 version, the base, or the route), and why a left start fails here when it did not on Unity.
- **v2_room_r4_nav, B5 20/20 (Unity 1/3).** Unity's two failures (160919Z, the corpus route, and 182207Z) are B5
  stalls: `ok` throughout with sub-breakaway commands (|omega| <= 0.07) at (7.85-7.89, 13.20-13.26) for about 100 s,
  ten route replans and nine recoveries not freeing it, then Nav2's abort at 166 s. The collision monitor did not cut
  the command (/cmd_vel equals /cmd_vel_smoothed within 0.02 through its events): B5 slowed by itself. Before the
  stall Unity's base delivered 0.12-0.17 m/s for 0.25 commanded and turned at 0.53-0.65 rad/s for 0.40; the plant's
  wheeled base tracks the turn and B5 passes on the same route. Cause: Unity's base response (under-speed, over-turn)
  setting off a B5 stall the plant's base does not (a simulator difference; not confirmed by a plant run with
  Unity's response).
- **v2_worker_stops_in_lane_nav, F2 0/20 (Unity 1/1).** The worker stops in the lane; the robot waits 0.14 m from it
  until the timeout (one run brushes it). The plant follows the cell's first recorded route, which runs straight along
  y = 0 through x = -12.25 where the worker stops (about 17.5 s); Unity's Nav2 replanned at 19.1 s to y = -0.9. Cause:
  the route source (no replanning in the plant's loop; the plant's own route planner is spec 2).

**Clearances and contacts** (the medians of the true 3D minimum)

- **Follows the outcome or the stall point:** store_room B5 (the plant's holds stop before the tight part: 0.093-0.107
  m; the corpus run's failure on Unity 0.113 m), room_r4 B5 (the corpus route on Unity 0.107 m, the plant 0.101-0.105
  m; Unity's median mixes other routes), low_beam_123 B5 and low_beam_130 B5 (both fail on both; they stall at different
  distances from the beam: plant 0.030-0.042 / 0.041-0.093 m, Unity 0.017-0.033 / 0.011-0.042 m).
- **The worker cells: metres apart.** Unity's `clearance_3d_m` measures the static obstacles only; the plant's ground
  truth includes the movers. Unity's separate mover clearance (Nav2's 2D footprint to the worker: 0.334, 0.000 and
  0.025 m) is not the plant's 3D one (0.278, 0.108, 0.109 m); the two are not compared.
- **v2_spine_nav, v2_through_building_mission: plant "-".** Nothing within the ground truth's 2 m cut-off (Unity 2.4 m,
  3.0 m). The cut-off also biases medians near 2 m low (goal_near_skip, open_yard: runs beyond it are dropped).
- **Not attributed (one Unity run each, 13-39 mm):** chicane F2 (Unity 0.182, plant 0.093-0.172 m), goal_near_skip F2
  (1.402 / 1.412-1.417 m), services_deadend F2 (0.187 / 0.204-0.232 m), yard_store F2 (0.182 / 0.198-0.215 m): Unity's
  value lies outside the plant's whole 20-run range in all four, and Unity's spread is unknown. Left open.
- **Not attributed, systematic:** gate_right_swing B5 (plant 0.094-0.099 m in all 20 runs against Unity's
  0.130-0.140 m on the same route; ideal 0.072 m); low_beam_123 F2 (plant 0.029-0.100 m, median 0.040, below Unity's
  three F2 runs 0.059-0.089 m; the replicas follow a cancelled B5 run's route, 164814Z, 0.033 m on Unity, a candidate
  cause). Open.
- **v2_door_105_mission F2: 1 of 20 runs touches.** The front-left wheel brushes the door frame (0.4 mm deep, the goal
  still reached, after 139 s instead of 43 s). B5's own model saw the same (-0.39 mm against the plant's -0.41 mm): the
  run executed 862 plans with slack, up to the whole 0.12 m base margin, and followed infeasible medium plans for 3920
  cycles. Cause: a planner struggle at the door (executed slack), once in 20 here, not in Unity's one run.

**Ideal mode** (5 seeds; 20 for gate_right_swing and store_room): the ranking flips in door_then_beam (F2 4/5 against
B5 5/5), gate_right_swing (19/20 against 20/20) and store_room (18/20 against 20/20): without the realistic base, B5
alone passes the tight places every time and F2 occasionally does not.

**Status.** The ranking holds in every realistic cell but gate_right_swing (an F2 left-start failure; differences
from Unity not significant at these counts). Attributed: store_room to the wheel-contact base's sideslip into the door
(on every route; B5's lock makes it final), room_r4 to Unity's base response setting off a B5 stall, worker_stops_in_lane
to the route source, the worker and cut-off clearances to the metrics, store_room's and room_r4's B5 clearances to the
outcome or the route. Open: store_room's F2 failures inside the room, gate_right_swing's B5 clearance, low_beam_123's
F2 clearance, four single-run clearances. CP4 re-opened (user, 2026-11-07).
