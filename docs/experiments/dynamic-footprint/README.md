# Dynamic convex-hull footprint (Phase 3, baseline B4)

Configuration-dependent Nav2 footprint after Sagar, Long and Garcia Santiago (CoDIT 2026,
DOI 10.1109/CoDIT70676.2026.11630999): the ground projection of base, arm and the attached
panel, hulled and padded, published as the Nav2 footprint. Design:
[spec](../../superpowers/specs/2026-10-04-phase3-dynamic-footprint-design.md).

## Nav2 prerequisites (2026-10-04)

Nav2 behaviours the design relies on, checked in the installed version's source (Jazzy
1.3.13, `ros-navigation/navigation2` tag `1.3.13`) and live in Unity.

| # | Behaviour | Source (1.3.13) | Live check | Holds |
|---|---|---|---|---|
| 1 | Smac Lattice uses the current footprint on every plan | `nav2_smac_planner/src/smac_planner_lattice.cpp:304`, `createPlan` calls `_collision_checker.setFootprint(_costmap_ros->getRobotFootprint(), ...)` | see below | yes |
| 2 | The inflation layer re-inflates on a footprint change | `nav2_costmap_2d/plugins/inflation_layer.cpp:169-175`, `onFootprintChanged` recomputes the inscribed radius and caches and sets `need_reinflation_` | see below | yes |
| 3 | The collision monitor accepts any polygon on `polygon_sub_topic` | `nav2_collision_monitor/src/polygon.cpp:531-560`, `updatePolygon` only rejects fewer than 3 points and transforms every vertex | (Task 7) | yes |
| 4 | Nav2's `footprint_padding` still contains a many-vertex hull | `nav2_costmap_2d/src/footprint.cpp:145-153`, `padFootprint` adds `sign0(x) * padding`, `sign0(y) * padding` per vertex | numeric, see below | yes, with a smaller margin |

Live check of 1 and 2 (robot at the 1.30 m gate start (0, -16), `global_planning.launch.py
footprint_profile:=vertical_carry`, Smac Lattice queries from (0, -16) to (-4.8, -16)):

| Footprint on `/global_costmap/footprint` | `published_footprint` | Lattice route |
|---|---|---|
| `vertical_carry` (launch) | rectangle | 4.80 m, through the gate |
| hexagon 1.5 m wide | the padded hexagon, within 2 s | 11.50 m, detour around the gate |
| `vertical_carry` republished | rectangle | 4.80 m, through the gate |

Check 4: the URDF mesh hull (lidar-filter geometry, 64-gon cylinders) of the three qualified
poses and of 21 joint-space samples on each of home -> vertical_carry and
home -> level_extension, padded with Nav2's rule (0.01 m), contains the unpadded hull in all
43 poses and stays convex. The margin is not the padding, though: 0.010 m for `home`
(4 vertices), 0.0055 m for `vertical_carry` (72 vertices), 0.0046 m for `level_extension`,
and as little as 0.0002 m on an intermediate pose, because edges between vertices near an
axis barely move. B4's own clearance therefore comes from its exact 0.02 m outward offset;
Nav2's padding adds at most 0.01 m on top.

## B4 regression on the B3 missions (2026-10-04, `4868c4d`)

The three B3 missions with `--footprint-mode dynamic` (mesh model), RPP, 3 runs each, in one
Play epoch. Missions are unchanged (same steps and tolerances as B3);
`dynamic_footprint_node` owned both costmap footprints and the collision monitor zones
(ownership check passed before every run), and `ReconfigurePanel` checked the planned and
measured padded hull against the global costmap instead of switching profiles. Summaries:
[runs/](runs/) (`*-rpp-dyn.json`). B3 RPP for comparison: the 2026-10-04 rerun at `849f19b`
([moveit-arm](../moveit-arm/README.md)).

| Mission | B4 success | B3 RPP | Contact | Total s (B4) | Total s (B3) | Hull refusals | Hull clearance at the gate pose m | Footprint publishes in drives | Node compute µs mean/max |
|---|---|---|---|---|---|---|---|---|---|
| `narrow_gate_mission` | 2/3 | 3/3 | 0 | 57.1 (56.7-57.4) | 63.0 (59.5-63.1) | 0 | 0.042-0.046 (restore home) | 0 | 54/776 |
| `wide_gate_mission` | 3/3 | 3/3 | 0 | 56.4 (56.3-57.5) | 57.2 (57.0-59.5) | 0 | 0.175-0.185 (restore home) | 1 | 57/1606 |
| `constrained_reconfiguration_mission` | 3/3 | 3/3 | 0 | 27.4 (26.8-29.5) | 28.4 (26.6-29.7) | 0 | 0.215-0.265 (beside the tall box) | 0 | 61/1211 |

- **8/9 missions, no contact.** The failure is the third narrow-gate run: RPP aborted the
  1.05 m throat drive after 11 recoveries with 0.14 m static clearance and no contact, the
  documented throat behaviour of every baseline (Phase 1 8/9, B3 7/9 throat drives; roadmap
  open decision 2026-10-03, kept as the stress test). In vertical carry the hull is the
  `vertical_carry` profile's width (y +/-0.385 m padded), so B4 does not change the throat.
  With 3 runs per mission B4 and B3 are not distinguishable.
- **All arm checks pass** in the successful runs (panel bottom >= 0.56 m, tilt <= 0.13 deg,
  JTC path error <= 0.112 rad); no `HULL_IN_COLLISION`; MoveIt planning 0.00-0.44 s, never
  re-planned.
- **The footprint follows the arm at standstill.** The node published 36-145 footprints per
  mission, all but one while the arm moved during a reconfiguration (6 vertices in `home`,
  up to 29 mid-motion). One publish fell 0.96 s into a drive (wide gate, run 3): the panel
  deflecting as the base accelerated moved the hull by more than the 0.01 m threshold. Nav2
  took it like any footprint change; 1 of 9 missions.
- **Hull clearance** is the padded hull's distance to the nearest lethal global-costmap cell at
  the reconfiguration pose (1.0 m means none within the 1.0 m search radius). Restoring
  `home` right after the narrow gate leaves 0.04 m: the 1.24 m panel next to the gate's
  approach walls is the tightest reconfiguration the missions contain, and the check passed.
- **Update cost** in the live node: 49-61 µs mean per 20 Hz tick, at most 1.6 ms (Unity running
  in parallel), well inside the 50 ms period; the paper reports 689-857 µs mean (Table II).
- Mission times are within the B3 spread or slightly shorter; the wait for the costmaps
  after a reconfiguration is the same (median 0.80 s B4, 0.78 s B3, one full update cycle on
  the new footprint), and a reconfiguration step takes 5.7 s median against 6.2 s. The
  difference is not attributed to the footprint mode with 3 runs per mission.

## Footprint strategies, paper Table III analogue (2026-10-04, `c602101`; the base-only drives ran with the runner's epoch recovery, committed in the next commit)

The paper compares a static base-only footprint, a static enlarged footprint and the dynamic
footprint with the arm held tucked or extended. Here the arm is held in `home` (the 1.24 m
panel overhangs the base sideways, the paper's "extended") or `vertical_carry` (panel within
the wheel track, close to "tucked"), and the routes are the 1.05 m and 1.30 m gate scenarios
(posts 2.4 m tall). Strategies:

- **base-only:** `--footprint-profile base_only` (base box, wheels and pedestal plus 0.02 m:
  x +/-0.46, y +/-0.385 m; the panel is not in the footprint).
- **enlarged:** `--footprint-profile home`, the envelope of both transport poses (an envelope
  that includes `level_extension` would exceed the 1.0 m inflation radius every baseline uses;
  spec Section 6).
- **dynamic:** `--footprint-mode dynamic` with the mesh model, and the disc model for the
  plan-only rows.

Geometric truth: `home` does not fit the 1.05 m gate (1.24 m panel) and leaves 0.03 m per side
at the 1.30 m gate (below the 0.02 m padding plus Nav2's 0.01 m and the cell quantization,
so no strategy should route it through); `vertical_carry` fits both gates.

### Plan-only (Smac Lattice, the navigation planner; NavFn in parentheses), route length m

| Arm pose, gate | base-only | enlarged (`home`) | dynamic mesh | dynamic disc | Should route through |
|---|---|---|---|---|---|
| `home`, 1.05 m | **4.80 through** (4.97) | 11.13 detour (10.99) | 11.17 detour (5.13 through) | 11.13 detour (10.99) | no |
| `vertical_carry`, 1.05 m | 4.80 through (4.97) | **11.13 detour** (10.99) | 4.80 through (4.97) | 4.80 through (4.97) | yes |
| `home`, 1.30 m | **4.80 through** (4.97) | 11.50 detour (5.00 through) | 11.50 detour (5.00 through) | 11.50 detour (5.00 through) | no |
| `vertical_carry`, 1.30 m | 4.80 through (4.97) | **11.50 detour** (5.00) | 4.80 through (4.97) | 4.80 through (4.97) | yes |

Bold: a wrong decision. With Smac Lattice the pattern is the paper's: base-only plans the
extended panel through both gates (unsafe), enlarged rejects both passages the robot fits in
vertical carry (false infeasibility, an 11 m detour instead of 4.8 m), dynamic decides all
four correctly with either model. The disc model's extra conservatism never changes a route:
at the qualified poses the panel and the wheels set the hull and the disc and mesh hulls have
the same area (Task 3 benchmark).

NavFn (GridBased) checks only a circle of the footprint's inscribed radius against the
costmap, the limitation the paper names for holonomic planners. It routes `home` through the
1.30 m gate with every footprint, and through the 1.05 m gate with the dynamic mesh hull: the
`home` profile's inscribed radius (0.55 m padded) blocks the gate centre cell by about 5 mm,
and the real hull's front edge is a few millimetres closer (gate-centre cost 97 instead of 99,
inscribed). Navigation uses Smac Lattice, which checks the full polygon in SE(2).

### Drives (RPP, 5 runs per cell, 60 drives)

Success / runs, dominant failure mode, median time s and path m of successful drives, minimum
clearance of the real robot (scenario profile) to the static map m. A drive with Unity contact
is canceled at the first contact and counts as a collision. Summaries: [runs/](runs/)
(`*-rpp-dyn.json`, `*-rpp-fphome.json`, `*-rpp-fpbase_only.json`).

| Arm pose, gate | base-only | enlarged (`home`) | dynamic (mesh) |
|---|---|---|---|
| `home`, 1.05 m | **0/5, collision 5** (panel against the gate post, 0.0017-0.0022 m penetration, canceled after 13-16 s) | 5/5, detour 45.7 s, 11.03 m | 5/5, detour 45.0 s, 11.02 m |
| `vertical_carry`, 1.05 m | 2/5, throat abort 3, 25.7 s, 4.66 m | 5/5, **detour** 47.4 s, 11.02 m | 2/5, throat abort 3, 26.4 s, 4.65 m |
| `home`, 1.30 m | 5/5 through, 25.4 s, 4.65 m, clearance 0.05 m | 5/5, detour 46.1 s, 11.26 m | 5/5, detour 45.5 s, 11.30 m |
| `vertical_carry`, 1.30 m | 5/5 through, 25.4 s, 4.65 m | 5/5, **detour** 45.4 s, 11.30 m | 5/5 through, 25.0 s, 4.65 m |
| **Total** | **12/20**, 5 collisions | **20/20**, 2 cells with an 11 m detour instead of 4.65 m | **17/20**, 0 collisions |

Against the paper's Table III (base-only 20-30 %, enlarged 40-50 %, dynamic 90 %):

- **Base-only is unsafe:** every `home` drive at the 1.05 m gate put the panel into the gate
  post (5/5 collisions, all stopped at a graze by the runner's contact cancel). It also drives
  `home` through the 1.30 m gate with 0.03 m per side, which passed 5/5 with 0.05 m measured
  clearance (the real panel stays inside the profile's 0.02 m allowance): no margin, and no
  stop-zone protection, because the collision monitor zones are sized from the base too.
- **Enlarged is over-conservative but never fails here:** it rejects both passages vertical
  carry fits and detours 11 m (about 45 s instead of 25 s). The paper counts blocked paths as
  failures; our map always has a detour, so over-conservatism shows as time and distance.
- **Dynamic decides every cell like the geometry**, with no collision. Its 3 failures are the
  1.05 m throat aborts in vertical carry, the same rate as base-only in the same cell (2/5
  each; Phase 1 static `vertical_carry`, RPP: 2/3): the effective opening, not the footprint
  (roadmap open decision 2026-10-03). This matches the paper's dynamic failure mode, "goal or
  controller infeasibility".
- **`home` at the 1.30 m gate:** dynamic and enlarged both detour although base-only showed that
  the panel physically passes with 0.03 m per side. The dynamic hull's 0.02 m padding plus Nav2's
  0.01 m leave no free cell, so B4 rejects a passage that is geometrically just feasible; this is
  the margin by design, not a hull error.
- **The dynamic footprint changes while driving in `home`:** 35 and 40 publishes over the 5
  drives at the two gates (vertical carry: 0). The cantilevered horizontal panel sways
  `wrist_1` by up to 18.9 mrad under base acceleration and turning (vertical carry at most
  3.2 mrad), which moves the hull by more than the 0.01 m threshold several times per drive;
  each publish re-inflates both costmaps. No drive failed from it. Options for later: a larger
  or asymmetric threshold (grow at once, shrink only at rest), or footprints from the arm
  controller's reference instead of the measured joints.
- Node compute time during the drives: 55-58 µs mean, at most 6.2 ms (one tick during a home
  drive with Unity and Nav2 loading the container).

Run conditions: Unity Play, Nav2 RPP, `global_obstacles:=persistent`, one Play epoch per
strategy batch; the base-only batch needed new epochs after contacts (a drive canceled at a
contact leaves the arm deflected against the post, and MoveIt cannot plan from a colliding
state; the runner now starts a new epoch then). No run had host sleep or container OOM kills.

## Update cost and disc vs mesh, paper Table II analogue (2026-10-04)

`footprint_benchmark --evaluations 5000` (container, Unity stopped; [benchmark.json](benchmark.json)):
time of one `footprint(joints, 0.02)` (forward kinematics, projection, hull, exact offset)
per qualified pose, and the hull it gives.

| Model | Pose | Mean µs | Min µs | p99 µs | Max µs | Points | Hull vertices | Area m² |
|---|---|---|---|---|---|---|---|---|
| mesh | `home` | 10.9 | 10.1 | 17.7 | 119.7 | 320 | 4 | 1.54 |
| mesh | `vertical_carry` | 11.3 | 10.6 | 15.6 | 107.7 | 320 | 24 | 0.84 |
| mesh | `level_extension` | 10.8 | 10.2 | 15.9 | 53.8 | 320 | 6 | 2.00 |
| disc | `home` | 13.5 | 12.8 | 17.8 | 36.7 | 687 | 4 | 1.54 |
| disc | `vertical_carry` | 14.5 | 13.8 | 19.6 | 44.5 | 687 | 24 | 0.84 |
| disc | `level_extension` | 14.3 | 13.6 | 20.7 | 47.1 | 687 | 6 | 2.00 |

- The live node (20 Hz tick: hull, Hausdorff change test, zone offsets, statistics) took
  43-61 µs mean per tick in the regression and strategy runs, at most 6.2 ms with Unity and Nav2
  loading the container. The paper reports 689-857 µs mean and up to 2.7 ms (Table II, TF
  lookups per link, 1003-1195 points); here the projection uses the library's own forward
  kinematics, not TF.
- **Disc vs mesh:** the disc hull's area equals the mesh hull's on all three poses (ratio
  1.000), and it encloses the mesh points over the 45-pose sweep (`DiscModel.ContainsMeshOverSweep`).
  The 1.2 x 1.2 m panel and the wheels set this robot's hull; the arm's discs fall inside it.
  The paper's disc-model conservatism (its Fig. 6) does not appear with a payload this large, and
  the disc model changed no route (plan-only rows above). It costs 25-30 % more time than the mesh
  model with twice the points.

## B4 rerun with the grow-only footprint (2026-10-11, `1487df8`)

The 2026-10-04 drives republished the footprint 7-8 times per `home` drive because the
cantilevered panel sways under base acceleration. `1487df8` changes the node: while `/odom`
reports motion (0.01 m/s or 0.02 rad/s, held 0.5 s) it republishes only when the new padded
hull leaves the current hull grown by the 0.01 m threshold, and then as the hull of the union;
at standstill the Hausdorff rule applies, so the footprint shrinks after 0.5 s of rest. The
expectation recorded in the Phase 4 plan was at most 1 publish per `home` drive. Same setup as
the 2026-10-04 rows (Unity Play, Nav2 RPP, `global_obstacles:=persistent`, mesh model, clean
tree at `1487df8`, no host sleep or container OOM kills). The summaries stay local
(`experiment_runs/`, git-ignored); the numbers come from `tools/summarize_nav_runs.py` and the
`footprint_stats` of each run. A first start of `narrow_gate_mission` (`155212Z`) aborted
before producing data (empty run directory, infrastructure); it is not a result and was
repeated.

### B4 regression on the B3 missions (RPP, 3 runs each)

| Mission | B4 success (10-11) | B4 success (10-04) | Contact | Total s (10-11) | Total s (10-04) | Hull refusals | Publishes in drives (10-11 / 10-04) | Publishes per mission | Node compute µs mean/max |
|---|---|---|---|---|---|---|---|---|---|
| `narrow_gate_mission` | 3/3 | 2/3 | 0 | 58.0 (56.9-59.0) | 57.1 (56.7-57.4) | 0 | 0 / 0 | 97-132 (36-80 on 10-04) | 78/2029 |
| `wide_gate_mission` | 3/3 | 3/3 | 0 | 57.2 (56.8-57.4) | 56.4 (56.3-57.5) | 0 | 0 / 1 | 100-107 (72-102) | 77/15448 |
| `constrained_reconfiguration_mission` | 3/3 | 3/3 | 0 | 29.5 (27.2-30.0) | 27.4 (26.8-29.5) | 0 | 0 / 0 | 113-147 (90-145) | 78/1876 |

- **9/9 missions, no contact, no `HULL_IN_COLLISION`**, all arm checks passed (3/3 per
  mission; panel bottom 0.56-0.70 m, tilt at most 0.15 deg, JTC path error at most 0.075 rad),
  no re-planning. The 10-04 failure (a throat abort in the third narrow-gate run) did not recur;
  with 3 runs per mission that is the known throat variance, not an effect of the footprint.
- **No footprint publish fell inside a mission drive** (10-04: one, wide gate run 3). All
  publishes come from the arm moving at standstill, where the rule is unchanged, so the totals
  per mission measure arm pacing and are not a result of this change. They are higher than on
  10-04 (97-147 against 36-145); I did not isolate why (run-to-run arm timing is the obvious
  candidate and the standstill code path is identical).
- Compute per tick is 77-78 µs mean against 43-61 µs on 10-04 (the added containment test is a
  few tens of µs on the 28-32 vertex hulls of a reconfiguration), at most 2.0 ms, and one
  15.4 ms tick in a wide-gate run with Unity loading the container; the period is 50 ms.

### Footprint strategies, dynamic column (RPP, 5 runs per cell, 20 drives)

| Arm pose, gate | Success (10-11) | Success (10-04) | Median time s / path m (10-11) | Publishes in drives per drive (10-11) | Publishes in drives per drive (10-04) |
|---|---|---|---|---|---|
| `home`, 1.05 m | 4/5, detour; 1 abort (153 s, 11 recoveries, "Failed to make progress", 0.13 m from the goal) | 5/5 | 45.5 s, 11.01 m | 3, 2, 2, 2, 3 (**12**) | 4, 10, 8, 7, 6 (**35**) |
| `vertical_carry`, 1.05 m | 3/5, throat abort 2 | 2/5, throat abort 3 | 25.9 s, 4.65 m | 0, 0, 0, 2, 0 (**2**) | 0 (0) |
| `home`, 1.30 m | 5/5, detour | 5/5 | 45.9 s, 11.30 m | 2, 3, 2, 2, 2 (**11**) | 10, 4, 6, 10, 10 (**40**) |
| `vertical_carry`, 1.30 m | 5/5 through | 5/5 | 25.0 s, 4.65 m | 0 (0) | 0 (0) |
| **Total** | **17/20**, 0 collisions | **17/20**, 0 collisions | | **25** | **75** |

Publishes in drives are those between the drive's start and its end, which reproduces the 35
and 40 of the 10-04 table. Node compute time during the drives: 62-93 µs mean per cell run, at
most 5.9 ms (10-04: 55-58 µs, 6.2 ms). Polygons keep their vertex counts (6 in `home`, 15-37 in
`vertical_carry`).

- **The expectation of at most 1 publish per `home` drive is not met.** The grow-only rule cut
  publishes in `home` drives from 7.0 and 8.0 per drive to 2.4 and 2.2 (-66 % and -72 %) and the
  worst drive from 10 to 3, but every `home` drive still republished 2 or 3 times. The vertical
  carry cells are unchanged except for one aborted throat drive (2 publishes while the robot
  was in recoveries).
- **Where the remaining publishes fall.** All 10 `home` drives published once within 1.1 s of the
  start and 7 of 10 once more by 3.1 s, the base's first yaw and acceleration (the trajectory
  starts to turn at about 1 s); the rest are one publish near the end of the drive (38-47 s)
  where the base slows and stops. Early ones are grows (the sway extends the hull past hull plus
  threshold) or the standstill rule firing just before the odometry thresholds are crossed; the
  node stores only publish times, not the reason, so I cannot say which. A publish at stop is
  the designed shrink. To reach at most 1 the likely options are a larger grow margin during
  the first second of motion or logging the reason per publish first.
- **No drive failed or touched anything because of it.** 17/20 as on 10-04 with 0 contacts, and
  the failures are the known ones: the 1.05 m throat in vertical carry (2/5 here, 3/5 on 10-04,
  the same as base-only in that cell) and one `home` narrow-gate detour that aborted at the goal
  with "Failed to make progress" 0.13 m away. That last one differs from 10-04 (5/5) but its
  footprint published once in the 100 s before the abort and it stopped at the goal's
  neighbourhood; with RPP's known problem starting in-place turns I attribute it to the
  controller, not the footprint, but the data do not prove that with one run.
- **What fewer publishes buys:** each publish still re-inflates both costmaps, so the gain is
  fewer re-inflations during a drive, not a different routing decision (the plan-only rows
  above are unaffected, the robot is at standstill there).
