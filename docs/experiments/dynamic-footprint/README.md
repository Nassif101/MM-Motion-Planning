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
