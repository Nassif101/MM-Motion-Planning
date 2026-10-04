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
