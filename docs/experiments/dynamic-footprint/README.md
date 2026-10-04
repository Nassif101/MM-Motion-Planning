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
