# ADR 0010: Dynamic convex-hull footprint and footprint ownership (baseline B4)

Date: 2026-10-04. Status: accepted for Phase 3.

## Context

Phase 3 implements the configuration-dependent Nav2 footprint of Sagar, Long and Garcia
Santiago (CoDIT 2026, DOI 10.1109/CoDIT70676.2026.11630999): the ground projection of the
base and arm, for this thesis plus the attached panel, hulled, padded and published on Nav2's
footprint topics. B3 (ADR 0009) switches between two named footprint profiles at standstill;
the hull removes the profiles and works for any arm pose, which the later phases need (B4 as
a comparison row, the live demo with arbitrary goals, the geometry for the Phase 4 MPC). The
arm still moves only at standstill (as in the paper's simulation experiments). Design:
`docs/superpowers/specs/2026-10-04-phase3-dynamic-footprint-design.md`.

## Options considered

- Where the hull is computed: (1) a standalone node on a geometry library independent of ROS;
  (2) inside `ReconfigurePanel`, which already has the projection and the joint state;
  (3) a custom costmap layer that sets the footprint inside Nav2.
- Padding: the paper's centroid-radial vertex push (their Eq. 8), or an exact outward offset.
- Geometry: URDF collision primitives plus the panel (the paper's mesh variant), or the
  paper's disc model (link-origin discs and segment samples).
- What `ReconfigurePanel` checks without profiles: nothing (MoveIt is collision-free in 3D),
  or the planned and measured hull against Nav2's global costmap.

## Decision

- **Option 1.** `mobile_manipulator_geometry` (no ROS graph) holds the robot model, ground
  projection, convex hull, outward offset and both footprint models; `dynamic_footprint_node`
  publishes the padded hull at 20 Hz when it moves by more than 0.01 m. Option 2 would tie the
  footprint to the MoveIt server, which the demo and the later phases may not run; option 3
  contradicts roadmap decision 6 and the paper's "no Nav2 modification".
- **One footprint owner per run**, chosen with `footprint_mode`: `static` (no publisher; the
  launch profile stays), `profiles` (`ReconfigurePanel`, B3), `dynamic` (the node, B4). The
  owner publishes both costmap footprints and the collision monitor's stop and slowdown zone
  inputs (the footprint offset by the zone margins). `check_footprint_ownership` and the
  mission preflight refuse a run whose publishers do not match the mode.
- **Exact outward offset** (every edge moved out by 0.02 m, sharp corners cut by the tangent to
  the 0.02 m circle), not the centroid push, which gives a thin hull's long edges far less than
  the margin (for the 1.24 x 0.04 m panel outline 3 % of it).
- **Mesh model by default**; the disc model is an option (`footprint_model:=disc`). Its radii
  are capsule radii computed from the URDF, not hand-calibrated.
- **Hull check in `ReconfigurePanel`** (dynamic mode): the planned final state's padded hull
  at TF `map -> base_footprint` must cover no lethal cell of `/global_costmap/costmap`, and the
  measured state's after execution; otherwise `HULL_IN_COLLISION` (also without a costmap or
  the transform). A pose safe in 3D can still leave Nav2 unable to start a drive.
- The node keeps the last footprint on stale (> 0.5 s) or incomplete joint states and never
  shrinks it without data (the paper falls back to the bare base, which would hide the panel).

## Consequences

- B3 is unchanged (`footprint_mode:=profiles` for missions by default); B4 is
  `--footprint-mode dynamic` on any scenario, including plan-only ones.
- The footprint follows the arm during a reconfiguration (36-145 publishes per mission) and
  stays still while driving (1 publish inside a drive in 9 regression missions, from the panel
  deflecting under base acceleration).
- Nav2 behaviours this relies on were verified in the 1.3.13 source and live
  (`docs/experiments/dynamic-footprint`, "Nav2 prerequisites"). Nav2's sign-based
  `footprint_padding` leaves a coordinate of exactly 0 unpadded; `footprint_matches` now does
  the same.
- A hull whose circumscribed radius exceeds the 1.0 m inflation radius (possible for arbitrary
  arm poses, for example near `level_extension`) is flagged by the node: Smac's collision
  shortcut then misjudges.
- NavFn checks a circle of the inscribed radius only; a footprint a few millimetres smaller can
  open a gate the panel does not fit (seen at the 1.05 m gate). Navigation uses Smac Lattice.
- Test harness note: launch_testing mock tests can leave `control_description` running with a
  latched `/arm/robot_description`, which another mock test or a Unity arm control launch can
  pick up; stop it before such runs.

## Validation

- Geometry, node logic, ownership, hull-check and profile tests (gtest, pytest); mock-hardware
  tests for both modes (`test_reconfigure_mock.py`, `test_reconfigure_mock_dynamic.py`).
- Unity: B4 regression 8/9 on the B3 missions with RPP, no contact (the failure is the known
  1.05 m throat stop); the footprint-strategy comparison and the update cost are in
  `docs/experiments/dynamic-footprint`.

## Revisit when

- The arm must move while the base drives (Phases 4-5).
- Future arm poses should shape the footprint (footprint prediction, the paper's future work;
  roadmap Phase 5).
- The Phase 4 MPC needs continuous collision constraints from the same geometry.
