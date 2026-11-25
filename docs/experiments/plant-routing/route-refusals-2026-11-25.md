# Route refusals by the swept validation (check, 2026-11-25)

The timeboxed check the user placed right after the medium-diagnosis merge (roadmap decision log, 2026-11-25):
reproduce one refusal, find the failing pose and why; fix it if small, else a spec 2 follow-up for the user to slot.

## The symptom

In the medium diagnosis's door runs (`experiment_runs/medium-diag/door`, `docs/experiments/medium-diagnosis/
corpus_door.json`; realistic plant, lattice routes), 9 of 60 runs had a route refused: "rejected: the smoothed and the
raw plan failed the swept validation".

- 8 refusals of the route from `v2_room_r4_nav`'s start on the spine (5.275, -0.5, 1.5708) to a hallway point: to
  (3.5, 12.65, 0) in seeds 5, 11, 16, 17, 18 and to (11.5, 12.65, -pi) in seeds 4, 13, 14, at 0.94-1.1 s;
- 1 refusal of the route from (3.37, 12.73) to R4's goal (seed 20 of the west approach, at 69.2 s).

The seeds differ only in the start's perturbation (under 1 mm and 1 mrad); other seeds' routes to the same points are
accepted.

## Reproduction

`TEST(V2Routes, DISABLED_HallwaySeedsDiagnostic)` (`ros2_ws/src/mobile_manipulator_campaign/test/test_v2_routes.cpp`,
run with `--gtest_also_run_disabled_tests --gtest_filter='*Hallway*'`) plans the same routes with the plant's routing
inputs from the exact start and from the seeds' perturbed starts. It reproduces the refusals deterministically (west:
seed 11 refused, seed 1 and the exact start accepted; east: seed 4 refused, seed 1 accepted) and prints each blocked
segment: its poses, whether each pose alone is clear, the nearest occupied cell, and the gap between the footprint and
the occupied cells' squares over the swept motion (samples every 0.5 mm of vertex travel). The diagnostic of spec 2
Task 13 (`DISABLED_StoreRoomSeed4Diagnostic`) shares the code.

## What fails, and why

Every blocked segment is a turn in place, or a short step out of one, beside the hallway's south partition wall
(y 11.55, its cells at y 11.625), and each pose of it is clear on its own:

| Route | Blocked segment | Gap of the padded footprint over the swept motion |
|---|---|---|
| west, seed 11 | turn in place at (2.274, 12.251), yaw 2.46 -> 2.36 | +1.1 mm |
| west, seed 11 | the next turn, yaw 2.36 -> 2.20 | **-0.9 mm (overlaps a wall cell)** |
| west, seed 11 | the next turn, yaw 2.20 -> 2.03 | +0.1 mm |
| east, seed 4 (raw plan) | turn in place at (12.676, 12.250), yaw 0.62 -> 0.79, then a 7 cm step | +0.5 mm, +0.6 mm |
| east, seed 4 (smoothed plan) | a 7 cm step with a heading change, near the north wall | +12.7 mm |

The footprint here is the validation's: the projected base hull with the geometric padding (`dynamic_footprint.yaml`
`padding_m`, 0.02 m), so the robot's own hull stays 19-33 mm from the wall in every case.

**Cause.** Smac (the Lattice port) checks its footprint, with the costmap's padding added, at the lattice poses only.
In a turn in place the footprint's far vertices move by about the footprint radius times the heading step between two
checked poses (several centimetres), and Smac does not look at the swept space between them. The swept validation
does: it tests the continuous motion with a conservative envelope (its interpolation bound plus the closed-contact
test), so a turn that grazes the padded footprint against a wall cell is refused (correctly once here: -0.9 mm; within
the envelope in the others). The refused route is then dropped: the search ends `Rejected`, and the goal fails, with no
second attempt. Which lattice poses Smac picks in the 2 m hallway depends on the start's sub-millimetre perturbation,
hence the seed-to-seed pattern. The same mechanism produced spec 2 Task 13's store-room refusal, which Amendment 3
treated by checking the hull with the geometric padding only (`routing_inputs.cpp`).

## Not a small fix: a spec 2 follow-up

Any repair changes how the route layer handles a plan Smac finds and the validation refuses (spec 2 Amendment 1 / ADR
0019: an exact Smac port, with our wrapper and a swept validation pass on the returned path). Candidates, none built:

1. **Re-plan around the refused segment** (in our wrapper): mark the cells of the blocked sweep and search again, a
   bounded number of times. Keeps Smac exact and the validation strict; costs a second search (about 200k iterations
   here: about 50 ticks of 4,096 iterations, about 1 s of planning hold).
2. **Repair the refused turn** in our wrapper: replace a blocked turn in place with the same turn a few centimetres
   further from the wall, along the route, and validate again.
3. **Change what Smac checks** (a larger costmap padding, or denser rotation primitives): leaves the exact port or
   costs passage in narrow places; not recommended.

Until then a refusal fails the goal; the user's viewer sessions and any lattice-route test see it in about one run in
seven on these hallway goals.
