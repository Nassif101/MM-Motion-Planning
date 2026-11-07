## Investigation of the differences (realistic mode)

Each difference the agreement lists, with its cause. "The stack" is the Nav2 layer Unity's runs have around B5 and F2
(replans of the route, recoveries, the collision monitor, the task's abort) and the plant does not (spec Section 2: no
collision monitor in the plant; the ROS bridge, spec 3, runs the real Nav2 on the plant).

**Outcomes**

- **v2_gate_right_swing_nav, F2 17/20 against B5 20/20: the one cell where the ranking does not hold (Unity: F2 4/4,
  B5 3/3).** The three failures stop just past the gate posts (base x -0.6 to -0.7, the rear still between the posts),
  turned right (yaw -0.5 to -1.2) with the panel swung left (pan +0.8 rad): from about 27 s B5 reports `ok`, follows
  F2's plan (left homotopy) and commands zero for the rest of the 120 s, a local minimum in which every way out swings
  the rear corner or the panel into a post. The kinematic ideal mode reaches it too (1 of 20, same pose), so it is an
  F2/B5 property the realistic base meets more often (3 of 20), not a plant artefact; Unity's 4 F2 runs could not show
  a failure rate near 10 % (0 of 4 has probability 0.66 at 10 %). Cause: a planner deadlock after the gate; an
  execution-fixes item.
- **v2_store_room_nav, B5 0/20 (Unity 2/3) and F2 5/20 (Unity 4/6).** B5 reaches the store's 1.05 m door at about
  43 s and enters its slack-gate hold: the plans cut into the door's margins, a held plan is not executed, and the next
  cycle linearizes around it, so the hold persists (`holding` with zero commands until the 300 s timeout, no contact).
  Unity's runs hold here too (474 and 820 held cycles in its two failures), and Unity's stack sometimes frees them
  (Nav2's replans and recoveries move the approach; its failures end in Nav2 aborts after 10 and 14 recoveries). The
  plant has no stack, so a hold is final. The ground's grip changes how the gripping base arrives in the door: at
  contact_mu 0.8 F2 reaches 18/20 and B5 9/20 (0.4: 7/20, 0/20), and the kinematic ideal mode mostly passes (B5 20/20,
  F2 18/20 with 20 seeds).
  Cause: B5's hold deadlock in a tight door (a planner property, on both simulators), final on the plant for want of
  the stack, more frequent with the gripping base. An execution-fixes item (roadmap: after the plant).
- **v2_room_r4_nav, B5 20/20 (Unity 1/3).** Unity's two failures have no hold at all: the collision monitor's
  `approach` slowdowns trigger Nav2 recoveries (9 each) and the task aborts at 166 s. Cause: Unity's stack; the plant's
  B5 drives through in 60 s (Unity's success: 62.5 s).
- **v2_worker_stops_in_lane_nav, F2 0/20 (Unity 1/1).** The worker stops in the lane; the robot waits 0.14 m from it
  until the timeout (one run brushes it). The plant follows the cell's first recorded route, which runs through where
  the worker stops; on Unity, Nav2 replanned around it. Cause: the route source (recorded routes are not replanned in
  the plant's loop, as in the ROS-free bench; the plant's own route planner is spec 2).

**Clearances and contacts**

- **The worker cells (crossing, late crossing, stops in lane): metres apart.** Unity's `clearance_3d_m` measures the
  static obstacles only; the plant's ground truth includes the movers. Cause: the metric, not the runs. Unity's
  separate mover clearance (Nav2's 2D footprint to the worker: 0.334, 0.000 and 0.025 m in campaign B) is not the
  plant's 3D one (medians 0.278, 0.108 and 0.109 m) either; the two are not compared.
- **v2_spine_nav, v2_through_building_mission: plant clearance "-".** Nothing within the ground truth's 2 m distance
  cut-off (Unity: 2.4 m, 3.0 m). Cause: the cut-off.
- **v2_low_beam_123_nav F2: median 0.039 m against Unity's 0.082 m.** The panel under the beam on both; the plant's
  runs range 0.029-0.100 m and Unity's four 0.043-0.089 m. Cause: Unity's small sample within the plant's spread.
- **v2_door_105_mission F2: 1 of 20 runs touches.** The front-left wheel brushes the door frame (0.4 mm deep, the
  goal still reached, after 139 s instead of the usual 43 s). B5's own model saw the same (-0.39 mm against the
  plant's -0.41 mm): that run executed 862 plans with slack, up to the whole 0.12 m base margin, and followed
  infeasible medium plans for 3920 cycles. Cause: a planner struggle at the door (executed slack plans), seen once in
  20 here and not in Unity's one run; an execution-fixes item with the store room's holds.

**Ideal mode** (5 seeds; 20 for gate_right_swing and store_room, extended for the investigation): the same differences
from Unity's stack (room_r4's and store_room's B5) and the route source (worker_stops_in_lane), none of the realistic
base's. Its ranking flips in three cells by a failed F2 run or two against B5 at every run: door_then_beam (4/5 against
5/5), gate_right_swing (19/20 against 20/20, the deadlock above) and store_room (18/20 against 20/20): without the
realistic base, B5 alone passes the tight places every time and F2 occasionally does not (the chaotic F2 loop of CP1).

**Verdict.** In realistic mode the ranking holds in every cell but gate_right_swing, where an F2 deadlock after the
gate that the kinematic mode shows too keeps F2 under B5 (in ideal mode F2 also loses single runs at two more tight
places). Every other difference is Unity's stack (Nav2 replans, recoveries, aborts,
the collision monitor), the route source (no replanning around a stopped worker), a metric (static-only clearance in
Unity, the 2 m cut-off) or a planner struggle in a tight door that the gripping base makes more frequent
(store_room, door_105_mission). None is a plant defect with a physical fix; the planner findings go to execution fixes.
