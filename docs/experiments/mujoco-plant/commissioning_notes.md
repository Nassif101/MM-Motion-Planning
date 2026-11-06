## Analysis (hand-written; appended by the generator)

### What changed against the Task 7 profile

The realistic plant no longer imitates Unity's base. It stands on its wheels, with the URDF's settings and a ground
of mu 0.6. Every value either comes from the URDF or is listed in the profile's `assumptions` section with its
source. The software-stack delays from Task 7 stay (30 ms on the arm command path). `profile_unity.yaml` keeps the
Unity-calibrated planar profile, so the Task 7 results remain reproducible.

### Two MuJoCo-specific findings

1. **The wheel loop must stay slower than the contact.** MuJoCo's contacts are soft (a 20 ms time constant at the
   2 ms step). A wheel velocity loop much stiffer than that winds the contact up, and the base overshoots its
   acceleration ramp: by 0.24 m/s at kv 1000. A 0.5 ms step or stiffer contacts shrink the effect, which marks it as
   a simulation artefact. kv 80 is the stiffest loop that follows the ramp within 0.02 m/s.
2. **The effective track is 3.0 m (4.7x the geometric track).** Ideal Coulomb friction predicts about 1.9x, and
   Clearpath's A300 uses 1.75x. Rigid cylinder wheels with no tyre compliance, driven by a velocity loop softer than
   a real motor's, need more wheel-speed difference to turn. The value is measured, not tuned, and the planners'
   envelope stays reachable (0.9 of the 1.12 m/s rim limit).

### Simulator differences against Unity (for the thesis)

- **Speed.** This base achieves 0.99 of a commanded straight speed; Unity's achieves 0.93 (its tracking ratio in
  nav_operating_envelope.yaml). On straight runs the plant therefore pulls ahead of Unity by about 0.06 m per 2 s.
  This is most of the drift column above; the base targets are Task 7's, kept for comparison only.
- **Turns.** Unity's 0.02 floor lets its base slide: yaw overshoot, sideslip, over-speed after long turns. This base
  grips (mu 0.6) and follows the command. The F2 runs with hard turns differ the most (0.5–0.67 m per 2 s).
- **Breakaway.** This base starts from 0.005 m/s and 0.02 rad/s; Unity's needs 0.0375 m/s and 0.07 rad/s. The
  planners' envelope keeps Unity's (conservative) values; nothing they command is lost on this plant.
- **Arm.** Unchanged in substance from Task 7: the joints follow Unity's to about 1 mrad, and the tracking error is
  smaller on fast moves (Unity's JTC sees staler feedback). The arm's errors are dominated by the 30 ms command
  delay: every bandwidth from 5 to 15 Hz gives the same errors within 1 mrad, so the stated 10 Hz stands.

### What it means for the planners

The base does what it is told more closely than Unity's did. Commands inside the envelope are reached within a few
percent; the exceptions are the gentlest turns (0.79 of the command at 0.1 rad/s and 0.3 m/s) and the fastest
in-place turns (0.87). Whether planner outcomes depend on these differences is Task 11's question (per-cell
distributions over perturbed runs, against Unity's).
