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
   envelope stays reachable (0.9 of the 1.12 m/s rim limit). The amendment's contact variants (a throwaway probe in
   the spike model, 2026-11-06) don't bring it near 2x without breaking something else:

   | Wheel contact | Effective track | Side effect |
   |---|---|---|
   | cylinder (the URDF's) | 4.7-5.5x | reference |
   | sphere, ellipsoid | 4.7x | none (no change) |
   | softer contact (solref 0.04, tyre-like give) | 3.9x | gentle turns 0.66 of command; fails the ramp check (0.029 m/s) |
   | lower lateral friction (0.3 across, 0.6 along) | 3.1x | gentle turns collapse (0.29) |
   | lower longitudinal friction | 4.7x | (0.3, 0.3) turns at 0.52 |

   The multiplier is a property of MuJoCo's rigid-wheel contact, not of the wheel's shape; the cylinder stays.
3. **What it can and can't affect.** The effective track lives in the base controller's (v, omega) -> wheel-speed
   mapping, so the planners never see it; they see the response measured above. Its indirect effects:
   - the reachable envelope shrinks (|v| + 1.5 |omega| <= 1.12 m/s), guarded by the consistency test;
   - rigid wheels may resist turning more than real tyres, so the base slides less in turns than a real one. That
     could favour planners that turn tightly (F2: 5-12 % hard turns, B5 alone: none).

   - the effective track is fitted in carry; across the other qualified poses (home, vertical carry, level extension,
     fold) medium and in-place turns stay at 0.88-1.02 of command, and the gentlest turn (0.1 rad/s) at 0.67-0.77
     (0.79 in carry): the understeer at small yaw rates depends on the load, and B5 closes the loop on it.

   Mitigation: Unity's base errs the other way (its 0.02 floor slides), so the two simulators bracket a real robot.
   Task 11 compares per-cell outcome distributions on both, and adds a grip ensemble (mu 0.4 and 0.8) on a subset of
   cells. A ranking that holds across both simulators and across grip doesn't depend on the base model.

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
  delay. The servo alone (the delay removed, after the Task 7b review) is stable to 30 Hz and well damped to 15 Hz; the
  stated 10 Hz stands (user ruling 2026-11-06): 1 % overshoot, 0.1 s settling, the slow move within 2-6 mrad.

### What it means for the planners

The base does what it is told more closely than Unity's did. Commands inside the envelope are reached within a few
percent; the exceptions are the gentlest turns (0.79 of the command at 0.1 rad/s and 0.3 m/s) and the fastest
in-place turns (0.87). Whether planner outcomes depend on these differences is Task 11's question (per-cell
distributions over perturbed runs, against Unity's).
