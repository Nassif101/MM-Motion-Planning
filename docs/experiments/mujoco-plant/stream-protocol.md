# MuJoCo plant stream protocol, version 1

The demo server (`plant_demo_server`) streams a plant session to one TCP client, and run logs (`*.mmlog`) hold
the same frames. Any language can implement a client: the format needs only TCP, JSON, little-endian integers and
IEEE-754 float32. Reference implementations:

- C++: `mobile_manipulator_plant/frames.hpp` and `run_log.hpp`.
- Python: `tools/plant/plant_stream.py`, standard library only.
- Viewer: `tools/plant/view_run.py`.

## Frames

```
[u32 length][u16 type][u32 json_len][json bytes][u32 n_blocks]{[u32 n_floats][float32 x n_floats]}
```

- Every integer is little-endian.
- `length` counts the bytes after itself.
- The JSON is UTF-8. Each block is a flat float32 array, whose layout is set by the frame's type (below).
- A reader decodes a frame once `4 + length` bytes are buffered. Lengths that disagree with each other, an unknown
  type or bad JSON make the frame malformed.
- `tools/plant/fixtures/frame_v1.bin` holds a reference frame. The C++ and Python codecs are both tested against it.

| Type | Value | Direction |
|---|---|---|
| Hello | 1 | server to client at connect; a session's header at each reset; the first frame of a run log |
| Control | 2 | client to server |
| Event | 3 | server to client |
| State | 10 | server to client, one per plant step (every B5 cycle, 20 ms) |
| Plans | 11 | server to client, one per medium plan |
| Scan | 12 | server to client, one per lidar scan (decimated as configured) |

## Connecting

1. The server sends a **server Hello**:
   `{"schema": 2, "server": "plant_demo_server", "mode": "ideal"|"realistic", "site_checksum": "...", "scenarios": [...]}`.
2. The client checks `schema` and `site_checksum`; the checksum is the site model the client should draw.
3. The client's first Control must be `{"cmd": "hello", "schema": 2}`. A different schema gets an error event, and
   then the server closes the connection. Any other command before the hello gets an error event.
4. The server serves one client at a time; a second client waits until the first one leaves. A client that has not
   said hello within 5 s is disconnected, and so is one that announces a frame over 64 KiB (a Control frame is a small
   JSON header).
5. A client that reads too slowly loses State, Plans and Scan frames: the server's backlog is capped at 4 MiB. Hello
   and Event frames are never dropped. The server reports drops with a `dropped` event, at most once a second.
6. Commands other than `hello`, `list_scenarios`, `reset` and `pause` need a reset first (an error event otherwise).
7. A paced run that the host cannot keep up with runs as fast as it can: the server spends at most 50 ms stepping
   between reads of the client, then re-anchors the pace instead of catching up.

## Control commands (JSON header, no blocks)

| `cmd` | Fields | Effect |
|---|---|---|
| `hello` | `schema` | Required first. |
| `list_scenarios` | — | Replies with an event `{"event": "scenarios", "scenarios": [...]}`. |
| `reset` | `scenario`; optional `bag`, `start` [x, y, yaw], `seed` (default 1) | A new run, paused. With a `bag`, the run follows the recording's goals in order (Nav2's replans of one goal count once); without one, the scenario's start and goal. The seed perturbs the start (±1 mm, ±1 mrad) and drives the plant's noise. The server streams the session Hello. |
| `set_goal` | `goal` [x, y, yaw] | Replaces the goals with a straight route from the robot. |
| `run` | `speed` (default 1) | Runs paced at `speed` × real time; 0 runs unpaced (as fast as the host allows). Running stops when the goals end. |
| `pause` | — | Stops running. |
| `step` | `n` (default 1) | Runs `n` B5 cycles now, then pauses. |
| `move_body` | `name`, `center` [x, y, z], `yaw` | Places one of the session Hello's `reserved` bodies (an obstacle appearing mid-run). The lidar, contacts and the ground truth see it; the planners do not. A scenario's movers follow their own motion and cannot be moved. |

Errors come back as `{"event": "error", "reason": "..."}`.

## Server frames

**Session Hello** (after each reset; also the first frame of a run log). Fields:
- `schema`, `scenario`, `bag`, `seed`;
- `mode`: `ideal` or `realistic`;
- `medium`: false when B5 runs alone;
- `profile`: the robot profile's path on the server;
- `site_checksum`;
- `start`: [x, y, yaw], perturbation included;
- `goals`: [[x, y, yaw], ...];
- `boxes`: the scenario's boxes `{name, center[3], size[3], yaw}`, including any the site already has. A client skips
  those whose `site/<name>` geom is in the site model;
- `movers`: `{name, size[3]}`;
- `reserved`: the bodies `move_body` can place, `{name, size[3]}` (`reserved/0` … `reserved/3`), parked out of sight
  until moved;
- `provenance`: what the server or runner was started from (`git_commit`, `git_dirty`), `{}` outside a repository;
- `profile_sha256`, `robot_model_sha256` (the generated model the profile's base uses), `site_model_sha256`;
- `config`: `medium`, `medium_config`, `lidar`, `scan_decimation`, `timeout_s`, `iteration_budget`, `perturb_start`;
  from schema 2 (E1) also `b5_config` (the B5 config variant given, "" for the installed `wholebody_mpc.yaml`),
  `b5_config_sha256` (of the file loaded), `gate_rule` (`increment` or `absolute`, B5's `slack_gate_inside`) and
  `start_nudge_rad` (added to the start yaw after the seed's perturbation; twin runs).

**State.** Header fields:
- `t`: the plant's clock in seconds;
- `goal`: the index of the active goal;
- `status`: B5's cycle status;
- `fallback`: the medium layer's fallback reason, or "" while it follows a plan;
- `contacts`: the robot's current contacts, the wheels' contacts with the ground excepted (a chassis or arm touching the
  ground counts);
- `movers`: [[name, x, y], ...];
- `bodies`: [[name, x, y, yaw], ...] of the reserved bodies placed so far;
- `gate`: B5's slack gate this cycle, [gate_slack m, gate_violation m, unsafe 0/1] (diagnostics: a hold starts on
  unsafe);
- `dwell` (schema 2): 1 while B5 holds (`holding`) with a plan that passes the gate but waits out the resume dwell,
  else 0;
- `diag` (only with the session's diagnostics, `plant_campaign --diagnostics`; E1 Task 9): [the deepest pair among the
  gate's rows with slack: its current violation m, that pair's stage-1 slack m (the plan's linearized violation one
  stage ahead), the plan's largest linearization error over its rows of stages 1-10 m, or null without a solve];
- `layout`: a reminder of the blocks' layout.

| Block | Floats | Layout |
|---|---|---|
| 0 | 9 | the true base x, y, yaw and the six arm joints (shoulder_pan … wrist_3), with no state delay |
| 1 | 9 | the state B5 sees: the same layout, after the realistic mode's state delays |
| 2 | 8 | what the plant executed, as B5's closed loop sees it: in ideal mode the command (v, omega, six joint velocities); in realistic mode the measured base twist (after the state delay) and the arm reference's joint velocities |

**Plans.** Header: `t`, `plan_id`, `route_id`, `status` (`ok`, `infeasible` or `stale_reused`), `homotopy`, `capped`;
from schema 2 (E1's decision trace) also `seed` (the winning seed; on `stale_reused` the reused plan's, which this
cycle's `seeds` may not list), `full_multistart`, `waiting` (the planner's waiting
state after the cycle, which widens the next one), `homotopy_switched` and `seeds`, one `[name, merit, feasible 0/1,
progress_m, merged 0/1]` per seed of the cycle (no timings; `merged`: the seed's solution is an earlier seed's, from
2026-11-08 CP1; earlier schema-2 logs have four entries).
No wall-clock solve time (it would make same-seed logs differ, E2): the run summary has `medium_solve_s`. Block 0 holds 9 floats per sample: x, y, yaw, then the six joints.

**Scan.** Header: `scan`, `t_start`, `t_end`, `rays`.
- Block 0: 3 floats per returned point, x y z in the lidar frame (`livox_frame`), each at its ray's time; the sweep
  carries the sensor's motion.
- Block 1: one float per point giving the hit class: 0 site, 1 scenario, 2 mover, 3 ground, 4 robot, 5 panel,
  6 other (the plant's `HitClass` values, `lidar.hpp`; `plant_stream.HIT_CLASSES` names them in this order).

**Event.** Header: `event`, `t`, plus fields that depend on the event:
- `goal_reached`, `failed`, `timeout`, `stalled`: `goal`, `end_status`, `time_s` (`stalled`: no progress for the
  session's stall time, the base within 5 cm and 0.1 rad and every arm joint within 0.05 rad; campaigns 45 s, the demo
  server off);
- `contact`: `robot_geom`, `other_geom`, `depth` (sent when contacts begin);
- `fallback`: `reason` (sent when a new fallback reason begins);
- `dropped`: `frames` (the State, Plans and Scan frames dropped since the last report);
- `scenarios`, `error`: as above.

## Run logs

Schema 2 (E1, 2026-11-08) adds the decision-trace fields above; `plant_stream.RunLog` reads schema 1 and 2 logs (the
baseline replicas are schema 1), the demo server and its client speak schema 2 only.

A run log is the bytes `MMRUNLG1` followed by chunks `[u32 raw_len][u32 zlib_len][zlib bytes]`. The chunks
decompress to whole frames, about 1 MiB each, starting with the session Hello. The same run writes the same bytes.
