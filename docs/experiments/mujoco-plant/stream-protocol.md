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
   `{"schema": 1, "server": "plant_demo_server", "mode": "ideal"|"realistic", "site_checksum": "...", "scenarios": [...]}`.
2. The client checks `schema` and `site_checksum`; the checksum is the site model the client should draw.
3. The client's first Control must be `{"cmd": "hello", "schema": 1}`. A different schema gets an error event, and
   then the server closes the connection. Any other command before the hello gets an error event.
4. The server serves one client at a time; a second client waits until the first one leaves. A client that reads too
   slowly loses frames: the server's backlog is capped at 4 MiB, and it counts the frames it drops.

## Control commands (JSON header, no blocks)

| `cmd` | Fields | Effect |
|---|---|---|
| `hello` | `schema` | Required first. |
| `list_scenarios` | — | Replies with an event `{"event": "scenarios", "scenarios": [...]}`. |
| `reset` | `scenario`; optional `bag`, `start` [x, y, yaw], `seed` (default 1) | A new run, paused. With a `bag`, the run follows the recording's goals in order; without one, the scenario's start and goal. The seed perturbs the start (±1 mm, ±1 mrad) and drives the plant's noise. The server streams the session Hello. |
| `set_goal` | `goal` [x, y, yaw] | Replaces the goals with a straight route from the robot. |
| `run` | `speed` (default 1) | Runs paced at `speed` × real time; 0 runs unpaced (as fast as the host allows). Running stops when the goals end. |
| `pause` | — | Stops running. |
| `step` | `n` (default 1) | Runs `n` B5 cycles now, then pauses. |
| `move_body` | `name`, `center` [x, y, z], `yaw` | Moves a mocap body reserved for mid-run use. A scenario's movers follow their own motion. |

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
- `movers`: `{name, size[3]}`.

**State.** Header fields:
- `t`: the plant's clock in seconds;
- `goal`: the index of the active goal;
- `status`: B5's cycle status;
- `fallback`: the medium layer's fallback reason, or "" while it follows a plan;
- `contacts`: the robot's current contacts with anything but the ground;
- `movers`: [[name, x, y], ...].

| Block | Floats | Layout |
|---|---|---|
| 0 | 9 | the true base x, y, yaw and the six arm joints (shoulder_pan … wrist_3), with no state delay |
| 1 | 9 | the state B5 sees: the same layout, after the realistic mode's state delays |
| 2 | 8 | B5's executed command: v, omega, six joint velocities |

**Plans.** Header: `t`, `plan_id`, `route_id`, `status` (`ok`, `infeasible` or `stale_reused`), `homotopy`, `solve_s`,
`capped`. Block 0 holds 9 floats per sample: x, y, yaw, then the six joints.

**Scan.** Header: `scan`, `t_start`, `t_end`, `rays`.
- Block 0: 3 floats per returned point, x y z in the lidar frame (`livox_frame`), each at its ray's time; the sweep
  carries the sensor's motion.
- Block 1: one float per point giving the hit class: 0 other, 1 ground, 2 robot, 3 panel, 4 site, 5 scenario,
  6 mover.

**Event.** Header: `event`, `t`, plus fields that depend on the event:
- `goal_reached`, `failed`, `timeout`: `goal`, `end_status`, `time_s`;
- `contact`: `robot_geom`, `other_geom`, `depth` (sent when contacts begin);
- `fallback`: `reason` (sent when a new fallback reason begins);
- `scenarios`, `error`: as above.

## Run logs

A run log is the bytes `MMRUNLG1` followed by chunks `[u32 raw_len][u32 zlib_len][zlib bytes]`. The chunks
decompress to whole frames, about 1 MiB each, starting with the session Hello. The same run writes the same bytes.
