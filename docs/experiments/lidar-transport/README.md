# Lidar transport: filtering before publishing

Date: 2026-09-28. Question: does removing the lidar's zero-point misses in Unity, before
publishing, reduce transport load and improve timing of the small control/state
messages that share the ROS-TCP connection?

## Method

Each condition started from the same state: Unity out of Play, a freshly started
`unity_control_endpoint.py`, then `tools/run_nav_scenario.py --new-epoch open_space`
(arm control in HOLD, global planning active, robot stationary at the open fixture),
15 s settling, then two 30 s wall-clock windows of

```bash
ros2 run mobile_manipulator_navigation sensor_transport_probe.py --seconds 30 --label <condition>
```

The probe records lidar bytes and rate, wall-clock inter-arrival gaps of `/clock`,
`/tf`, and `/arm/state` at the ROS receiver, and the endpoint process CPU from `/proc`.
These are receiver-side observations, not one-way latency.

Conditions:

- **A** - committed setup: UnitySensors `LiDARPointCloud2MsgPublisher`, 20,000 points
  per scan including misses (fixed 320,000-byte message).
- **B** - project publisher that omits zero points (variable length, about 8,500 points),
  publishing once per sensor sample.
- **C** - the same project publisher temporarily keeping misses (fixed 320,000 bytes),
  to separate message size from publisher implementation.

## Results

| Condition | Lidar MB/s | Endpoint CPU | `/clock` gap p50 / p99 / max (ms) | `/arm/state` gap p99 / max (ms) |
|---|---:|---:|---|---|
| A-1 | 3.09 | 44.4 % | 20.0 / 29.7 / 48.3 | 29.6 / 48.3 |
| A-2 | 3.06 | 48.0 % | 19.8 / 32.8 / 331.6 | 32.9 / 350.1 |
| B-1 | 1.36 | 65.6 % | 20.0 / 51.3 / 98.8 | 51.2 / 98.8 |
| B-2 | 1.36 | 72.7 % | 19.7 / 62.4 / 119.2 | 64.5 / 119.1 |
| C-1 | 3.17 | 50.5 % | 19.9 / 35.4 / 335.0 | 35.6 / 334.8 |
| C-2 | 3.18 | 51.4 % | 19.8 / 36.5 / 326.6 | 36.7 / 348.1 |

Isolated maxima of 300-350 ms in A-2 and C occur in both publishers and match earlier
Editor-stall observations; the p99 values are the comparison.

## Conclusion

Dropping misses in Unity cut lidar bytes by 57 % but **raised** endpoint CPU by about
20 percentage points and roughly doubled the p99 gap of `/clock` and `/arm/state`.
Condition C shows that variable message length, not the byte count or the publisher,
causes the regression in this transport (`rmw_fastrtps_cpp` behind the Python
endpoint). No profiler was available in the container to identify the mechanism.
The managed-code publisher was also slightly worse than UnitySensors' Burst-based
serializer even at full size (C vs A).

Decision: keep UnitySensors' fixed-size cloud, including misses, and remove misses in the
ROS perception filter as the local-costmap contract already requires. The trial
publisher was not committed.

## Side observation: long-running endpoint

Before these controlled runs, an endpoint that had served about 3 hours and a dozen Play
epochs measured 59 % CPU with the lidar on and about 73 % with the lidar publisher
disabled, against 42-48 % for a fresh endpoint. Its thread (19-20) and socket (9) counts
did not grow across Play restarts, so the cause is unidentified. `run_nav_scenario.py
--new-epoch` now restarts the endpoint so measurements start from the same state.

## Follow-up: Unity TF listener

The project `ROSConnectionPrefab` had `listenForTFMessages` enabled, so ROS-TCP-Connector's
TF visualization subscribed to `/tf` and the endpoint streamed all ROS transforms back to
Unity, including Unity's own 50 Hz base transform. No project code uses `TFSystem`.
With it disabled (condition **D**, same method as A):

| Condition | Lidar MB/s | Endpoint CPU | `/clock` gap p50 / p99 / max (ms) | `/arm/state` gap p99 / max (ms) |
|---|---:|---:|---|---|
| D-1 | 3.13 | 42.6 % | 20.1 / 29.9 / 69.9 | 30.6 / 69.9 |
| D-2 | 2.92 | 42.0 % | 20.0 / 33.8 / 317.4 | 34.0 / 336.7 |

Endpoint CPU fell by 2-6 points against A; jitter is unchanged within run-to-run noise.
The endpoint no longer registers a `/tf` subscriber, removing the echo loop.
