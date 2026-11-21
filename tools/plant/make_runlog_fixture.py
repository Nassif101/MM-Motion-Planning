"""Writes fixtures/runlog_v3.mmlog, a small schema-3 run log shared by the Python and C# readers' tests (spec 4C Task 1).

A session Hello, 20 States 0.02 s apart, one Plans frame (F2 samples and 4B-2 display samples), two Predictions and
one Goal frame, in two zlib chunks split after the 7th frame. Run from tools/plant: python3 make_runlog_fixture.py
"""
import struct
import zlib
from pathlib import Path

import plant_stream as ps

OUT = Path(__file__).parent / "fixtures" / "runlog_v3.mmlog"


def frames():
    yield ps.encode(ps.HELLO, {
        "schema": 3, "scenario": "v2_door_105_nav", "seed": 1, "mode": "realistic", "epoch": 1,
        "capabilities": ["goal_frames", "route_frames", "cancel_goal", "plan_display_samples", "scan_point_times",
                         "b5_prediction"],
        "config": {"generated_routes": True, "medium": True, "lidar": True},
    }, [])
    for i in range(1, 21):
        t = round(0.02 * i, 2)
        pose = [0.5 * t, 0.0, 0.0, 0.0, 0.968, -1.735, 0.0, 0.767, 0.0]  # 0.5 m/s along x, the carry pose
        yield ps.encode(ps.STATE, {"t": t, "status": "ok", "execution": "tracking", "epoch": 1, "goal_id": 1},
                        [pose, pose, [0.0] * 8])
        if i == 5:  # t 0.1
            samples = [v for k in range(3) for v in (0.05 + 0.1 * k, 0, 0, 0, 0.968, -1.735, 0, 0.767, 0)]
            display = [v for k in range(4) for v in (0.1 * k, 0.05 + 0.05 * k, 0, 0, 0, 0.968, -1.735, 0, 0.767, 0)]
            yield ps.encode(ps.PLANS, {"t": t, "plan_id": 3, "status": "ok", "epoch": 1, "goal_id": 1},
                            [samples, display])
            yield ps.encode(ps.GOAL, {"event": "goal_accepted", "epoch": 1,
                                      "goal": {"goal_id": 1, "state": "tracking", "target": [5.0, 0.0, 0.0]}}, [])
        if i in (5, 10):  # t 0.1 and 0.2
            states = [v for k in range(21) for v in (0.5 * (t + 0.1 * k), 0, 0, 0, 0.968, -1.735, 0, 0.767, 0)]
            yield ps.encode(ps.PREDICTION, {"t": t, "status": "ok", "stage_dt": 0.1, "stages": 20, "plan_id": 3},
                            [states])


def main():
    all_frames = list(frames())
    data = b"MMRUNLG1"
    for part in (all_frames[:7], all_frames[7:]):
        raw = b"".join(part)
        packed = zlib.compress(raw)
        data += struct.pack("<II", len(raw), len(packed)) + packed
    OUT.write_bytes(data)
    print(f"{OUT}: {len(all_frames)} frames, {len(data)} bytes")


if __name__ == "__main__":
    main()
