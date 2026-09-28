import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobile_manipulator_navigation.telemetry import (  # noqa: E402
    EventLog, GapCounter, cross_track, downsample, path_length)


def test_downsample_keeps_ends_and_spacing():
    points = [(i * 0.05, 0.0) for i in range(41)]  # 2 m at 5 cm
    kept = downsample(points, 0.1)
    assert kept[0] == points[0] and kept[-1] == points[-1]
    assert all(math.dist(a, b) >= 0.1 - 1e-9 for a, b in zip(kept, kept[1:-1]))
    assert len(kept) == 21
    assert downsample([], 0.1) == [] and downsample([(1.0, 2.0)], 0.1) == [(1.0, 2.0)]


def test_path_length_and_cross_track():
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    assert math.isclose(path_length(path), 4.0)
    assert math.isclose(cross_track((1.0, 0.3), path), 0.3)
    assert math.isclose(cross_track((2.5, 1.0), path), 0.5)
    assert math.isclose(cross_track((-1.0, 0.0), path), 1.0)   # beyond the start
    assert cross_track((0.0, 0.0), []) is None


def test_event_log_is_bounded_and_ordered():
    log = EventLog(size=3)
    for index in range(5):
        log.add(float(index), f"e{index}")
    assert [e["text"] for e in log.as_list()] == ["e2", "e3", "e4"]


def test_gap_counter_rate_and_long_gaps():
    counter = GapCounter(0.5)
    gaps = [counter.add(0.1 * i) for i in range(11)]          # 10 Hz for 1 s
    assert all(g is None for g in gaps) and math.isclose(counter.rate(), 10.0)
    assert math.isclose(counter.add(1.8), 0.8)                  # 0.8 s gap
    assert counter.long_gaps == 1 and math.isclose(counter.max_gap, 0.8)
