"""Pure telemetry helpers for the navigation telemetry node (no ROS dependencies)."""
import math
from collections import deque

GOAL_STATUS = {0: "unknown", 1: "accepted", 2: "executing", 3: "canceling",
               4: "succeeded", 5: "canceled", 6: "aborted"}
MONITOR_ACTION = {0: "none", 1: "stop", 2: "slowdown", 3: "approach", 4: "limit"}


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def downsample(points, spacing):
    """Keep the first point, then points at least `spacing` apart, and always the last."""
    if not points:
        return []
    kept = [points[0]]
    for point in points[1:-1]:
        if math.dist(point[:2], kept[-1][:2]) >= spacing - 1e-9:  # tolerate float steps
            kept.append(point)
    if len(points) > 1:
        kept.append(points[-1])
    return kept


def cross_track(position, points):
    """Distance from position to the polyline through points (None without a path)."""
    if not points:
        return None
    if len(points) == 1:
        return math.dist(position, points[0][:2])
    best = math.inf
    px, py = position
    for (ax, ay, *_), (bx, by, *_) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        s = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        best = min(best, math.hypot(px - (ax + s * dx), py - (ay + s * dy)))
    return best


class EventLog:
    """Bounded log of (simulation time, text) events, newest last."""

    def __init__(self, size=15):
        self.events = deque(maxlen=size)

    def add(self, time_s, text):
        self.events.append({"t": round(time_s, 2), "text": text})

    def as_list(self):
        return list(self.events)


class GapCounter:
    """Rate and long gaps of a periodic stream from its message stamps."""

    def __init__(self, gap_threshold, window=20):
        self.threshold = gap_threshold
        self.stamps = deque(maxlen=window)
        self.long_gaps = 0
        self.max_gap = 0.0

    def add(self, stamp):
        """Returns the gap when it exceeds the threshold, else None."""
        gap = None
        if self.stamps and stamp > self.stamps[-1]:
            delta = stamp - self.stamps[-1]
            self.max_gap = max(self.max_gap, delta)
            if delta > self.threshold:
                self.long_gaps += 1
                gap = delta
        self.stamps.append(stamp)
        return gap

    def rate(self):
        if len(self.stamps) < 2 or self.stamps[-1] <= self.stamps[0]:
            return 0.0
        return (len(self.stamps) - 1) / (self.stamps[-1] - self.stamps[0])
