"""Bounded analytical fixtures for the standalone ROS-free route_check executable.

    python3 tools/plant/qualify_routing.py --binary PATH --primitives PATH --out DIR

This is a synthetic 0.20 x 0.16 m footprint qualification, not a robot campaign or a
Nav2 equivalence study. Inputs and bounded raw process output are retained. The
independent geometry check samples interpolation with a <=2.5 mm point-motion
bound; passing it does not independently prove continuous swept-volume safety.
Only the standard library is required.
"""

import argparse
import dataclasses
import hashlib
import json
import math
from pathlib import Path
import subprocess
import threading
import time


RESOLUTION = 0.05
WIDTH = HEIGHT = 60
ORIGIN = (-1.5, -1.5)
FOOTPRINT = ((0.10, 0.08), (0.10, -0.08), (-0.10, -0.08), (-0.10, 0.08))
MAX_POINT_STEP_M = 0.0025
MAX_PATH_POSES = 100000
MAX_VALIDATION_SAMPLES = 2000000
OUTPUT_LIMIT_BYTES = 4 * 1024 * 1024  # combined stdout/stderr prefix, not per pipe
MAX_PRIMITIVE_BYTES = 4 * 1024 * 1024
STATUSES = {"found", "invalid_start", "invalid_goal", "no_path", "budget_exhausted", "canceled", "stale", "rejected"}
# Smac Lattice (spec 2 Amendment 1): the search ends within this distance of the requested goal (its 0.20 m tolerance
# plus half a cell), the start heading is snapped to the nearest lattice heading, and smoothing re-estimates headings.
GOAL_TOLERANCE_M = 0.20 + RESOLUTION / 2
START_POSITION_TOLERANCE_M = 1e-5
EPS = 1e-12


@dataclasses.dataclass(frozen=True)
class Fixture:
    name: str
    start: tuple
    goal: tuple
    expected_status: str
    occupied: frozenset
    case_path: Path
    width: int = WIDTH
    height: int = HEIGHT
    resolution: float = RESOLUTION
    origin: tuple = ORIGIN
    footprint: tuple = FOOTPRINT


def _dump(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def generate_fixtures(directory, primitives):
    """Write fixed analytical cases. Caller supplies an empty output directory."""
    directory, primitives = Path(directory), Path(primitives)
    with primitives.open("rb") as stream:
        raw = stream.read(MAX_PRIMITIVE_BYTES + 1)
    if not raw or len(raw) > MAX_PRIMITIVE_BYTES:
        raise ValueError("primitive input must be nonempty and no larger than 4 MiB")
    assets = directory / "assets"
    assets.mkdir()
    (assets / "primitives.json").write_bytes(raw)
    vertical = frozenset((x, y) for x in (29, 30) for y in range(HEIGHT))
    specifications = [
        ("open", (-1.0, -0.8, 0.0), (1.0, 0.8, 0.0), "found", frozenset()),
        ("detour", (-1.0, -0.6, 0.0), (1.0, -0.6, 0.0), "found",
         frozenset((x, y) for x in (29, 30) for y in range(36))),
        ("narrow_door", (-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), "found",
         frozenset((x, y) for x, y in vertical if not 27 <= y < 33)),
        ("impassable_wall", (-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), "no_path", vertical),
        # Smac does not reject an occupied goal when its tolerance is positive; here every pose within the 0.20 m
        # tolerance lies in the box's inflated ring, so the search ends with no path
        ("occupied_goal", (-1.0, 0.0, 0.0), (0.8, 0.0, 0.0), "no_path",
         frozenset((x, y) for x in range(44, 48) for y in range(28, 32))),
        ("outside_goal", (-1.0, 0.0, 0.0), (2.0, 0.0, 0.0), "invalid_goal", frozenset()),
        ("heading_only", (0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2), "found", frozenset()),
        ("off_grid", (-1.037, -0.823, 0.17), (1.013, 0.677, -0.21), "found", frozenset()),
    ]
    fixtures = []
    for name, start, goal, expected, occupied in specifications:
        case_dir = directory / name
        case_dir.mkdir()
        pixels = bytes(0 if (x, HEIGHT - 1 - image_y) in occupied else 255
                       for image_y in range(HEIGHT) for x in range(WIDTH))
        (case_dir / "map.pgm").write_bytes(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode() + pixels)
        (case_dir / "map.yaml").write_text(
            "image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.5, -1.5, 0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n", encoding="utf-8")
        case_path = case_dir / "case.json"
        _dump(case_path, {"map": "map.yaml", "primitives": "../assets/primitives.json",
                         "footprint": FOOTPRINT, "start": start, "goal": goal,
                         "search": {"max_expansions": 1000000,
                                    "max_path_poses": MAX_PATH_POSES, "expansions_per_tick": 256,
                                    "cell_tests_per_tick": 16384}})
        fixtures.append(Fixture(name, start, goal, expected, occupied, case_path))
    _dump(directory / "fixtures.json", {
        "description": "Fixed analytical cases; synthetic footprint, not the physical mobile manipulator",
        "primitive_sha256": hashlib.sha256(raw).hexdigest(), "resolution_m": RESOLUTION,
        "width": WIDTH, "height": HEIGHT, "origin": ORIGIN, "footprint": FOOTPRINT,
        "max_point_step_m": MAX_POINT_STEP_M,
        "cases": [{"name": f.name, "input": f"{f.name}/case.json", "expected_status": f.expected_status,
                   "start": f.start, "goal": f.goal, "occupied_cells": sorted(f.occupied)} for f in fixtures]})
    return fixtures


def _orientation(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(point, a, b):
    return (abs(_orientation(a, b, point)) <= EPS
            and min(a[0], b[0]) - EPS <= point[0] <= max(a[0], b[0]) + EPS
            and min(a[1], b[1]) - EPS <= point[1] <= max(a[1], b[1]) + EPS)


def segments_intersect(a, b, c, d):
    """Closed segments, including edge/corner touching; independent of the core's SAT checks."""
    ab_c, ab_d = _orientation(a, b, c), _orientation(a, b, d)
    cd_a, cd_b = _orientation(c, d, a), _orientation(c, d, b)
    if ((ab_c > EPS and ab_d < -EPS) or (ab_c < -EPS and ab_d > EPS)) and (
            (cd_a > EPS and cd_b < -EPS) or (cd_a < -EPS and cd_b > EPS)):
        return True
    return any(_on_segment(p, u, v) for p, u, v in ((a, c, d), (b, c, d), (c, a, b), (d, a, b)))


def point_in_polygon(point, polygon):
    """Ray crossing with an explicit closed boundary; accepts concave simple polygons too."""
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if _on_segment(point, a, b):
            return True
        if (a[1] > point[1]) != (b[1] > point[1]):
            cross_x = a[0] + (point[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if cross_x > point[0]:
                inside = not inside
    return inside


def polygons_intersect(first, second):
    if any(point_in_polygon(p, second) for p in first) or any(point_in_polygon(p, first) for p in second):
        return True
    return any(segments_intersect(a, b, c, d)
               for a, b in zip(first, first[1:] + first[:1])
               for c, d in zip(second, second[1:] + second[:1]))


def _pose_collision(fixture, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    polygon = [(pose[0] + c * x - s * y, pose[1] + s * x + c * y) for x, y in fixture.footprint]
    x0, y0 = fixture.origin
    x1, y1 = x0 + fixture.width * fixture.resolution, y0 + fixture.height * fixture.resolution
    if any(x <= x0 + EPS or x >= x1 - EPS or y <= y0 + EPS or y >= y1 - EPS for x, y in polygon):
        return "footprint touches or crosses the map boundary"
    # Visit all full occupied squares in the footprint's bounding box, not only cell centres.
    low_x = max(0, math.floor((min(x for x, _ in polygon) - x0) / fixture.resolution) - 1)
    high_x = min(fixture.width - 1, math.floor((max(x for x, _ in polygon) - x0) / fixture.resolution) + 1)
    low_y = max(0, math.floor((min(y for _, y in polygon) - y0) / fixture.resolution) - 1)
    high_y = min(fixture.height - 1, math.floor((max(y for _, y in polygon) - y0) / fixture.resolution) + 1)
    for y in range(low_y, high_y + 1):
        for x in range(low_x, high_x + 1):
            if (x, y) not in fixture.occupied:
                continue
            left, bottom = x0 + x * fixture.resolution, y0 + y * fixture.resolution
            right, top = left + fixture.resolution, bottom + fixture.resolution
            if polygons_intersect(polygon, [(left, bottom), (right, bottom), (right, top), (left, top)]):
                return f"footprint intersects occupied cell ({x}, {y})"
    return None


def _pose(value):
    if (not isinstance(value, list) or len(value) != 3 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
        raise ValueError("each path pose must contain three finite numeric values")
    return value


def validate_path(fixture, result):
    """Check found-path endpoints and independently sampled footprint validity."""
    poses = result.get("poses")
    if not isinstance(poses, list) or not 1 <= len(poses) <= MAX_PATH_POSES:
        return {"pass": False, "reason": "found path must have 1..100000 poses", "samples_checked": 0}
    try:
        poses = [_pose(p) for p in poses]
        if result.get("kind") not in {"stationary", "translating"}:
            raise ValueError("found result must identify a stationary or translating path")
        if math.hypot(poses[0][0] - fixture.start[0], poses[0][1] - fixture.start[1]) > START_POSITION_TOLERANCE_M:
            raise ValueError("path does not start at the requested start position")
        if math.hypot(poses[-1][0] - fixture.goal[0], poses[-1][1] - fixture.goal[1]) > GOAL_TOLERANCE_M:
            raise ValueError("path does not end within the search tolerance of the goal")
        requested = _pose(result.get("requested_goal"))
        if math.hypot(requested[0] - fixture.goal[0], requested[1] - fixture.goal[1]) > 1e-7 or abs(
                math.remainder(requested[2] - fixture.goal[2], 2 * math.pi)) > 1e-7:
            raise ValueError("requested_goal differs from the requested goal")
        resolved = _pose(result.get("resolved_goal"))
        if math.hypot(resolved[0] - poses[-1][0], resolved[1] - poses[-1][1]) > 1e-9:
            raise ValueError("resolved_goal is not the path's last pose")
        value = result.get("length_m")
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError("length_m must be finite and nonnegative")
        radius = max(math.hypot(x, y) for x, y in fixture.footprint)
        count, measured_length = 0, 0.0
        previous = poses[0]
        for end in poses:
            length = math.hypot(end[0] - previous[0], end[1] - previous[1])
            dyaw = math.remainder(end[2] - previous[2], 2 * math.pi)
            steps = max(1, math.ceil((length + radius * abs(dyaw)) / MAX_POINT_STEP_M))
            if count + steps > MAX_VALIDATION_SAMPLES:
                raise ValueError("path exceeds the bounded validation sample budget")
            for index in range(1, steps + 1):
                u = index / steps
                point = (previous[0] + u * (end[0] - previous[0]),
                         previous[1] + u * (end[1] - previous[1]), previous[2] + u * dyaw)
                collision = _pose_collision(fixture, point)
                count += 1
                if collision:
                    return {"pass": False, "reason": collision, "samples_checked": count}
            measured_length += length
            previous = end
        if abs(measured_length - result["length_m"]) > 1e-6 * max(1.0, measured_length):
            raise ValueError("reported length differs from the raw pose-path length")
        if fixture.name == "heading_only" and (result.get("kind") != "stationary" or measured_length > 1e-7):
            raise ValueError("heading-only result introduces translation or the wrong path kind")
        return {"pass": True, "reason": None, "samples_checked": count, "measured_length_m": measured_length}
    except (ValueError, TypeError, OverflowError) as error:
        return {"pass": False, "reason": str(error), "samples_checked": 0}


def capture_process(command, directory, timeout_s, output_limit=OUTPUT_LIMIT_BYTES):
    """Capture bounded raw prefixes; kill a process on timeout or combined output overflow."""
    if not math.isfinite(timeout_s) or timeout_s <= 0 or not isinstance(output_limit, int) or output_limit <= 0:
        raise ValueError("capture requires a positive finite timeout and positive integer output limit")
    directory = Path(directory)
    timed_out, failed = False, None
    state = {"bytes": 0, "limited": False}
    lock = threading.Lock()
    start = time.monotonic()
    process = None
    with (directory / "stdout.bin").open("wb") as stdout_file, (directory / "stderr.bin").open("wb") as stderr_file:
        try:
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            def drain(pipe, destination):
                while True:
                    chunk = pipe.read(65536)
                    if not chunk:
                        break
                    with lock:
                        available = max(0, output_limit - state["bytes"])
                        destination.write(chunk[:available])
                        state["bytes"] += min(available, len(chunk))
                        if len(chunk) > available:
                            state["limited"] = True
                            try:
                                process.kill()
                            except ProcessLookupError:
                                pass

            readers = [threading.Thread(target=drain, args=(pipe, target), daemon=True)
                       for pipe, target in ((process.stdout, stdout_file), (process.stderr, stderr_file))]
            for reader in readers:
                reader.start()
            try:
                process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                timed_out = True
                process.kill()
                process.wait(timeout=5)
            for reader in readers:
                reader.join(timeout=5)
            if any(reader.is_alive() for reader in readers):
                failed = "output reader did not finish after process termination"
            process.stdout.close()
            process.stderr.close()
        except (OSError, subprocess.SubprocessError) as error:
            failed = f"{type(error).__name__}: {error}"
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=5)
    return {"returncode": process.returncode if process is not None else None,
            "timed_out": timed_out, "output_limited": state["limited"], "error": failed,
            "elapsed_s": time.monotonic() - start, "captured_bytes": state["bytes"]}


def run_case(binary, fixture, timeout_s):
    capture = capture_process([str(binary), str(fixture.case_path.resolve())], fixture.case_path.parent, timeout_s)
    errors, result, validation = [], None, None
    if capture["error"]:
        errors.append(capture["error"])
    if capture["timed_out"]:
        errors.append("process timeout")
    if capture["output_limited"]:
        errors.append("process output exceeded its bounded capture")
    if not errors:
        try:
            raw = (fixture.case_path.parent / "stdout.bin").read_bytes()
            result = json.loads(raw, parse_constant=_reject_constant, object_pairs_hook=_unique_object)
            if (not isinstance(result, dict) or not isinstance(result.get("status"), str)
                    or result["status"] not in STATUSES):
                raise ValueError("result is not an object with a recognized terminal status")
            status = result["status"]
            expected_code = 0 if status == "found" else 2
            if capture["returncode"] != expected_code:
                errors.append(f"status {status} requires exit {expected_code}, got {capture['returncode']}")
            if status != fixture.expected_status:
                errors.append(f"expected {fixture.expected_status}, got {status}")
            if status == "found":
                validation = validate_path(fixture, result)
                if not validation["pass"]:
                    errors.append(validation["reason"])
        except (ValueError, UnicodeDecodeError, OSError, RecursionError, TypeError) as error:
            errors.append(f"malformed/missing result: {error}")
    return {"name": fixture.name, "expected_status": fixture.expected_status, "pass": not errors,
            "errors": errors, "process": capture, "result": result, "validation": validation,
            "input": f"{fixture.name}/case.json", "stdout": f"{fixture.name}/stdout.bin",
            "stderr": f"{fixture.name}/stderr.bin"}


def _reject_constant(value):
    raise ValueError(f"non-finite JSON constant {value}")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON field {key}")
        result[key] = value
    return result


def _file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run_suite(binary, primitives, out, timeout_s=30.0):
    binary, primitives, out = Path(binary).resolve(), Path(primitives).resolve(), Path(out).resolve()
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= 300:
        raise ValueError("timeout must be finite and in (0, 300] seconds per case")
    if not binary.is_file() or not primitives.is_file():
        raise ValueError("binary and primitive input must be regular files")
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise ValueError("output directory must be absent or empty; existing evidence is not overwritten")
    out.mkdir(parents=True, exist_ok=True)
    fixtures = generate_fixtures(out, primitives)
    cases = [run_case(binary, fixture, timeout_s) for fixture in fixtures]
    report = {"schema": 1, "pass": all(case["pass"] for case in cases),
              "scope": "analytical synthetic-footprint fixtures; not robot or Nav2 equivalence qualification",
              "validator": {"method": "segment intersection and containment with interpolated pose samples",
                            "max_point_step_m": MAX_POINT_STEP_M,
                            "limit": "sampled evidence, not an independent proof of complete continuous swept safety"},
              "limits": {"timeout_s_per_case": timeout_s, "output_bytes_per_case": OUTPUT_LIMIT_BYTES,
                         "max_validation_samples": MAX_VALIDATION_SAMPLES},
              "binary": {"name": binary.name, "sha256": _file_hash(binary)},
              "primitive_sha256": _file_hash(out / "assets/primitives.json"), "cases": cases}
    _dump(out / "report.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--primitives", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args(argv)
    try:
        report = run_suite(args.binary, args.primitives, args.out, args.timeout_s)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps({"pass": report["pass"], "cases": len(report["cases"]),
                      "report": str(args.out / "report.json")}))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
