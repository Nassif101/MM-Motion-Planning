"""Gate L against Unity: the plant's lidar and Unity's at fixed site poses (MuJoCo plant plan Task 10, spec Section 7).

At each fixture pose (lidar_fixture_poses.json; the arm in carry) Unity's raw /livox/lidar scans and the plant's are
compared by:
- the hit fraction per class (site, scenario, ground, robot, panel; pass: within +-5 percentage points). The plant
  labels its own points; Unity's get the nearest primitive within `tol` (the site and scenario boxes, the ground plane,
  the robot's collision geoms and the panel, all from the plant's model at the pose), else "other";
- the self-hit regions: robot and panel points binned on a 5 cm voxel grid in the base frame (pass: Jaccard >= 0.7,
  a threshold to confirm at CP4);
- minimum-range and blind-zone violations (pass: none): points nearer than 0.1 m, or outside the Mid-360 pattern's
  elevations (-7.21 to +52.16 deg, from mid360.csv). Zero points are misses (UnitySensors publishes them), not points.

Subcommands:
    .venv-plant/bin/python tools/plant/compare_lidar.py record --out experiment_runs/lidar-fixtures-DATE   (host; Unity Editor up
        or startable, the container up, no timed job; stops Play at the end)
    python3 tools/plant/compare_lidar.py export DIR/FIXTURE   (container, record calls it: the bag -> unity.npz)
    .venv-plant/bin/python tools/plant/compare_lidar.py plant --out DIR   (host -> container: plant_lidar_fixture per fixture -> plant.mmlog)
    python3 tools/plant/compare_lidar.py compare DIR [--report docs/experiments/mujoco-plant/lidar.md]   (container: numpy)
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).with_name("lidar_fixture_poses.json")
CONTAINER = "mm-motion-planning-ma-robot-sim-1"
WORKSPACE = "/workspaces/mm-motion-planning"
CLASSES = ["site", "scenario", "ground", "robot", "panel", "mover", "other"]
GATED = ["site", "scenario", "ground", "robot", "panel"]
SELF = ("robot", "panel")
MIN_RANGE_M = 0.1
FOV_DEG = (90.0 - 97.2123, 90.0 - 37.836)  # mid360.csv's zenith range 37.836..97.2123 deg
FOV_TOL_DEG = 0.1
FRACTION_TOL_PP = 5.0
VOXEL_M = 0.05
JACCARD_MIN = 0.7
SCANS = 20
TOL_M = 4 * 0.02  # labelling tolerance: 4 sigma of the range noise (both simulators' sigma is 0.02 m)
RAW_TOPIC = "/livox/lidar"


# --- geometry -------------------------------------------------------------------------------------------------------

def distance_to(points, prim):
    """Unsigned distance of each point (N x 3) from a primitive's surface: box (size = half extents), cylinder (size =
    radius, half height; axis local z) or sphere (size = radius), posed by `pos` and `rot` (3 x 3, local to frame)."""
    local = (np.asarray(points, float) - np.asarray(prim["pos"], float)) @ np.asarray(prim["rot"], float)
    size = prim["size"]
    if prim["kind"] == "box":
        q = np.abs(local) - np.asarray(size[:3], float)
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
        inside = np.minimum(q.max(axis=1), 0.0)
        return np.abs(outside + inside)
    if prim["kind"] == "cylinder":
        dr = np.hypot(local[:, 0], local[:, 1]) - size[0]
        dz = np.abs(local[:, 2]) - size[1]
        outside = np.hypot(np.maximum(dr, 0.0), np.maximum(dz, 0.0))
        return np.abs(outside + np.minimum(np.maximum(dr, dz), 0.0))
    if prim["kind"] == "sphere":
        return np.abs(np.linalg.norm(local, axis=1) - size[0])
    raise ValueError(f"unknown primitive {prim['kind']}")


def bounding_radius(prim):
    s = prim["size"]
    return {"box": math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2), "cylinder": math.hypot(s[0], s[1]),
            "sphere": s[0]}[prim["kind"]]


def label_unity_points(points, boxes, robot_geoms, tol=TOL_M):
    """A class per point: the nearest of the ground plane (z = 0), the boxes and the robot geoms (each primitive's
    `label`) within `tol`, else "other". All in one frame."""
    points = np.asarray(points, float).reshape(-1, 3)
    best = np.abs(points[:, 2])
    labels = np.where(best <= tol, "ground", "other").astype(object)
    best = np.where(best <= tol, best, np.inf)
    for prim in list(boxes) + list(robot_geoms):
        near = np.linalg.norm(points - np.asarray(prim["pos"], float), axis=1) <= bounding_radius(prim) + tol
        if not near.any():
            continue
        d = distance_to(points[near], prim)
        idx = np.flatnonzero(near)
        better = (d <= tol) & (d < best[idx])
        best[idx[better]] = d[better]
        labels[idx[better]] = prim["label"]
    return labels.astype(str)


def class_fractions(labels):
    labels = np.asarray(labels)
    n = max(len(labels), 1)
    return {c: float((labels == c).sum()) / n for c in CLASSES}


# --- the comparison -------------------------------------------------------------------------------------------------

def _returns(pose_set):
    """Drop the misses (zero points): (lidar, base, labels) of the points that returned."""
    lidar = np.asarray(pose_set["lidar"], float).reshape(-1, 3)
    keep = np.linalg.norm(lidar, axis=1) > 1e-6
    return lidar[keep], np.asarray(pose_set["base"], float).reshape(-1, 3)[keep], np.asarray(pose_set["labels"])[keep]


def violations(lidar):
    r = np.linalg.norm(lidar, axis=1)
    elevation = np.degrees(np.arctan2(lidar[:, 2], np.hypot(lidar[:, 0], lidar[:, 1])))
    near = int((r < MIN_RANGE_M - 1e-6).sum())
    blind = int(((elevation < FOV_DEG[0] - FOV_TOL_DEG) | (elevation > FOV_DEG[1] + FOV_TOL_DEG)).sum())
    return near, blind


def voxels(base, labels):
    self_hits = base[np.isin(labels, SELF)]
    return set(map(tuple, np.floor(self_hits / VOXEL_M).astype(int)))


def compare(unity_scans, plant_scans):
    """Per pose present in both: {pose: {fractions (unity, plant), fraction_gap_pp (plant - unity), fractions_pass,
    self_hit_jaccard, min_range_violations, blind_zone_violations, pass}}. Each pose set holds `lidar` and `base`
    (N x 3, the lidar and base frames), `labels` (N) and `scans`."""
    out = {}
    for pose in unity_scans:
        if pose not in plant_scans:
            continue
        u_lidar, u_base, u_labels = _returns(unity_scans[pose])
        p_lidar, p_base, p_labels = _returns(plant_scans[pose])
        fu, fp = class_fractions(u_labels), class_fractions(p_labels)
        gap = {c: 100.0 * (fp[c] - fu[c]) for c in CLASSES}
        vu, vp = voxels(u_base, u_labels), voxels(p_base, p_labels)
        jaccard = len(vu & vp) / len(vu | vp) if vu | vp else 1.0
        (nu, bu), (np_, bp) = violations(u_lidar), violations(p_lidar)
        fractions_pass = all(abs(gap[c]) <= FRACTION_TOL_PP for c in GATED)
        row = dict(points={"unity": len(u_labels), "plant": len(p_labels)},
                   scans={"unity": unity_scans[pose].get("scans"), "plant": plant_scans[pose].get("scans")},
                   fractions={"unity": fu, "plant": fp}, fraction_gap_pp=gap, fractions_pass=fractions_pass,
                   self_hit_voxels={"unity": len(vu), "plant": len(vp)}, self_hit_jaccard=jaccard,
                   min_range_violations={"unity": nu, "plant": np_}, blind_zone_violations={"unity": bu, "plant": bp})
        row["pass"] = bool(fractions_pass and jaccard >= JACCARD_MIN and nu + np_ + bu + bp == 0)
        out[pose] = row
    return out


# --- the recordings -------------------------------------------------------------------------------------------------

def fixtures():
    return json.loads(FIXTURES.read_text())["fixtures"]


def record(out_dir, container=CONTAINER, seconds=3.0):
    """Host: each fixture in Unity (V2 scene, the arm in carry, the scenario's boxes): 3 s of raw /livox/lidar with TF
    and joint states into <out_dir>/<fixture>/bag, exported to unity.npz in the container; Play stopped at the end."""
    sys.path.insert(0, str(ROOT / "tools"))
    from run_nav_scenario import ARM_PROCESSES, NAV_PROCESSES, Runner, last_json, spec_command  # host only
    runner = Runner(container)
    scene = json.loads((ROOT / "ros2_ws/src/mobile_manipulator_navigation/maps/construction_site_v2.metadata.json")
                       .read_text())["sourceScene"]
    runner.stop("nav", NAV_PROCESSES)
    if runner.playing():
        runner.ensure_scene(scene, "construction_site_v2")
    else:
        runner.site = "construction_site_v2"
        runner.new_epoch(scene=scene)
    runner.unity("scenario_obstacle_clear")
    held = runner.qualified_pose() or runner.restore_qualified_pose_with_moveit(("home",))
    container_dir = f"{WORKSPACE}/{Path(out_dir).resolve().relative_to(ROOT)}"
    rows = []
    try:
        for f in fixtures():
            spec = last_json(runner.ros(spec_command(f["scenario"])), f"scenario_spec {f['scenario']}")["scenario"]
            runner.unity("scenario_obstacle_clear")
            x, y, yaw = f["pose"]
            for attempt in range(3):  # the hold must be fresh at the call
                try:
                    runner.wait_for_hold()
                    runner.unity("scenario_place", "--x", str(x), "--y", str(y), "--yaw", str(yaw), "--arm_pose", held)
                    break
                except RuntimeError:
                    if attempt == 2:
                        raise
                    time.sleep(2)
            runner.move_arm("carry")
            held = "carry"
            for o in spec.get("obstacles", []):
                runner.unity("scenario_obstacle", "--name", o["name"],
                             *(a for key in ("x", "y", "size_x", "size_y", "height", "z") if key in o for a in (f"--{key}", str(o[key]))))
            time.sleep(1.0)  # settled, the boxes in the scans
            bag = f"{container_dir}/{f['name']}/bag"
            runner.ros(f"rm -rf {bag} && mkdir -p {container_dir}/{f['name']} && timeout -s INT {seconds + 1.0} "
                       f"ros2 bag record -o {bag} {RAW_TOPIC} /tf /tf_static /joint_states > /tmp/mm_lidar_bag.log 2>&1; true",
                       timeout=seconds + 30)
            result = runner.ros(f"python3 {WORKSPACE}/tools/plant/compare_lidar.py export {container_dir}/{f['name']}",
                                timeout=120)
            rows.append(json.loads(result.stdout.strip().splitlines()[-1]))
            print(json.dumps(rows[-1]), flush=True)
            gx, gy, gyaw = rows[-1]["pose"]
            if math.hypot(gx - x, gy - y) > 0.02 or abs(math.remainder(gyaw - yaw, math.tau)) > math.radians(1.0):
                raise RuntimeError(f"{f['name']}: the robot is not at its fixture pose: {rows[-1]['pose']} vs {f['pose']}")
    finally:
        runner.unity("scenario_obstacle_clear")
        runner.stop("arm", ARM_PROCESSES)
        if runner.playing():
            runner.unity("editor_stop")  # the fanless host: never leave Play running
    return rows


def _matrix(t):
    x, y, z, w = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


def export(fixture_dir, scans=SCANS):
    """Container: <fixture_dir>/bag -> unity.npz: the last `scans` raw clouds (lidar frame, misses kept), the map ->
    base_footprint and base_footprint -> lidar transforms at the last cloud, the arm joints. Prints a JSON summary."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from sensor_msgs.msg import JointState, PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(fixture_dir) / "bag"), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    buffer, clouds, joints = Buffer(cache_time=rclpy_duration(60)), [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in ("/tf", "/tf_static"):
            for t in deserialize_message(data, TFMessage).transforms:
                (buffer.set_transform_static if topic == "/tf_static" else buffer.set_transform)(t, "bag")
        elif topic == RAW_TOPIC:
            clouds.append(deserialize_message(data, PointCloud2))
        elif topic == "/joint_states":
            joints = deserialize_message(data, JointState)
    if len(clouds) < scans:
        raise RuntimeError(f"{fixture_dir}: {len(clouds)} clouds, need {scans}")
    clouds = clouds[-scans:]
    frame = clouds[-1].header.frame_id
    stamp = Time.from_msg(clouds[-1].header.stamp)
    # Unity's odom is the map frame (its odom -> base_footprint is the placed map pose; map -> odom comes only with
    # navigation, which the runner stops)
    world = "map" if buffer.can_transform("map", "base_footprint", stamp) else "odom"
    base = _matrix(buffer.lookup_transform(world, "base_footprint", stamp).transform)
    try:  # the mount comes from robot_state_publisher's /tf_static, which the runner does not start
        lidar = _matrix(buffer.lookup_transform("base_footprint", frame, stamp).transform)
    except Exception:  # noqa: BLE001 (tf2 lookup errors): the comparison takes the URDF's mount (load_unity)
        lidar = np.full((4, 4), np.nan)
    points = [np.asarray(point_cloud2.read_points_numpy(c, field_names=("x", "y", "z")), dtype=np.float32) for c in clouds]
    q = [joints.position[list(joints.name).index(n)] for n in ARM_JOINTS] if joints else [math.nan] * 6
    np.savez_compressed(Path(fixture_dir) / "unity.npz", points=np.concatenate(points),
                        counts=np.array([len(p) for p in points]), base=base, lidar_in_base=lidar, q=np.array(q), frame=frame)
    yaw = math.atan2(base[1, 0], base[0, 0])
    print(json.dumps({"fixture": Path(fixture_dir).name, "scans": len(points), "points": int(sum(len(p) for p in points)),
                      "frame": frame, "world": world, "mount": "tf_static" if np.isfinite(lidar).all() else "urdf",
                      "pose": [float(base[0, 3]), float(base[1, 3]), yaw], "q": q}))


def rclpy_duration(seconds):
    from rclpy.duration import Duration
    return Duration(seconds=seconds)


ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def plant(out_dir, container=CONTAINER, scans=SCANS, seed=1):
    """Host -> container: plant_lidar_fixture at every fixture pose -> <out_dir>/<fixture>/plant.mmlog."""
    container_dir = f"{WORKSPACE}/{Path(out_dir).resolve().relative_to(ROOT)}"
    for f in fixtures():
        x, y, yaw = f["pose"]
        cmd = (f"source {WORKSPACE}/ros2_ws/install/setup.bash && mkdir -p {container_dir}/{f['name']} && "
               f"ros2 run mobile_manipulator_campaign plant_lidar_fixture --scenario {f['scenario']} --pose {x} {y} {yaw} "
               f"--scans {scans} --seed {seed} --out {container_dir}/{f['name']}/plant.mmlog")
        result = subprocess.run(["docker", "exec", "-w", WORKSPACE, container, "bash", "-c", cmd], capture_output=True, text=True)
        print(result.stdout.strip() or result.stderr.strip(), flush=True)
        if result.returncode:
            raise RuntimeError(f"plant_lidar_fixture failed for {f['name']}: {result.stderr}")


# --- the comparison of the recordings -------------------------------------------------------------------------------

def _posed(prim, transform):
    """A primitive moved by a 4 x 4 transform (world to world)."""
    rot = transform[:3, :3] @ np.asarray(prim["rot"], float)
    pos = transform[:3, :3] @ np.asarray(prim["pos"], float) + transform[:3, 3]
    return {**prim, "pos": pos.tolist(), "rot": rot.tolist()}


def _homogeneous(points, transform):
    return np.asarray(points, float) @ transform[:3, :3].T + transform[:3, 3]


def load_plant(path):
    """A plant.mmlog -> (header, pose set): its labels by the plant's own classes."""
    sys.path.insert(0, str(Path(__file__).parent))
    import plant_stream as ps
    log = ps.RunLog(path)
    lidar_in_base = np.asarray(log.header["lidar_in_base"], float)
    pts, labels, n = [], [], 0
    for kind, _, blocks in log.frames():
        if kind != ps.SCAN:
            continue
        p = np.asarray(blocks[0], float).reshape(-1, 3)
        pts.append(p)
        labels += [ps.HIT_CLASSES[int(c)] for c in blocks[1]]
        n += 1
    lidar = np.concatenate(pts)
    return log.header, dict(lidar=lidar, base=_homogeneous(lidar, lidar_in_base), labels=np.asarray(labels), scans=n)


def relabel(pose_set, header, base_world, lidar_in_base=None, tol=TOL_M):
    """A pose set labelled by proximity to the plant's primitives (label_unity_points): the static ones where they are,
    the robot's moved from the plant's base pose to `base_world` (this simulator's base pose); `lidar_in_base` default:
    the plant's. Both simulators' points are labelled so (the noise moves points off their surfaces alike); the labels
    the set had stay as `true_labels`."""
    lib = np.asarray(header["lidar_in_base"] if lidar_in_base is None else lidar_in_base, float)
    move = np.asarray(base_world, float) @ np.linalg.inv(np.asarray(header["base"], float))
    statics = [p for p in header["prims"] if p["label"] in ("site", "scenario")]
    robot = [_posed(p, move) for p in header["prims"] if p["label"] in SELF]
    lidar = np.asarray(pose_set["lidar"], float)
    world = _homogeneous(lidar, np.asarray(base_world, float) @ lib)
    keep = np.linalg.norm(lidar, axis=1) > 1e-6
    labels = np.full(len(lidar), "other", dtype=object)
    labels[keep] = label_unity_points(world[keep], statics, robot, tol)
    return {**pose_set, "labels": labels.astype(str), "true_labels": np.asarray(pose_set["labels"])}


def load_unity(path, header, tol=TOL_M):
    """A unity.npz -> pose set, Unity's points labelled against the plant's primitives (relabel at Unity's pose)."""
    d = np.load(path)
    lidar = d["points"].astype(float)
    mount = d["lidar_in_base"]
    if not np.isfinite(mount).all():  # no /tf_static in the bag: the URDF's mount, as the plant's header has it
        mount = np.asarray(header["lidar_in_base"], float)
    pose_set = dict(lidar=lidar, base=_homogeneous(lidar, mount), labels=np.full(len(lidar), "other"),
                    scans=len(d["counts"]))
    return relabel(pose_set, header, d["base"], mount, tol), d


def compare_dir(out_dir, report=None):
    """Every fixture in <out_dir> with both recordings: the comparison (JSON beside them) and, with `report`, lidar.md."""
    unity, plant_sets, extra = {}, {}, {}
    for f in fixtures():
        fd = Path(out_dir) / f["name"]
        if not (fd / "unity.npz").exists() or not (fd / "plant.mmlog").exists():
            print(f"{f['name']}: missing a recording, skipped", flush=True)
            continue
        header, truth = load_plant(fd / "plant.mmlog")
        plant_sets[f["name"]] = relabel(truth, header, header["base"])
        unity[f["name"]], d = load_unity(fd / "unity.npz", header)
        p = plant_sets[f["name"]]
        base_u = d["base"]
        extra[f["name"]] = {"pose_gap_m": float(np.hypot(*(base_u[:2, 3] - np.asarray(header["base"])[:2, 3]))),
                            "arm_gap_rad": float(np.nanmax(np.abs(d["q"] - np.asarray(header["q"])))),
                            "plant_label_agreement": float((p["labels"] == p["true_labels"]).mean()),
                            "lidar_mount_gap_m": float(np.linalg.norm(d["lidar_in_base"][:3, 3] - np.asarray(header["lidar_in_base"])[:3, 3]))}
    rows = compare(unity, plant_sets)
    for name in rows:
        rows[name].update(extra[name])
    (Path(out_dir) / "comparison.json").write_text(json.dumps(rows, indent=1) + "\n")
    if report:
        write_report(rows, out_dir, report)
    return rows


def write_report(rows, out_dir, path):
    lines = ["# Gate L against Unity: the plant's lidar at fixed poses", "",
             f"Generated by `tools/plant/compare_lidar.py compare` from `{Path(out_dir).resolve().relative_to(ROOT)}` (MuJoCo plant plan "
             "Task 10). Unity: 20 raw `/livox/lidar` scans per pose (before the robot self-filter); the plant: 20 scans of "
             "`plant_lidar_fixture` (realistic mode, seed 1) at the same pose. Both simulators' points are labelled alike: "
             f"the nearest plant primitive within {100 * TOL_M:.0f} cm (4 sigma of the range noise), the robot's posed at "
             "each simulator's base pose. The plant's own ground-truth labels check that labelling (agreement column).", "",
             f"Pass (spec Section 7, L): every class fraction within +-{FRACTION_TOL_PP:.0f} percentage points, the self-hit "
             f"voxel Jaccard (5 cm, base frame) >= {JACCARD_MIN} (to confirm at CP4), no point nearer than "
             f"{MIN_RANGE_M} m or outside the pattern's {FOV_DEG[0]:.2f} to {FOV_DEG[1]:.2f} deg.", "",
             "| Pose | Points U / P | " + " | ".join(f"{c} U / P (pp gap)" for c in GATED) + " | other U / P | Self-hit Jaccard "
             "| Range / FOV violations U, P | Pose, arm gap | Plant label agreement | Pass |",
             "|---|---|" + "---|" * len(GATED) + "---|---|---|---|---|---|"]
    for name, r in rows.items():
        fu, fp = r["fractions"]["unity"], r["fractions"]["plant"]
        cells = [f"{100 * fu[c]:.1f} / {100 * fp[c]:.1f} ({r['fraction_gap_pp'][c]:+.1f})" for c in GATED]
        mr, bz = r["min_range_violations"], r["blind_zone_violations"]
        lines.append(f"| {name} | {r['points']['unity']} / {r['points']['plant']} | " + " | ".join(cells)
                     + f" | {100 * fu['other']:.1f} / {100 * fp['other']:.1f} | {r['self_hit_jaccard']:.2f} | "
                     f"{mr['unity'] + bz['unity']}, {mr['plant'] + bz['plant']} | {1000 * r['pose_gap_m']:.1f} mm, "
                     f"{1000 * r['arm_gap_rad']:.1f} mrad | {100 * r['plant_label_agreement']:.1f} % | {'yes' if r['pass'] else 'NO'} |")
    notes = Path(path).with_name("lidar_notes.md")
    if notes.exists():
        lines += ["", notes.read_text().rstrip()]
    Path(path).write_text("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("record", "plant"):
        s = sub.add_parser(name)
        s.add_argument("--out", required=True)
        s.add_argument("--container", default=CONTAINER)
    s = sub.add_parser("export")
    s.add_argument("fixture_dir")
    s = sub.add_parser("compare")
    s.add_argument("out")
    s.add_argument("--report")
    args = ap.parse_args()
    if args.cmd == "record":
        record(args.out, args.container)
    elif args.cmd == "plant":
        plant(args.out, args.container)
    elif args.cmd == "export":
        export(args.fixture_dir)
    else:
        rows = compare_dir(args.out, args.report)
        for name, r in rows.items():
            print(name, "pass" if r["pass"] else "FAIL", {c: round(r["fraction_gap_pp"][c], 1) for c in GATED},
                  "jaccard", round(r["self_hit_jaccard"], 2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
