"""Gate V: the plant's scene beside Unity's at the same poses (MuJoCo plant plan Task 12; spec Section 8).

At three poses (the 1.05 m door, the 1.23 m beam, the worker crossing; the arm in carry) Unity renders a chase camera
(`scene_capture`) with the robot placed and the scenario's boxes and worker, and MuJoCo renders the plant's scene (the
site, the planar robot model, the scenario's boxes and movers, from a replica run's Hello) from the same camera. The
pairs go to OUT/<pose>-unity.png and OUT/<pose>-mujoco.png, and OUT/index.html shows them side by side.

    .venv-plant/bin/python tools/plant/visual_check.py --out experiment_runs/visual-check-DATE [--mujoco-only]
      (host; the Unity Editor up, the container up, no timed job; Play is stopped at the end)
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(ROOT / "tools"))
CARRY = [0.0, 0.968, -1.735, 0.0, 0.767, 0.0]  # the SRDF's carry
REPLICAS = ROOT / "experiment_runs/plant-replicas"
WIDTH, HEIGHT, FOV = 1600, 900, 60.0
POSES = [
    {"name": "door_105", "scenario": "v2_door_105_nav", "pose": [16.275, 4.3, 1.570796], "back": 3.5, "up": 2.4, "pitch": 22.0},
    {"name": "low_beam_123", "scenario": "v2_low_beam_123_nav", "pose": [-1.6, -5.5, 0.0], "back": 3.5, "up": 2.4, "pitch": 18.0},
    {"name": "worker_crossing", "scenario": "v2_worker_crossing_nav", "pose": [-17.25, 0.0, 0.0], "back": 3.5, "up": 2.6, "pitch": 18.0},
]


def camera(p):
    """The chase camera: `back` metres behind the base, `up` high, looking along the robot's heading, pitched down."""
    x, y, yaw = p["pose"]
    return {"x": x - p["back"] * math.cos(yaw), "y": y - p["back"] * math.sin(yaw), "z": p["up"],
            "yaw_deg": math.degrees(yaw), "pitch_deg": p["pitch"]}


def free_camera(cam):
    """MuJoCo's free camera (lookat, distance, azimuth, elevation) for a position and a heading pitched down."""
    yaw, pitch = math.radians(cam["yaw_deg"]), math.radians(cam["pitch_deg"])
    forward = (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch))
    distance = 5.0
    lookat = [cam["x"] + distance * forward[0], cam["y"] + distance * forward[1], cam["z"] + distance * forward[2]]
    return {"lookat": lookat, "distance": distance, "azimuth": cam["yaw_deg"], "elevation": -cam["pitch_deg"]}


def hello_of(scenario):
    """A replica run's session Hello for the scenario (its boxes and movers) and its movers' start positions (the first
    State frame's)."""
    import plant_stream as ps
    log = next(iter(sorted(REPLICAS.glob(f"{scenario}-*-realistic-s1/run.mmlog"))), None)
    if log is None:
        raise FileNotFoundError(f"no replica run of {scenario} in {REPLICAS}")
    run = ps.RunLog(log)
    movers = next((h.get("movers", []) for kind, h, _ in run.frames() if kind == ps.STATE), [])
    return run.header, movers


def write_png(path, rgb):
    """An 8-bit RGB array (H x W x 3) as a PNG, standard library only."""
    import struct
    import zlib
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[r].tobytes() for r in range(h))
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    Path(path).write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def render_mujoco(p, path):
    import mujoco
    import plant_stream as ps
    import view_run as vr
    hello, movers = hello_of(p["scenario"])
    scene = vr.Scene(hello)
    scene.model.vis.global_.offwidth, scene.model.vis.global_.offheight = WIDTH, HEIGHT
    scene.model.vis.global_.fovy = FOV
    scene.apply(ps.STATE, {"t": 0.0, "movers": movers}, [list(p["pose"]) + CARRY, [], []])
    renderer = mujoco.Renderer(scene.model, HEIGHT, WIDTH)
    cam = mujoco.MjvCamera()
    fc = free_camera(camera(p))
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = fc["lookat"]
    cam.distance, cam.azimuth, cam.elevation = fc["distance"], fc["azimuth"], fc["elevation"]
    renderer.update_scene(scene.data, cam)
    write_png(path, renderer.render())


def capture_unity(poses, out_dir, container):
    from run_nav_scenario import ARM_PROCESSES, NAV_PROCESSES, Runner, last_json, spec_command  # host only
    runner = Runner(container)
    scene = json.loads((ROOT / "ros2_ws/src/mobile_manipulator_navigation/maps/construction_site_v2.metadata.json")
                       .read_text())["sourceScene"]
    try:
        runner.stop("nav", NAV_PROCESSES)
        if runner.playing():
            runner.ensure_scene(scene, "construction_site_v2")
        else:
            runner.site = "construction_site_v2"
            runner.new_epoch(scene=scene)
        held = runner.qualified_pose() or runner.restore_qualified_pose_with_moveit(("home",))
        for p in poses:
            spec = last_json(runner.ros(spec_command(p["scenario"])), f"scenario_spec {p['scenario']}")["scenario"]
            runner.unity("scenario_obstacle_clear")
            x, y, yaw = p["pose"]
            runner.wait_for_hold()
            runner.unity("scenario_place", "--x", str(x), "--y", str(y), "--yaw", str(yaw), "--arm_pose", held)
            runner.move_arm("carry")
            held = "carry"
            for o in spec.get("obstacles", []):
                runner.unity("scenario_obstacle", "--name", o["name"],
                             *(a for key in ("x", "y", "size_x", "size_y", "height", "z") if key in o for a in (f"--{key}", str(o[key]))))
            for m in spec.get("movers", []):  # placed at their start; a far robot does not trigger them
                runner.unity("scenario_mover", "--name", m["name"], "--start_x", str(m["start"][0]), "--start_y", str(m["start"][1]),
                             "--end_x", str(m["end"][0]), "--end_y", str(m["end"][1]), "--size_x", str(m["size_x"]),
                             "--size_y", str(m["size_y"]), "--height", str(m["height"]), "--speed", str(m["speed_mps"]),
                             "--trigger_distance", str(m["trigger_distance_m"]), "--crossings", str(m["crossings"]))
            time.sleep(1.0)
            c = camera(p)
            runner.unity("scene_capture", "--output", str((Path(out_dir) / f"{p['name']}-unity.png").resolve().relative_to(ROOT)),
                         "--x", f"{c['x']:.4f}", "--y", f"{c['y']:.4f}", "--z", f"{c['z']:.4f}", "--yaw", f"{c['yaw_deg']:.3f}",
                         "--pitch", f"{c['pitch_deg']:.3f}", "--fov", str(FOV), "--width", str(WIDTH), "--height", str(HEIGHT),
                         timeout=120)
            print(f"{p['name']}: Unity captured", flush=True)
    finally:
        for step in (lambda: runner.unity("scenario_obstacle_clear"), lambda: runner.stop("arm", ARM_PROCESSES),
                     lambda: runner.unity("editor_stop") if runner.playing() else None):
            try:
                step()
            except Exception as error:  # noqa: BLE001 (every cleanup step runs; Play must stop on the fanless host)
                print(f"cleanup: {error}", flush=True)


def index(out_dir, poses):
    rows = "".join(f"<h2>{p['name']} ({p['scenario']}, base {p['pose']})</h2><div class=row>"
                   f"<figure><img src='{p['name']}-unity.png'><figcaption>Unity</figcaption></figure>"
                   f"<figure><img src='{p['name']}-mujoco.png'><figcaption>MuJoCo plant</figcaption></figure></div>"
                   for p in poses)
    (Path(out_dir) / "index.html").write_text(
        "<!doctype html><meta charset=utf-8><title>Plant visual check</title><style>body{font-family:sans-serif;margin:16px}"
        ".row{display:flex;gap:12px;flex-wrap:wrap}figure{margin:0;flex:1 1 480px}img{width:100%}</style>"
        "<h1>Gate V: the plant beside Unity</h1>" + rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mujoco-only", action="store_true")
    ap.add_argument("--container", default="mm-motion-planning-ma-robot-sim-1")
    args = ap.parse_args()
    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    for p in POSES:
        render_mujoco(p, out / f"{p['name']}-mujoco.png")
        print(f"{p['name']}: MuJoCo rendered", flush=True)
    if not args.mujoco_only:
        capture_unity(POSES, out, args.container)
    index(out, POSES)
    print(f"open {out / 'index.html'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
