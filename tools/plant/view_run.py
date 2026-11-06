"""Show a MuJoCo plant run in MuJoCo's viewer (MuJoCo plant plan Task 9). Runs on the host (.venv-plant: mujoco).

    .venv-plant/bin/mjpython tools/plant/view_run.py RUN.mmlog [--speed 1.0]   (macOS: mjpython for the viewer)
    .venv-plant/bin/mjpython tools/plant/view_run.py --live localhost:7700 --scenario v2_door_105_nav [--goal x y yaw]
        [--seed N] [--speed 1.0]
    .venv-plant/bin/python tools/plant/view_run.py RUN.mmlog --check     # no window: build the scene, apply every frame

The scene is the committed site model, the committed planar robot model (the true pose and joints of each State
frame), the run's scenario boxes and movers (from its Hello frame), the latest medium plan (orange spheres) and the
latest lidar scan (every n-th point, coloured by what it hit). The demo server protocol: stream-protocol.md.
"""
import argparse
import math
import sys
import time
import warnings
from pathlib import Path

import numpy as np

import plant_stream as ps

ROOT = Path(__file__).resolve().parents[2]
SITE = ROOT / "ros2_ws/src/mobile_manipulator_plant/models/site_v2.xml"
ROBOT = ROOT / "ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/robot.xml"
ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
CLASS_RGBA = {0: (0.5, 0.5, 0.5, 1), 1: (0.2, 0.6, 1, 1), 2: (1, 0.3, 0.3, 1), 3: (1, 0.8, 0.2, 1), 4: (0.3, 1, 0.3, 1),
              5: (1, 0.3, 1, 1), 6: (1, 1, 1, 1)}
SCAN_EVERY = 20


def build_scene(hello):
    """The site, the robot, the run's own boxes (those the site doesn't have) and its movers as mocap boxes."""
    import mujoco
    site = mujoco.MjSpec.from_file(str(SITE))
    robot = mujoco.MjSpec.from_file(str(ROBOT))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        site.worldbody.add_frame().attach_body(robot.body("base_footprint"), "", "")
    site_geoms = {g.name for g in site.geoms}
    for b in hello.get("boxes", []):
        if f"site/{b['name']}" in site_geoms:
            continue
        yaw = b["yaw"]
        site.worldbody.add_geom(name=f"scenario/{b['name']}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=b["center"],
                                size=[s / 2 for s in b["size"]], quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)],
                                rgba=[0.9, 0.5, 0.2, 0.6], contype=0, conaffinity=0)
    for mover in hello.get("movers", []):
        body = site.worldbody.add_body(name=f"mover/{mover['name']}", mocap=True)
        sx, sy, h = mover["size"]
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[sx / 2, sy / 2, h / 2], pos=[0, 0, h / 2],
                      rgba=[0.9, 0.2, 0.9, 0.8], contype=0, conaffinity=0)
    model = site.compile()
    return model, mujoco.MjData(model)


class Scene:
    def __init__(self, hello):
        import mujoco
        self.mj = mujoco
        self.model, self.data = build_scene(hello)
        self.qadr = [self.model.joint(n).qposadr[0] for n in ["base_x", "base_y", "base_yaw"] + ARM]
        self.movers = {m["name"]: self.model.body(f"mover/{m['name']}").mocapid[0] for m in hello.get("movers", [])}
        self.plan, self.scan, self.t = [], None, 0.0
        self.events = []

    def apply(self, kind, header, blocks):
        """One frame: a State moves the robot and the movers; Plans and Scans replace the drawn plan and scan."""
        if kind == ps.STATE:
            self.t = header["t"]
            for adr, value in zip(self.qadr, blocks[0]):
                self.data.qpos[adr] = value
            for name, x, y in header.get("movers", []):
                if name in self.movers:
                    self.data.mocap_pos[self.movers[name]][:2] = (x, y)
            self.mj.mj_forward(self.model, self.data)
        elif kind == ps.PLANS and blocks and blocks[0]:
            self.plan = np.asarray(blocks[0]).reshape(-1, 9)[:, :3]
        elif kind == ps.SCAN and blocks:
            points = np.asarray(blocks[0]).reshape(-1, 3)[::SCAN_EVERY]
            classes = np.asarray(blocks[1])[::SCAN_EVERY] if len(blocks) > 1 else np.zeros(len(points))
            site = self.data.site("livox_frame")
            world = site.xpos + points @ site.xmat.reshape(3, 3).T  # the pose now (the scan's own times are close)
            self.scan = (world, classes)
        elif kind == ps.EVENT:
            self.events.append(header)
            print(f"t {header.get('t', 0):7.2f}  {header['event']}  "
                  + ", ".join(f"{k}={v}" for k, v in header.items() if k not in ("event", "t")), flush=True)

    def draw(self, viewer):
        scn = viewer.user_scn
        scn.ngeom = 0
        eye = np.eye(3).flatten()
        def add(pos, size, rgba):
            if scn.ngeom < scn.maxgeom:
                self.mj.mjv_initGeom(scn.geoms[scn.ngeom], self.mj.mjtGeom.mjGEOM_SPHERE, [size, 0, 0], pos, eye, rgba)
                scn.ngeom += 1
        for x, y, _ in self.plan:
            add([x, y, 0.05], 0.04, [1, 0.55, 0, 0.9])
        if self.scan is not None:
            for p, c in zip(*self.scan):
                add(p, 0.025, CLASS_RGBA.get(int(c), (1, 1, 1, 1)))


def pace(t0_wall, t0_sim, t_sim, speed):
    if speed > 0:
        wait = t0_wall + (t_sim - t0_sim) / speed - time.monotonic()
        if wait > 0:
            time.sleep(wait)


def replay(path, speed, check):
    log = ps.RunLog(path)
    scene = Scene(log.header)
    print(f"{log.header.get('scenario')} seed {log.header.get('seed')} ({log.header.get('mode')})", flush=True)
    if check:
        n = sum(1 for kind, header, blocks in log.frames() if scene.apply(kind, header, blocks) is None)
        print(f"applied {n} frames; robot at ({scene.data.qpos[scene.qadr[0]]:.2f}, {scene.data.qpos[scene.qadr[1]]:.2f})")
        return 0
    import mujoco.viewer
    with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
        t0_wall, t0_sim = time.monotonic(), None
        for kind, header, blocks in log.frames():
            if not viewer.is_running():
                break
            scene.apply(kind, header, blocks)
            if kind == ps.STATE:
                t0_sim = header["t"] if t0_sim is None else t0_sim
                pace(t0_wall, t0_sim, header["t"], speed)
                scene.draw(viewer)
                viewer.sync()
        while viewer.is_running():
            time.sleep(0.1)
    return 0


def live(address, scenario, goal, seed, speed):
    host, port = address.rsplit(":", 1)
    client = ps.Client(host, int(port))
    print(f"connected: {client.hello.get('server')} ({client.hello.get('mode')}), {len(client.hello.get('scenarios', []))} scenarios")
    client.command("reset", scenario=scenario, seed=seed)
    kind, hello, _ = next(f for f in iter(client.next, None) if f[0] == ps.HELLO)
    scene = Scene(hello)
    if goal:
        client.command("set_goal", goal=goal)
    client.command("run", speed=speed)
    import mujoco.viewer
    with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
        done = False
        while viewer.is_running():
            frame = client.next(timeout=0.05)
            if frame is not None:
                scene.apply(*frame)
                done = done or (frame[0] == ps.EVENT and frame[1]["event"] in ("goal_reached", "failed", "timeout"))
                if frame[0] != ps.STATE:
                    continue
            scene.draw(viewer)
            viewer.sync()
    client.close()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?")
    ap.add_argument("--live")
    ap.add_argument("--scenario")
    ap.add_argument("--goal", type=float, nargs=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    if args.live:
        if not args.scenario:
            ap.error("--live needs --scenario")
        return live(args.live, args.scenario, args.goal, args.seed, args.speed)
    if not args.log:
        ap.error("a run log or --live")
    return replay(args.log, args.speed, args.check)


if __name__ == "__main__":
    sys.exit(main())
