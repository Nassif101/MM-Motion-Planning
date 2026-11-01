#!/usr/bin/env python3
"""The medium layer's per-run statistics from a live run's bag (Phase 5A campaigns, plan Task 13 Step 3).

For each run directory (tools/run_nav_scenario.py's experiment_runs/<run>), over the task's window (summary.json
task.started_s .. finished_s, sim time):
  - the medium cycles (/medium/cycle): count, total_s statistics (medium_bench.statistics, nearest rank), capped
    cycles, full multi-starts;
  - the published plans (/medium/plan): count, statuses, and their swing sides as the closed-loop bench counts them
    (mobile_manipulator_medium::SideTracker: a label without a side, comfort or lowered, keeps the last side, so
    right -> lowered -> left is a switch): the first side, the sequence and the switches after the first;
  - the static gravity torque of the measured arm (/joint_states) by the model the medium layer constrains
    (generate_model.static_torque, the CasADi function the planner's GeneratedStaticTorque is generated from): per
    joint the peak |torque| and its ratio to medium.yaml's torque_limits_nm (the run's medium config, else the
    installed default; 60 % of the URDF efforts).
Writes <run>/medium_stats.json. B5-only runs get the torque (the comparison) and empty medium fields.

Runs in the container (rosbag2_py, CasADi, the installed shares):

  python3 tools/medium_run_stats.py experiment_runs/<run> [...]
"""
import argparse
import json
import math
import sys
from pathlib import Path

from medium_bench import statistics

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "ros2_ws" / "src" / "mobile_manipulator_mpc" / "scripts"


def side_of(homotopy):
    """The swing side of a homotopy label ("left", "right" or "")."""
    for side in ("left", "right"):
        if homotopy.startswith(side):
            return side
    return ""


class SideTracker:
    """As mobile_manipulator_medium::SideTracker."""

    def __init__(self):
        self.first, self.last, self.sequence, self.switches = "", "", [], 0

    def observe(self, homotopy):
        side = side_of(homotopy)
        if not side or side == self.last:
            return
        if not self.first:
            self.first = side
        else:
            self.switches += 1
        self.last = side
        self.sequence.append(side)


def torque_peaks(samples, torque, limits):
    """Per joint the peak |torque| over `samples` (arm joint vectors in the model's order) and its ratio to `limits`
    (None: no limit); `torque` maps a joint vector to the six torques (N m)."""
    peaks = [0.0] * 6
    for q in samples:
        for j, t in enumerate(torque(q)):
            peaks[j] = max(peaks[j], abs(t))
    ratios = [None if limit is None else peak / limit for peak, limit in zip(peaks, limits)]
    return {"peak_nm": peaks, "limit_nm": list(limits), "ratio": ratios}


def medium_stats(cycles, plans, arm, torque, limits):
    """The statistics from the window's messages: `cycles` (total_s, capped, full_multistart), `plans` (status,
    homotopy) in time order, `arm` the measured arm joint vectors."""
    sides = SideTracker()
    statuses = {}
    for plan in plans:
        sides.observe(plan["homotopy"])
        statuses[plan["status"]] = statuses.get(plan["status"], 0) + 1
    return {
        "cycles": len(cycles),
        "total_s": statistics([c["total_s"] for c in cycles]),
        "capped_cycles": sum(1 for c in cycles if c["capped"]),
        "full_multistarts": sum(1 for c in cycles if c["full_multistart"]),
        "plans": len(plans),
        "plan_statuses": statuses,
        "first_side": sides.first,
        "side_sequence": sides.sequence,
        "homotopy_switches_after_first": sides.switches,
        "torque": torque_peaks(arm, torque, limits) if arm else None,
        "arm_samples": len(arm),
    }


def read_bag(bag, window, joint_order):
    """The window's /medium/cycle, /medium/plan and the /joint_states arm vectors (model order), by header stamp."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in ("/medium/cycle", "/medium/plan", "/joint_states") if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    cycles, plans, arm = [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if not window[0] <= stamp <= window[1]:
            continue
        if topic == "/medium/cycle":
            cycles.append({"total_s": msg.total_s, "capped": msg.capped, "full_multistart": msg.full_multistart})
        elif topic == "/medium/plan":
            plans.append({"status": msg.status, "homotopy": msg.homotopy})
        else:
            index = {name: i for i, name in enumerate(msg.name)}
            if all(name in index for name in joint_order):
                arm.append([msg.position[index[name]] for name in joint_order])
    return cycles, plans, arm


def static_torque_function():
    """The model's static torque (q -> six N m) and the joint order, from the installed description and payload."""
    from ament_index_python.packages import get_package_share_directory
    from urdf_parser_py.urdf import URDF
    sys.path.insert(0, str(GENERATOR))
    import generate_model

    urdf = Path(get_package_share_directory("mobile_manipulator_description")) / "urdf" / "mobile_manipulator.urdf"
    payload = json.loads((Path(get_package_share_directory("mobile_manipulator_control")) / "config" /
                          "qualified_payload.json").read_text())
    robot = URDF.from_xml_string(urdf.read_text())
    function = generate_model.static_torque(robot, payload["joint_order"], payload["payload"], "tool0")
    return (lambda q: [float(v) for v in function(q)[0].full().ravel()]), payload["joint_order"]


def torque_limits(medium_record):
    """medium.yaml's torque_limits_nm: the run's medium config (repository-relative), else the installed default."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    config = (medium_record or {}).get("config_file")
    path = ROOT / config if config else (Path(get_package_share_directory("mobile_manipulator_medium")) / "config" /
                                         "medium.yaml")
    return yaml.safe_load(path.read_text())["torque_limits_nm"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", help="run directories")
    args = parser.parse_args(argv)
    torque, joint_order = static_torque_function()
    for run in map(Path, args.runs):
        summary = json.loads((run / "summary.json").read_text())
        task = summary["task"]
        window = (task.get("started_s", -math.inf), task.get("finished_s", math.inf))
        cycles, plans, arm = read_bag(run / "bag", window, joint_order)
        stats = medium_stats(cycles, plans, arm, torque, torque_limits(summary.get("medium")))
        (run / "medium_stats.json").write_text(json.dumps(stats, indent=1) + "\n")
        print(f"{run.name}: {stats['cycles']} cycles, {stats['plans']} plans, sides {stats['side_sequence']}, "
              f"torque ratio {stats['torque'] and stats['torque']['ratio']}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
