"""Process handling of tools/run_nav_scenario.py's contact watch and the B5 runner decisions, without Docker or Unity.

Run: python3 -m pytest tools/test_run_nav_scenario.py
"""
import argparse
import subprocess
import sys
import types

import pytest

sys.modules.setdefault("analyze_arm_qualification", types.SimpleNamespace(analyze=None))
import run_nav_scenario as rns  # noqa: E402

REAL_POPEN = subprocess.Popen


class FakeRunner:
    container = "unused"

    def __init__(self, contact_after=None):
        self.polls, self.cancels, self.contact_after = 0, 0, contact_after

    def unity(self, command):
        self.polls += 1
        if self.contact_after is None:
            raise subprocess.TimeoutExpired(command, 1)  # not a RuntimeError
        return {"contact": self.polls >= self.contact_after}

    def ros(self, *args, **kwargs):
        self.cancels += 1


def local(command):
    """Replace the docker exec with a local command."""
    return lambda args, **kwargs: REAL_POPEN(["bash", "-c", command], **kwargs)


def test_large_output_and_unity_errors_do_not_stall(monkeypatch):
    monkeypatch.setattr(rns.subprocess, "Popen", local("head -c 200000 /dev/zero | tr '\\0' x; exit 3"))
    result, canceled = rns.run_watching_contacts(FakeRunner(), "task", timeout=30)
    assert result.returncode == 3 and len(result.stdout) == 200000 and canceled is None


def test_contact_cancels_once(monkeypatch):
    monkeypatch.setattr(rns.subprocess, "Popen", local("sleep 3"))
    runner = FakeRunner(contact_after=1)
    _, canceled = rns.run_watching_contacts(runner, "task", timeout=30)
    assert canceled is not None and runner.cancels == 1


def test_timeout_kills_the_task(monkeypatch):
    started = []
    monkeypatch.setattr(rns.subprocess, "Popen",
                        lambda args, **kwargs: started.append(REAL_POPEN(["sleep", "30"], **kwargs)) or started[-1])
    with pytest.raises(RuntimeError, match="did not finish"):
        rns.run_watching_contacts(FakeRunner(), "task", timeout=2)
    assert started[0].poll() is not None

def args(**changes):
    return argparse.Namespace(**{"controller": "rpp", "footprint_mode": None, "footprint_profile": None, **changes})


def test_wholebody_drives_navigate_and_mission_scenarios_in_the_wholebody_footprint_mode():
    for task in ("navigate_to_pose", "mission"):
        assert rns.footprint_mode(args(controller="wholebody"), task) == "wholebody"
    assert rns.footprint_mode(args(controller="wholebody", footprint_mode="wholebody"), "mission") == "wholebody"


def test_the_wholebody_footprint_mode_belongs_to_wholebody_alone():
    with pytest.raises(RuntimeError, match="wholebody"):
        rns.footprint_mode(args(footprint_mode="wholebody"), "navigate_to_pose")
    with pytest.raises(RuntimeError, match="wholebody"):
        rns.footprint_mode(args(controller="wholebody", footprint_mode="dynamic"), "navigate_to_pose")


def test_baselines_keep_their_footprint_modes():
    assert rns.footprint_mode(args(), "navigate_to_pose") == "static"
    assert rns.footprint_mode(args(), "mission") == "profiles"
    assert rns.footprint_mode(args(footprint_mode="dynamic"), "mission") == "dynamic"


def test_footprint_wait_for_wholebody_reads_the_base_only_footprint_on_the_global_costmap():
    command = rns.footprint_wait_command("wholebody", True, "home")
    assert "--mode dynamic" in command and "--global-only" in command and "--zones" not in command
    assert "--mode static --profile home" in rns.footprint_wait_command("static", True, "home")
    assert "--global-only" in rns.footprint_wait_command("static", False, "home")
    assert "--mode dynamic --zones" in rns.footprint_wait_command("dynamic", True, "home")


def test_wholebody_is_a_controller_and_its_bag_records_the_mpc_topics():
    assert "wholebody" in rns.CONTROLLERS
    # The MPC streams the arm in every B5 run: the JTC state gives its tracking (path) error.
    assert rns.WHOLEBODY_BAG_TOPICS == ["/wholebody_mpc/cycle", "/wholebody_mpc/plan",
                                        "/arm_controller/controller_state", "/arm_controller/joint_trajectory"]
    assert {"/tf", "/tf_static", "/joint_states"} <= set(rns.BAG_TOPICS)


def test_arm_pose_accepts_the_qualified_poses_only():
    parser = rns.build_parser()
    assert parser.parse_args(["overhead_course_nav", "--arm-pose", "home"]).arm_pose == "home"
    assert parser.parse_args(["overhead_course_nav"]).arm_pose is None
    for name in ("base_only", "level_extension", "no_such_pose"):
        with pytest.raises(SystemExit):
            parser.parse_args(["overhead_course_nav", "--arm-pose", name])


def test_arm_pose_resolves_the_scenario_in_that_pose_and_its_profile():
    # scenario_spec --arm-pose replaces arm_pose and footprint_profile and checks the start, goal and
    # obstacles for that profile, so the whole run (teleport, Nav2 footprint, metrics) uses it.
    assert rns.spec_command("overhead_course_nav", "home") == (
        "ros2 run mobile_manipulator_navigation scenario_spec overhead_course_nav --arm-pose home")
    assert rns.spec_command("overhead_course_nav", None) == (
        "ros2 run mobile_manipulator_navigation scenario_spec overhead_course_nav")
    assert rns.spec_command("x", "home", start_free_for="vertical_carry").endswith(
        "scenario_spec x --arm-pose home --start-free-for vertical_carry")


def test_arm_pose_overrides_for_any_controller_but_not_b3_b4_mission_steps():
    for controller in rns.CONTROLLERS:
        rns.check_arm_pose(args(controller=controller, arm_pose="home"), mission_steps=False)
    rns.check_arm_pose(args(arm_pose=None), mission_steps=True)
    # mission_scenario_task and ReconfigurePanel start from the scenario's own pose and profile.
    with pytest.raises(RuntimeError, match="--arm-pose"):
        rns.check_arm_pose(args(arm_pose="home"), mission_steps=True)


def test_no_slowdown_zone_is_a_flag_for_every_controller_and_on_by_default():
    parser = rns.build_parser()
    assert parser.parse_args(["narrow_gate_home_nav"]).slowdown_zone is True
    for controller in rns.CONTROLLERS:
        args = parser.parse_args(["narrow_gate_home_nav", "--controller", controller, "--no-slowdown-zone"])
        assert args.slowdown_zone is False


def test_mpc_options_need_wholebody_and_name_their_suffix():
    parser = rns.build_parser()
    plain = parser.parse_args(["overhead_course_nav", "--controller", "wholebody"])
    assert rns.mpc_overrides(plain, wholebody=True) == {} and rns.mpc_suffix({}) == ""
    given = parser.parse_args(["overhead_course_nav", "--controller", "wholebody", "--posture-mode", "comfort_only",
                               "--comfort-pose", "home"])
    overrides = rns.mpc_overrides(given, wholebody=True)
    assert overrides == {"posture_mode": "comfort_only", "comfort_pose": "home"}
    assert rns.mpc_suffix(overrides) == "-posturecomfortonly-comforthome"
    with pytest.raises(RuntimeError, match="--controller wholebody"):
        rns.mpc_overrides(parser.parse_args(["narrow_gate_home_nav", "--posture-mode", "none"]), wholebody=False)
    for name in ("vertical_carry", "bent"):
        with pytest.raises(SystemExit):
            parser.parse_args(["narrow_gate_home_nav", "--comfort-pose", name])


def test_a_comfort_pose_missing_from_the_payload_is_refused_by_name():
    poses = rns.json.loads(rns.QUALIFIED_PAYLOAD.read_text())["poses_rad"]
    args = rns.build_parser().parse_args(["x", "--controller", "wholebody", "--comfort-pose", "carry"])
    if "carry" in poses:
        assert rns.mpc_overrides(args, wholebody=True) == {"comfort_pose": "carry"}
    else:
        with pytest.raises(RuntimeError, match="'carry'"):
            rns.mpc_overrides(args, wholebody=True)


def test_clearance_weight_is_a_wholebody_option_with_a_suffix():
    parser = rns.build_parser()
    args = parser.parse_args(["narrow_gate_home_nav", "--controller", "wholebody", "--posture-mode", "comfort_only",
                              "--clearance-weight", "20", "--no-slowdown-zone"])
    overrides = rns.mpc_overrides(args, wholebody=True)
    assert overrides == {"posture_mode": "comfort_only", "clearance_weight": 20.0}
    assert rns.mpc_suffix(overrides) == "-posturecomfortonly-clear20"
    assert rns.mpc_suffix({"clearance_weight": 2.5}) == "-clear2.5"
    with pytest.raises(RuntimeError, match="clearance-weight"):
        rns.mpc_overrides(parser.parse_args(["x", "--controller", "wholebody", "--clearance-weight", "-1"]), True)
