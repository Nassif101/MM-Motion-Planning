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
    assert rns.WHOLEBODY_BAG_TOPICS[:4] == ["/wholebody_mpc/cycle", "/wholebody_mpc/plan",
                                            "/arm_controller/controller_state", "/arm_controller/joint_trajectory"]
    assert {"/tf", "/tf_static", "/joint_states"} <= set(rns.BAG_TOPICS)


def test_arm_pose_accepts_the_qualified_poses_only():
    parser = rns.build_parser()
    assert parser.parse_args(["overhead_course_nav", "--arm-pose", "home"]).arm_pose == "home"
    assert parser.parse_args(["overhead_course_nav", "--arm-pose", "carry"]).arm_pose == "carry"
    assert parser.parse_args(["overhead_course_nav"]).arm_pose is None
    for name in ("base_only", "level_extension", "no_such_pose"):
        with pytest.raises(SystemExit):
            parser.parse_args(["overhead_course_nav", "--arm-pose", name])


def test_qualified_poses_are_the_payload_transport_poses():
    # The runner's --arm-pose choices (and Unity's scenario_place list) mirror poses_rad;
    # level_extension is a payload pose but not a transport pose (no footprint profile).
    poses = rns.json.loads(rns.QUALIFIED_PAYLOAD.read_text())["poses_rad"]
    assert set(rns.QUALIFIED_POSES) == set(poses) - {"level_extension"}
    for name, q in rns.QUALIFIED_POSES.items():
        assert q == pytest.approx(poses[name], abs=1e-12), name


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


def test_slowdown_zone_defaults_to_the_launch_default():
    # Phase 4B-A: navigation.launch.py's default is off for B5 and on for B1-B4; the runner passes nothing.
    parser = rns.build_parser()
    assert parser.parse_args(["narrow_gate_home_nav"]).slowdown_zone is None
    assert rns.effective_slowdown(None, wholebody=True) is False
    assert rns.effective_slowdown(None, wholebody=False) is True
    assert rns.slowdown_launch_argument(None) == ""


def test_slowdown_zone_can_be_set_either_way_for_every_controller():
    parser = rns.build_parser()
    for controller in rns.CONTROLLERS:
        off = parser.parse_args(["narrow_gate_home_nav", "--controller", controller, "--no-slowdown-zone"])
        assert off.slowdown_zone is False
        assert parser.parse_args(["narrow_gate_home_nav", "--controller", controller,
                                  "--slowdown-zone", "false"]).slowdown_zone is False
        on = parser.parse_args(["narrow_gate_home_nav", "--controller", controller, "--slowdown-zone", "true"])
        assert on.slowdown_zone is True
    assert rns.slowdown_launch_argument(True) == " slowdown_zone:=true"
    assert rns.slowdown_launch_argument(False) == " slowdown_zone:=false"
    assert rns.effective_slowdown(True, wholebody=True) is True


def test_slowdown_suffix_marks_a_departure_from_the_default():
    assert rns.slowdown_suffix(None, wholebody=True) == rns.slowdown_suffix(None, wholebody=False) == ""
    assert rns.slowdown_suffix(False, wholebody=False) == "-noslow"
    assert rns.slowdown_suffix(False, wholebody=True) == ""   # B5's default
    assert rns.slowdown_suffix(True, wholebody=True) == "-slow"  # the slowdown ablation
    assert rns.slowdown_suffix(True, wholebody=False) == ""


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


def test_no_slack_gate_is_a_wholebody_option_with_a_suffix():
    parser = rns.build_parser()
    args = parser.parse_args(["worker_late_crossing_nav", "--controller", "wholebody", "--no-slack-gate"])
    overrides = rns.mpc_overrides(args, wholebody=True)
    assert overrides == {"slack_gate": False}
    assert rns.mpc_suffix(overrides) == "-nogate"
    assert rns.mpc_launch_arguments(overrides) == " slack_gate:=false"
    assert rns.mpc_launch_arguments({"posture_mode": "comfort_only", "clearance_weight": 20.0}) == \
        " posture_mode:=comfort_only clearance_weight:=20.0"
    # Not given: wholebody_mpc.yaml's gate (on).
    assert rns.mpc_overrides(parser.parse_args(["x", "--controller", "wholebody"]), True) == {}
    with pytest.raises(RuntimeError, match="no-slack-gate"):
        rns.mpc_overrides(parser.parse_args(["x", "--no-slack-gate"]), False)


V1_SCENE, V2_SCENE = "Assets/Scenes/ConstructionSiteV1.unity", "Assets/Scenes/ConstructionSiteV2.unity"


class SiteRunner(rns.Runner):
    """The real Runner logic over a fake Unity and container; records every call."""

    def __init__(self, scene=V1_SCENE):
        super().__init__("unused")
        self.scene, self.playing_now, self.calls = scene, True, []

    def unity(self, command, *parameters, timeout=60):
        self.calls.append((command, *parameters))
        if command == "site_active":
            return {"scene": self.scene, "playing": self.playing_now}
        if command == "site_open":
            self.scene = parameters[1]
        if command in ("editor_stop", "editor_play"):
            self.playing_now = command == "editor_play"
        return {}

    def playing(self):
        return self.playing_now

    def ros(self, script, **kwargs):
        self.calls.append(("ros", script))

    def start(self, name, command):
        self.calls.append(("start", name, command))

    def stop(self, name, patterns):
        self.calls.append(("stop", name))

    def wait_for_hold(self, seconds=60):
        return {}


@pytest.fixture(autouse=True)
def no_sleep(monkeypatch):
    monkeypatch.setattr(rns.time, "sleep", lambda seconds: None)


def unity_calls(runner):
    return [call[0] if call[0] != "site_open" else call[:3] for call in runner.calls
            if call[0] in ("editor_stop", "editor_play", "site_open")]


def test_mixed_site_batch_switches_scene_only_when_it_changes():
    runner = SiteRunner()
    sites = [("construction_site", V1_SCENE), ("construction_site_v2", V2_SCENE),
             ("construction_site_v2", V2_SCENE), ("construction_site", V1_SCENE)]
    switched = [runner.ensure_scene(scene, site) for site, scene in sites]
    assert switched == [False, True, False, True]
    assert unity_calls(runner) == ["editor_stop", ("site_open", "--scene", V2_SCENE), "editor_play",
                                   "editor_stop", ("site_open", "--scene", V1_SCENE), "editor_play"]
    assert runner.site == "construction_site"


def test_new_epoch_without_a_scene_does_not_open_one():
    runner = SiteRunner()
    runner.new_epoch()
    assert unity_calls(runner) == ["editor_stop", "editor_play"]


def test_every_launch_carries_the_site():
    runner = SiteRunner()
    assert runner.site == "construction_site"
    runner.ensure_scene(V2_SCENE, "construction_site_v2")
    runner.start_launch("nav", "mobile_manipulator_navigation", "navigation.launch.py",
                        "footprint_profile:=home controller:=rpp")
    command = [call for call in runner.calls if call[:2] == ("start", "nav")][0][2]
    assert command == ("ros2 launch mobile_manipulator_navigation navigation.launch.py "
                       "footprint_profile:=home controller:=rpp site:=construction_site_v2")


def test_disturbances_are_switched_on_for_the_scenario_and_off_after():
    runner = SiteRunner()
    with rns.DisturbanceScope(runner) as disturbances:
        disturbances.enable({"disturbances": True})
        assert ("site_disturbances", "--enabled", "true") in runner.calls
    assert runner.calls[-1] == ("site_disturbances", "--enabled", "false")


def test_disturbances_are_switched_off_after_a_failed_run():
    runner = SiteRunner()
    with pytest.raises(RuntimeError, match="boom"):
        with rns.DisturbanceScope(runner) as disturbances:
            disturbances.enable({"disturbances": True})
            raise RuntimeError("boom")
    assert runner.calls[-1] == ("site_disturbances", "--enabled", "false")


def test_v1_scenarios_never_touch_disturbances():
    runner = SiteRunner()
    with rns.DisturbanceScope(runner) as disturbances:
        disturbances.enable({"disturbances": False})
    assert not [call for call in runner.calls if call[0] == "site_disturbances"]


def test_plan_only_runs_a_nav_scenario_as_compute_path():
    parser = rns.build_parser()
    nav = {"task": "navigate_to_pose", "timeout_s": 90, "planners": ["x"]}
    plain = parser.parse_args(["open_space_nav"])
    assert rns.plan_only_scenario(nav, plain) is nav and plain.plan_only is False
    planned = rns.plan_only_scenario(nav, parser.parse_args(["open_space_nav", "--plan-only"]))
    assert planned["task"] == "compute_path" and planned["planners"] == ["GridBased", "Lattice"]
    mission = rns.plan_only_scenario({"task": "mission"}, parser.parse_args(["x", "--plan-only"]))
    assert mission["task"] == "compute_path"
    path = {"task": "compute_path", "planners": ["A"]}
    assert rns.plan_only_scenario(path, parser.parse_args(["x", "--plan-only"])) is path


def test_summary_records_site_scene_and_disturbances():
    spec = {"site": "construction_site_v2", "scene": V2_SCENE, "disturbances": True}
    assert rns.site_summary(spec) == spec
    assert rns.site_summary({"site": "construction_site", "scene": V1_SCENE}) == {
        "site": "construction_site", "scene": V1_SCENE, "disturbances": False}


class SpecRunner(SiteRunner):
    def ros(self, script, **kwargs):
        self.calls.append(("ros", script))
        spec = {"site": "construction_site_v2", "scene": V2_SCENE}
        return types.SimpleNamespace(stdout=rns.json.dumps(spec) + "\n", stderr="", returncode=0)


def test_new_epoch_flag_opens_the_first_scenarios_scene_in_one_epoch():
    runner = SpecRunner()  # Unity starts on V1
    rns.begin_epoch(runner, "v2_open_nav")
    assert unity_calls(runner) == ["editor_stop", ("site_open", "--scene", V2_SCENE), "editor_play"]
    assert runner.ensure_scene(V2_SCENE, "construction_site_v2") is False  # run_scenario adds no second epoch
    assert unity_calls(runner).count("editor_play") == 1 and runner.site == "construction_site_v2"


def test_a_failing_disturbance_switch_off_does_not_replace_the_original_error(capsys):
    runner = SiteRunner()
    runner.set_disturbances = lambda enabled: (_ for _ in ()).throw(RuntimeError("unity gone")) if not enabled else None
    with pytest.raises(RuntimeError, match="boom"):
        with rns.DisturbanceScope(runner) as disturbances:
            disturbances.enable({"disturbances": True})
            raise RuntimeError("boom")
    assert "unity gone" in capsys.readouterr().out


def test_all_scenarios_run_the_default_site_only_unless_a_site_is_given():
    parser = rns.build_parser()
    assert parser.parse_args([]).site == "construction_site"
    assert parser.parse_args(["--site", "construction_site_v2"]).site == "construction_site_v2"
    assert rns.list_command("construction_site").endswith("scenario_spec --list --site construction_site")


def test_main_without_names_lists_the_selected_sites_scenarios(monkeypatch):
    asked = []

    class ListRunner(SiteRunner):
        def ros(self, script, **kwargs):
            asked.append(script)
            return types.SimpleNamespace(stdout='["v2_a"]\n', stderr="", returncode=0)

    seen = []
    monkeypatch.setattr(rns, "Runner", lambda container: ListRunner())
    monkeypatch.setattr(rns, "keep_host_awake", lambda: None)
    monkeypatch.setattr(rns, "run_scenario", lambda runner, name, args: seen.append(name) or ("run", False))
    monkeypatch.setattr(rns.sys, "argv", ["run_nav_scenario.py"])
    rns.main()
    assert asked == [rns.list_command("construction_site")] and seen == ["v2_a"]
    asked.clear()
    monkeypatch.setattr(rns.sys, "argv", ["run_nav_scenario.py", "--site", "construction_site_v2"])
    rns.main()
    assert asked == [rns.list_command("construction_site_v2")]


def test_arm_pose_offers_only_poses_with_a_footprint_profile():
    parser = rns.build_parser()
    assert parser.parse_args(["overhead_course_nav", "--arm-pose", "vertical_carry"]).arm_pose == "vertical_carry"
    with pytest.raises(SystemExit):
        parser.parse_args(["overhead_course_nav", "--arm-pose", "vertical_carry_right"])


def test_wholebody_bag_records_movers():
    # Phase 4B-B: the replay bench needs the movers the MPC saw (spec Section 3.2).
    assert "/scenario/movers" in rns.WHOLEBODY_BAG_TOPICS


def test_mpc_config_suffix_and_launch_argument():
    parser = rns.build_parser()
    args = parser.parse_args(["narrow_gate_home_nav", "--controller", "wholebody", "--mpc-config", "x/fast.yaml"])
    assert rns.mpc_config_suffix(args.mpc_config) == "-cfgfast"
    assert rns.mpc_config_launch_argument(args.mpc_config) == \
        f" mpc_config_file:={rns.WORKSPACE}/x/fast.yaml"
    assert rns.mpc_config_suffix(None) == "" and rns.mpc_config_launch_argument(None) == ""
    with pytest.raises(RuntimeError, match="--controller wholebody"):
        rns.check_mpc_config(parser.parse_args(["x", "--mpc-config", "x/fast.yaml"]), wholebody=False)
    with pytest.raises(RuntimeError, match="inside the repository"):
        rns.check_mpc_config(parser.parse_args(["x", "--controller", "wholebody", "--mpc-config", "/tmp/a.yaml"]),
                             wholebody=True)


def test_mpc_config_must_exist_and_is_recorded_relative():
    # Task 1 review: a typo fails before the run starts; summary.json records the repository-relative path.
    parser = rns.build_parser()
    with pytest.raises(RuntimeError, match="no such file"):
        rns.check_mpc_config(parser.parse_args(["x", "--controller", "wholebody", "--mpc-config",
                                                "docs/no_such_config.yaml"]), wholebody=True)
    existing = "ros2_ws/src/mobile_manipulator_mpc/config/wholebody_mpc.yaml"
    rns.check_mpc_config(parser.parse_args(["x", "--controller", "wholebody", "--mpc-config", existing]), True)
    assert rns.mpc_config_record(str(rns.ROOT / existing)) == existing
    assert rns.mpc_config_record(existing) == existing
    assert rns.mpc_config_record(None) is None


def test_clearance_summary_includes_the_self_minimum():
    clearance = {"min_distance_m": 0.051, "link": "panel", "obstacle": "beam", "time_s": 12.5,
                 "self_min_m": 0.105, "self_link": "wrist_2_link"}
    assert rns.clearance_summary(clearance) == {
        "clearance_3d_m": 0.051, "clearance_3d_link": "panel", "clearance_3d_obstacle": "beam",
        "clearance_3d_time_s": 12.5, "clearance_self_m": 0.105, "clearance_self_link": "wrist_2_link"}
    assert rns.clearance_summary({})["clearance_self_m"] is None


def test_every_run_sets_the_arm_gravity_feedforward_explicitly():
    runner = SiteRunner()
    rns.set_arm_gravity_feedforward(runner, True)
    rns.set_arm_gravity_feedforward(runner, False)
    assert [call for call in runner.calls if call[0] == "arm_gravity_feedforward"] == [
        ("arm_gravity_feedforward", "--enabled", "true"), ("arm_gravity_feedforward", "--enabled", "false")]


def test_gravity_feedforward_runs_get_their_own_directory():
    assert rns.gravity_feedforward_suffix(True) == "-gff"
    assert rns.gravity_feedforward_suffix(False) == ""


def test_gravity_feedforward_is_on_unless_the_frozen_arm_is_asked_for():
    parser = rns.build_parser()
    assert parser.parse_args(["overhead_course_nav"]).arm_gravity_feedforward is True
    assert parser.parse_args(["overhead_course_nav", "--no-arm-gravity-feedforward"]).arm_gravity_feedforward is False


def test_every_run_sets_the_arm_motion_feedforward_explicitly():
    runner = SiteRunner()
    rns.set_arm_motion_feedforward(runner, True)
    rns.set_arm_motion_feedforward(runner, False)
    assert [call for call in runner.calls if call[0] == "arm_motion_feedforward"] == [
        ("arm_motion_feedforward", "--enabled", "true"), ("arm_motion_feedforward", "--enabled", "false")]


def test_motion_feedforward_is_on_by_default_and_marks_its_runs():
    parser = rns.build_parser()
    assert parser.parse_args(["overhead_course_nav"]).arm_motion_feedforward is True
    assert parser.parse_args(["overhead_course_nav", "--no-arm-motion-feedforward"]).arm_motion_feedforward is False
    assert rns.motion_feedforward_suffix(True) == "-mff" and rns.motion_feedforward_suffix(False) == ""


def test_setpoint_preview_is_set_every_run_on_by_default_and_marks_its_runs():
    runner = SiteRunner()
    rns.set_arm_setpoint_preview(runner, True)
    assert ("arm_setpoint_preview", "--enabled", "true") in runner.calls
    parser = rns.build_parser()
    assert parser.parse_args(["overhead_course_nav"]).arm_setpoint_preview is True
    assert parser.parse_args(["overhead_course_nav", "--no-arm-setpoint-preview"]).arm_setpoint_preview is False
    assert rns.setpoint_preview_suffix(True) == "-spv" and rns.setpoint_preview_suffix(False) == ""
