"""tools/demo.py decisions that need no Docker or Unity.

Run: python3 -m pytest tools/test_demo.py
"""
import re
import sys
import types

import pytest

sys.modules.setdefault("analyze_arm_qualification", types.SimpleNamespace(analyze=None))
import demo  # noqa: E402


class NoContainer:
    """A runner that fails the test when the demo reaches the container."""

    def ros(self, *args, **kwargs):
        raise AssertionError(f"reached the container: {args}")


def test_arm_refuses_carry_before_reaching_moveit():
    # carry is qualified only through home with arm_experiment and as B5's comfort pose, not as a
    # MoveIt (reconfigure_panel) target.
    with pytest.raises(SystemExit) as refused:
        demo.arm(NoContainer(), "carry")
    message = str(refused.value)
    assert "not qualified with MoveIt" in message
    assert "--arm-pose carry" in message and "B5" in message


def test_arm_offers_only_the_moveit_qualified_poses():
    assert demo.MOVEIT_ARM_POSES == ("home", "vertical_carry")
    assert set(demo.MOVEIT_ARM_POSES) < set(demo.ARM_POSES)


def test_demo_site_defaults_to_the_scenario_site():
    table = {"open_space_nav": ("navigate_to_pose", "d", "construction_site"),
             "v2_open_nav": ("navigate_to_pose", "d", "construction_site_v2")}
    assert demo.choose_site(table, "v2_open_nav", None) == "construction_site_v2"
    assert demo.choose_site(table, "open_space_nav", None) == "construction_site"
    assert demo.choose_site(table, None, None) == "construction_site"
    assert demo.choose_site(table, "v2_open_nav", "construction_site") == "construction_site"


class Stopped(Exception):
    pass


def test_drive_opens_the_sites_scene_before_starting_the_stack(monkeypatch):
    calls = []

    class Recorder:
        site = "construction_site"

        def ensure_scene(self, scene, site):
            calls.append((scene, site))

        def playing(self):
            return True

        def stop(self, *args):
            raise Stopped

    monkeypatch.setattr(demo, "site_scene", lambda site: f"scene-of-{site}")
    with pytest.raises(Stopped):
        demo.drive(Recorder(), "rpp", None, site="construction_site_v2")
    assert calls == [("scene-of-construction_site_v2", "construction_site_v2")]


def test_menu_up_keeps_the_chosen_site(monkeypatch):
    ups, answers = [], iter(["up", "quit"])
    monkeypatch.setattr(demo, "up", lambda runner, site: ups.append(site))
    monkeypatch.setattr(demo, "choose", lambda *args, **kwargs: next(answers))
    demo.menu(types.SimpleNamespace(site="construction_site_v2"))
    assert ups == ["construction_site_v2"]


class FakeUnity:
    def __init__(self, scene, playing):
        self.site, self.scene, self.is_playing, self.calls = "construction_site", scene, playing, []

    def unity(self, command, *parameters, timeout=60):
        return {"scene": self.scene, "playing": self.is_playing}

    def playing(self):
        return self.is_playing

    def ensure_scene(self, scene, site):
        self.site = site
        return False

    def new_epoch(self, scene=None):
        self.calls.append("new_epoch")

    def stop(self, *args):
        raise Stopped


V1_SCENE, V2_SCENE = "Assets/Scenes/ConstructionSiteV1.unity", "Assets/Scenes/ConstructionSiteV2.unity"


def test_site_defaults_to_the_site_open_in_unity():
    assert demo.choose_site({}, None, None, FakeUnity(V2_SCENE, True)) == "construction_site_v2"
    assert demo.choose_site({}, None, None, FakeUnity(V1_SCENE, True)) == "construction_site"
    assert demo.choose_site({}, None, None, FakeUnity("Assets/Scenes/Other.unity", True)) == "construction_site"
    assert demo.choose_site({}, None, "construction_site", FakeUnity(V2_SCENE, True)) == "construction_site"


def test_active_site_falls_back_to_v1_when_unity_is_unreachable():
    class Down:
        def unity(self, *args, **kwargs):
            raise RuntimeError("No Unity Editor instances found")

    assert demo.active_site(Down()) == "construction_site"


def test_menu_site_is_the_open_scenes_site_unless_given(monkeypatch):
    sites = []
    monkeypatch.setattr(demo, "menu", lambda runner: sites.append(runner.site))
    monkeypatch.setattr(demo, "keep_host_awake", lambda: None)
    for argv, expected in ((["demo.py"], "construction_site_v2"),
                           (["demo.py", "--site", "construction_site"], "construction_site")):
        runner = FakeUnity(V2_SCENE, True)
        monkeypatch.setattr(demo, "Runner", lambda container, runner=runner: runner)
        monkeypatch.setattr(sys, "argv", argv)
        demo.main()
    assert sites == ["construction_site_v2", "construction_site"]


def test_drive_enters_play_when_the_right_scene_is_open_but_stopped(monkeypatch):
    monkeypatch.setattr(demo, "site_scene", lambda site: V2_SCENE)
    runner = FakeUnity(V2_SCENE, False)
    with pytest.raises(Stopped):
        demo.drive(runner, "rpp", None, site="construction_site_v2")
    assert runner.calls == ["new_epoch"]
    running = FakeUnity(V2_SCENE, True)
    with pytest.raises(Stopped):
        demo.drive(running, "rpp", None, site="construction_site_v2")
    assert running.calls == []


class FakeStack:
    """A runner whose container holds the given process names; records stops and starts."""

    def __init__(self, processes):
        self.site, self.processes, self.events = "construction_site", processes, []

    def ros(self, command, **kwargs):
        regex = command.split("grep -E '")[1].split("'")[0]
        listed = [name for name in self.processes if re.search(regex, name)]
        return types.SimpleNamespace(stdout="\n".join(listed))

    def stop(self, name, patterns):
        self.events.append(("stop", name))
        self.processes = []

    def start(self, name, command):
        self.events.append(("start", name))

    def unity(self, command, *parameters, timeout=60):
        raise Stopped


@pytest.mark.parametrize("processes,events", [
    (["ros2 launch simulation.launch.py", "robot_state_publisher", "static_transform_publisher --child-frame-id odom map_to_odom"], []),
    (["static_transform_publisher --child-frame-id odom map_to_odom"], [("stop", "description"), ("start", "description")]),
    ([], [("start", "description")]),
])
def test_up_needs_the_description_stack_not_just_map_to_odom(monkeypatch, processes, events):
    monkeypatch.setattr(demo, "site_scene", lambda site: V1_SCENE)
    runner = FakeStack(processes)
    with pytest.raises(Stopped):
        demo.up(runner)
    assert runner.events == events


class LaunchRecorder:
    """A runner that accepts every call and records the navigation launch arguments."""

    def __init__(self):
        self.launches = {}
        self.site = None

    def ensure_scene(self, scene, site):
        return False

    def playing(self):
        return True

    def stop(self, *args):
        pass

    def unity(self, *args):
        return {}

    def qualified_pose(self):
        return "carry"

    def ros(self, *args, **kwargs):
        return types.SimpleNamespace(returncode=0, stdout="", stderr="")

    def start_launch(self, name, package, launch_file, arguments=""):
        self.launches[name] = arguments


def test_drive_passes_the_slowdown_zone_only_when_set(monkeypatch):
    # Phase 4B-A: unset keeps navigation.launch.py's default (off for B5, on otherwise).
    monkeypatch.setattr(demo, "site_scene", lambda site: "scene")
    for value, expected in ((None, None), (True, "slowdown_zone:=true"), (False, "slowdown_zone:=false")):
        recorder = LaunchRecorder()
        demo.drive(recorder, "wholebody", None, slowdown_zone=value)
        arguments = recorder.launches["nav"]
        if expected is None:
            assert "slowdown_zone" not in arguments
        else:
            assert expected in arguments


class UpRunner:
    """Unity in Play with V1 open; records Unity commands."""

    def __init__(self):
        self.site, self.calls = "construction_site", []

    def unity(self, command, *parameters, timeout=60):
        self.calls.append((command, *parameters))
        return {"scene": V1_SCENE}

    def playing(self):
        return True

    def wait_for_hold(self, seconds=10):
        return {}

    def start(self, name, command):
        pass

    def ensure_scene(self, scene, site):
        return True


@pytest.mark.parametrize("already_up", [True, False])
def test_up_switches_the_gravity_feedforward_on(monkeypatch, already_up):
    # The default lives in the edit scene; an Editor session that loaded it earlier may still hold off.
    monkeypatch.setattr(demo, "site_scene", lambda site: V1_SCENE)
    monkeypatch.setattr(demo, "running", lambda runner, patterns: already_up)
    runner = UpRunner()
    demo.up(runner)
    assert ("arm_gravity_feedforward", "--enabled", "true") in runner.calls
    assert ("arm_motion_feedforward", "--enabled", "true") in runner.calls
    assert ("arm_setpoint_preview", "--enabled", "false") in runner.calls
