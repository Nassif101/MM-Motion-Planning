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
    assert rns.WHOLEBODY_BAG_TOPICS == ["/wholebody_mpc/cycle", "/wholebody_mpc/plan"]
    assert {"/tf", "/tf_static", "/joint_states"} <= set(rns.BAG_TOPICS)
