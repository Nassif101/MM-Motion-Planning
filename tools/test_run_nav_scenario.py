"""Process handling of tools/run_nav_scenario.py's contact watch, without Docker or Unity.

Run: python3 -m pytest tools/test_run_nav_scenario.py
"""
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
