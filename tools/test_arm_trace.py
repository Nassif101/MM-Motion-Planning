"""tools/arm_trace.py on a fixture trace (run: python3 -m pytest tools/test_arm_trace.py)."""
import json
import math

import pytest

import arm_trace

HOME = [0.0] * 6
VERTICAL = [math.pi / 2, 0, 0, 0, math.pi / 2, 0]
POSES = {"home": HOME, "vertical_carry": VERTICAL}


def sample(t, q, rpy=(0.0, 0.0, 0.0), tightness=0.0):
    return {"t": t, "q": list(q), "panel_rpy": list(rpy), "tightness": tightness}


@pytest.fixture
def trace():
    """5 s in home, 3 s raising to vertical carry, 4 s held there, 2 s in an unnamed lowered pose.

    The panel yaw crosses +-pi while in vertical carry (unwrapped it is one continuous range), and
    the joints jitter by 0.02 rad as measured joints do.
    """
    out, t = [], 100.0  # node clock: the summary times are relative to the first sample
    for i in range(25):
        out.append(sample(t, [0.02 * (-1) ** i] + HOME[1:], (0.0, 0.0, 0.5)))
        t += 0.2
    for i in range(1, 16):
        f = i / 15
        out.append(sample(t, [f * v for v in VERTICAL], (0.0, 0.0, 0.5 + f * 2.5), tightness=f))
        t += 0.2
    for i in range(20):
        yaw = math.remainder(3.0 + 0.03 * i, 2 * math.pi)  # 3.0 to 3.57: past pi at i = 5
        out.append(sample(t, VERTICAL, (0.0, 0.0, yaw), tightness=1.0))
        t += 0.2
    lowered = [1.06, -0.31, -0.27, 0.0, 0.9, 0.0]
    for i in range(10):
        out.append(sample(t, lowered, (0.1, -0.2, 3.0), tightness=0.4))
        t += 0.2
    return out


def test_held_poses_are_named_and_timed(trace):
    summary = arm_trace.summarize(trace, POSES, radius=0.1, min_dwell_s=1.0)
    assert summary["samples"] == 70
    held = [(pose["pose"], pose["held_s"], pose["first_t"]) for pose in summary["held"]]
    # home 5 s, vertical carry 4 s, the lowered pose 2 s; the raise is transit. The 1 s speed window
    # takes up to 0.4 s off each end of a hold next to a motion (the lowered pose is reached in one step).
    assert [name for name, _, _ in held] == ["home", "vertical_carry", None]
    assert held[0][1] == pytest.approx(5.0, abs=0.81) and held[0][2] == 0.0
    assert held[1][1] == pytest.approx(4.0, abs=0.81) and held[1][2] == pytest.approx(8.0, abs=0.41)
    assert held[2][1] == pytest.approx(2.0, abs=0.41)
    assert summary["transit_s"] == pytest.approx(70 * 0.2 - sum(h for _, h, _ in held))
    assert summary["held"][2]["q"] == [1.06, -0.31, -0.27, 0.0, 0.9, 0.0]


def test_joint_panel_and_tightness_ranges(trace):
    summary = arm_trace.summarize(trace, POSES)
    assert summary["joints_rad"]["shoulder_pan"] == [-0.02, round(math.pi / 2, 3)]
    assert summary["joints_rad"]["shoulder_lift"] == [-0.31, 0.0]
    assert summary["joints_rad"]["wrist_2"] == [0.0, round(math.pi / 2, 3)]
    # The yaw crosses +-pi in vertical carry: unwrapped, its range is continuous (no -pi..pi span).
    low, high = summary["panel_rpy_rad"]["yaw"]
    assert low == pytest.approx(0.5) and high == pytest.approx(3.57)
    assert summary["panel_rpy_rad"]["pitch"] == [-0.2, 0.0]
    assert summary["tightness"] == [0.0, 1.0]
    assert summary["duration_s"] == pytest.approx(69 * 0.2, abs=0.05)


def test_a_coarser_dwell_lists_fewer_poses(trace):
    assert [p["pose"] for p in arm_trace.summarize(trace, POSES, min_dwell_s=3.0)["held"]] == ["home",
                                                                                            "vertical_carry"]


def test_reads_run_directories_and_tolerates_runs_without_a_trace(tmp_path, trace, capsys, monkeypatch):
    run = tmp_path / "20261013T000000Z-narrow_gate_home_nav-wholebody-noslow"
    run.mkdir()
    (run / "summary.json").write_text(json.dumps({
        "slowdown_zone": False,
        "mpc": {"options": {"posture_mode": "comfort_only", "clearance_weight": 20.0}, "arm_trace": trace}}))
    other = tmp_path / "old-summary.json"
    other.write_text(json.dumps({"mpc": {"cycles": 10}}))
    monkeypatch.setattr("sys.argv", ["arm_trace.py", str(run), str(other)])
    monkeypatch.setattr(arm_trace, "named_poses", lambda: POSES)
    arm_trace.main()
    out, err = capsys.readouterr()
    assert "posture_mode comfort_only, clearance_weight 20.0, no slowdown zone" in out
    assert "held home" in out and "held vertical_carry" in out and "held (unnamed)" in out
    assert "no mpc.arm_trace" in err


def test_the_named_poses_come_from_the_qualified_payload():
    poses = arm_trace.named_poses()
    assert poses["home"] == HOME
    assert poses["vertical_carry"] == pytest.approx(VERTICAL)


def test_roll_and_yaw_leave_out_the_gimbal_lock_and_the_normal_tilt_is_given():
    # Vertical carry: pitch pi/2, where roll and yaw trade off freely (here they jump by pi).
    trace = [sample(0.2 * i, HOME, (0.0, 0.0, 0.1)) for i in range(5)]
    trace += [sample(1.0 + 0.2 * i, VERTICAL, (math.pi * (i % 2), math.pi / 2 - 1e-6, 0.3 + math.pi * (i % 2)))
              for i in range(5)]
    panel = arm_trace.summarize(trace, POSES)["panel_rpy_rad"]
    assert panel["locked_samples"] == 5
    assert panel["roll"] == [0.0, 0.0] and panel["yaw"] == [0.1, 0.1]
    assert panel["pitch"] == [0.0, round(math.pi / 2 - 1e-6, 3)]
    assert panel["normal_tilt"] == [0.0, round(math.pi / 2, 3)]
    assert "without the 5 samples" in arm_trace.text(arm_trace.summarize(trace, POSES))
