"""Tests of the parallel plant campaign runner's pure parts (MuJoCo plant plan Task 8)."""
import json

import pytest

import plant_campaign as pc


def corpus(tmp_path):
    path = tmp_path / "corpus.json"
    path.write_text(json.dumps({"cells": [{"cell": "a", "scenario": "v2_a", "bag": "runs/a/bag"},
                                          {"cell": "b", "scenario": "v2_b", "bag": None}]}))
    return path


def test_runs_cover_cells_and_seeds(tmp_path):
    runs = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=3, mode="ideal", kinds=("f2", "b5"))
    assert len(runs) == 2 * 3 * 2
    assert len({r["name"] for r in runs}) == len(runs)  # one directory each
    assert {r["seed"] for r in runs} == {1, 2, 3}
    synthetic = next(r for r in runs if r["cell"] == "b")
    assert "--bag" not in synthetic["args"]


def test_command_runs_in_the_container(tmp_path):
    run = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=1, mode="realistic", kinds=("b5",))[0]
    cmd = pc.command(run, "out/x", container="c1")
    assert cmd[:3] == ["docker", "exec", "c1"]
    joined = " ".join(cmd)
    assert "plant_campaign" in joined and "--mode realistic" in joined and "--b5-only" in joined
    assert pc.command(run, "out/x", container="")[0].endswith("plant_campaign")  # already inside


def test_table_has_plant_fields():
    record = {"cell": "a", "seed": 2, "kind": "f2", "plant_profile": "mujoco-ideal", "reached": True, "end_status": "goal_reached",
              "time_s": 30.5, "min_clearance_m": 0.06, "plant_min_clearance_m": 0.055, "plant_contacts": 0,
              "following_cycles": 900, "b5_cycles": 1000, "real_time_factor": 12.3}
    text = pc.table([record])
    for column in ("Plant min clearance", "Contacts", "RTF", "Seed"):
        assert column in text
    assert "mujoco-ideal" in text and "0.055" in text


def test_ensemble_table_rates_per_cell():
    records = [{"cell": "a", "kind": "f2", "reached": r, "time_s": 10.0 + i, "plant_profile": "mujoco-ideal"}
               for i, r in enumerate([True, True, False, True])]
    text = pc.ensemble_table(records)
    assert "| a | f2 | 4 | 3/4 (75 %) |" in text


def test_table_names_a_timeout():
    record = {"cell": "a", "seed": 1, "reached": False, "end_status": "ok", "timed_out": True, "time_s": 120.0}
    assert "no (timeout)" in pc.table([record])


def test_runs_take_a_profile_variant(tmp_path):
    # the grip ensemble (CP3b): the same cells on a profile variant, its tag in the run names
    runs = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=2, mode="realistic", kinds=("f2",),
                      profile="experiment_runs/x/profile_mu0.4.yaml", tag="mu0.4")
    assert all(r["args"][r["args"].index("--profile") + 1] == "experiment_runs/x/profile_mu0.4.yaml" for r in runs)
    assert all(r["name"].endswith(("mu0.4-s1", "mu0.4-s2")) and r["variant"] == "mu0.4" for r in runs)


def test_finished_runs_are_skipped(tmp_path):
    # a campaign of hours restarts where it stopped: a run with a summary is done
    runs = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=2, mode="ideal", kinds=("f2",))
    done = tmp_path / "out" / runs[0]["name"]
    done.mkdir(parents=True)
    (done / "summary.json").write_text(json.dumps({"args": runs[0]["args"]}))  # done with these arguments
    (tmp_path / "out" / runs[1]["name"]).mkdir()  # started, no summary: runs again
    left = pc.pending(runs, tmp_path / "out")
    assert [r["name"] for r in left] == [r["name"] for r in runs[1:]]


def test_table_names_a_stall():
    record = {"cell": "a", "seed": 1, "reached": False, "end_status": "holding", "timed_out": False, "stalled": True,
              "time_s": 85.0}
    assert "no (stalled)" in pc.table([record])


def test_resume_reruns_a_run_with_another_configuration(tmp_path):
    # a summary is done only for the arguments it ran with (another timeout, lidar, profile or bag: run again)
    runs = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=1, mode="ideal", kinds=("f2",))
    for r, args in zip(runs, (runs[0]["args"], runs[1]["args"] + ["--lidar"])):
        d = tmp_path / "out" / r["name"]
        d.mkdir(parents=True)
        (d / "summary.json").write_text(json.dumps({"args": args}))
    left = pc.pending(runs, tmp_path / "out")
    assert [r["name"] for r in left] == [runs[1]["name"]]


def test_command_quotes_its_arguments(tmp_path):
    import shlex
    run = {"name": "x", "args": ["--scenario", "v2_a", "--bag", "experiment_runs/a run/bag"]}
    inner = pc.command(run, "out/x", container="c1")[-1]
    words = shlex.split(inner.split("&& ")[-1])
    assert words[words.index("--bag") + 1] == "experiment_runs/a run/bag"


def test_command_takes_an_absolute_out(tmp_path):
    import pytest
    run = {"name": "x", "args": ["--scenario", "v2_a"]}
    inner = pc.command(run, pc.ROOT / "experiment_runs/y/x", container="c1")[-1]
    assert f"--out {pc.WORKSPACE}/experiment_runs/y/x" in inner
    with pytest.raises(ValueError):
        pc.command(run, "/tmp/elsewhere/x", container="c1")  # the container sees the repository only


def test_stop_command_stops_only_this_campaigns_runs():
    cmd = pc.stop_command("experiment_runs/y", "c1")
    assert cmd[:3] == ["docker", "exec", "c1"] and "pkill" in cmd
    assert f"--out {pc.WORKSPACE}/experiment_runs/y/" in cmd[-1]
