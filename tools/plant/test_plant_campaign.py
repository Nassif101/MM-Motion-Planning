"""Tests of the parallel plant campaign runner's pure parts (MuJoCo plant plan Task 8)."""
import json
import shutil

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


def test_runs_name_their_corpus_cell(tmp_path):
    path = tmp_path / "c.json"
    path.write_text(json.dumps({"cells": [{"cell": "v2_x@run1", "scenario": "v2_x", "bag": "b"},
                                          {"cell": "v2_y", "scenario": "v2_y", "bag": "b"}]}))
    runs = pc.runs_of(pc.load_corpus(path), seeds=1, mode="ideal", kinds=("f2",))
    named = {r["cell"]: r["args"] for r in runs}
    assert named["v2_x@run1"][named["v2_x@run1"].index("--cell") + 1] == "v2_x@run1"
    assert "--cell" not in named["v2_y"]  # the scenario's own name: nothing to add


def test_runs_take_a_b5_config_and_a_nudge(tmp_path):
    # E1 (spec Section 3.3): a B5 config variant for every run, and twin runs nudged at the start yaw (name suffix -n)
    runs = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=3, mode="realistic", kinds=("b5",),
                      b5_config="experiment_runs/e1/b5_absolute.yaml", tag="abs", start_nudge=1e-12)
    assert all(r["args"][r["args"].index("--b5-config") + 1] == "experiment_runs/e1/b5_absolute.yaml" for r in runs)
    assert all(r["args"][r["args"].index("--start-nudge") + 1] == "1e-12" for r in runs)
    assert all(r["name"].endswith(("-abs-s1-n", "-abs-s2-n", "-abs-s3-n")) and r["twin"] for r in runs)
    plain = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=1, mode="realistic", kinds=("b5",))
    assert all("--b5-config" not in r["args"] and "--start-nudge" not in r["args"] and not r["twin"] for r in plain)
    assert plain[0]["name"] == "a-b5-realistic-s1"


def test_resume_reruns_when_the_b5_config_changed(tmp_path):
    # Review Focus 4: an edited variant under an unchanged path runs again (the summary's b5_config_sha256 differs)
    import hashlib
    rel = f"experiment_runs/test_plant_campaign_{tmp_path.name}/b5.yaml"  # repository-relative, as E1 uses it
    cfg = pc.ROOT / rel
    cfg.parent.mkdir(parents=True, exist_ok=True)
    cfg.write_text("slack_gate_inside: absolute\n")
    runs = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=1, mode="ideal", kinds=("f2",), b5_config=rel, tag="v")
    sha = hashlib.sha256(cfg.read_bytes()).hexdigest()
    for r in runs:
        d = tmp_path / "out" / r["name"]
        d.mkdir(parents=True)
        (d / "summary.json").write_text(json.dumps({"args": r["args"], "b5_config_sha256": sha}))
    assert pc.pending(runs, tmp_path / "out") == []
    cfg.write_text("slack_gate_inside: increment\n")
    try:
        assert [r["name"] for r in pc.pending(runs, tmp_path / "out")] == [r["name"] for r in runs]
    finally:
        import shutil
        shutil.rmtree(cfg.parent)


def test_run_all_counts_failed_runs(tmp_path, monkeypatch):
    # the pool that run and divergence.py share: every run once, its log beside it, the number of failures back
    runs = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=2, mode="ideal", kinds=("f2",))
    seen = []

    def fake(cmd, stdout, stderr):
        seen.append(cmd)
        class Done:
            returncode = 1 if "--seed" in cmd and cmd[cmd.index("--seed") + 1] == "2" else 0
        return Done()
    monkeypatch.setattr(pc.subprocess, "run", fake)
    out = tmp_path / "out"
    assert pc.run_all(runs, out, jobs=2, container="") == 2  # seed 2 of both cells
    assert len(seen) == len(runs)
    assert all((out / f"{r['name']}.log").exists() for r in runs)


def test_a_host_absolute_variant_becomes_repository_relative(tmp_path):
    # the runs start in the container's repository root: a host-absolute path under the repository is passed relative
    # (Task 3 review); one outside the repository is refused, as the container cannot see it
    inside = pc.ROOT / "experiment_runs" / "e1" / "b5_absolute.yaml"
    runs = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=1, mode="ideal", kinds=("b5",), b5_config=str(inside),
                      tag="abs")
    assert runs[0]["args"][runs[0]["args"].index("--b5-config") + 1] == "experiment_runs/e1/b5_absolute.yaml"
    assert runs[0]["b5_config"] == "experiment_runs/e1/b5_absolute.yaml"
    with pytest.raises(ValueError, match="outside the repository"):
        pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=1, mode="ideal", kinds=("b5",),
                   b5_config=str(tmp_path / "b5.yaml"), tag="abs")


def test_a_b5_variant_needs_a_tag(tmp_path):
    # without a tag the variant's runs would take the baseline runs' names and overwrite them in a shared --out
    with pytest.raises(ValueError, match="tag"):
        pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=1, mode="ideal", kinds=("b5",),
                   b5_config="experiment_runs/e1/b5_absolute.yaml")


def test_a_tag_has_no_hyphen(tmp_path):
    # run names are <cell>-<kind>-<mode>[-<tag>]-s<seed>: a hyphen in the tag breaks their parsing (compare_variants)
    with pytest.raises(ValueError, match="hyphen"):
        pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=1, mode="ideal", kinds=("b5",), tag="abs-mu0.4")


def test_runs_take_cells_a_seed_list_and_diagnostics(tmp_path):
    # E2 Task 4: the measurement runs pick cells and single seeds and record the diagnostics
    cells = pc.load_corpus(corpus(tmp_path))
    names = [c["cell"] for c in cells]
    runs = pc.runs_of(cells, seeds=None, mode="realistic", kinds=("f2",), seed_list=[4, 19], cells_only=[names[0]],
                      diagnostics=True)
    assert [r["seed"] for r in runs] == [4, 19]
    assert {r["cell"] for r in runs} == {names[0]}
    assert all("--diagnostics" in r["args"] and r["args"][r["args"].index("--seed") + 1] in ("4", "19") for r in runs)
    plain = pc.runs_of(cells, seeds=1, mode="realistic", kinds=("f2",))
    assert all("--diagnostics" not in r["args"] for r in plain)
    with pytest.raises(ValueError, match="no_such_cell"):
        pc.runs_of(cells, seeds=1, mode="realistic", kinds=("f2",), cells_only=["no_such_cell"])


def test_runs_take_a_tracking_model_and_its_adaptation(tmp_path):
    # E2 Task 7: a tracking model (repository-relative, or none) and the online correction on or off, every run;
    # either one is a variant and needs a tag; an edited model under the same path runs again on resume
    import hashlib
    cells = pc.load_corpus(corpus(tmp_path))
    rel = f"experiment_runs/test_plant_campaign_{tmp_path.name}/tracking.yaml"
    model = pc.ROOT / rel
    model.parent.mkdir(parents=True, exist_ok=True)
    model.write_text("sideslip: 0.1\n")
    try:
        runs = pc.runs_of(cells, seeds=1, mode="realistic", kinds=("b5",), tracking_model=str(model),
                          tracking_adaptation="on", tag="track")
        assert all(r["args"][r["args"].index("--tracking-model") + 1] == rel for r in runs)
        assert all(r["args"][r["args"].index("--tracking-adaptation") + 1] == "on" for r in runs)
        none = pc.runs_of(cells, seeds=1, mode="realistic", kinds=("b5",), tracking_model="none", tag="unicycle")
        assert all(r["args"][r["args"].index("--tracking-model") + 1] == "none" for r in none)
        plain = pc.runs_of(cells, seeds=1, mode="realistic", kinds=("b5",))
        assert all("--tracking-model" not in r["args"] and "--tracking-adaptation" not in r["args"] for r in plain)
        with pytest.raises(ValueError, match="tag"):
            pc.runs_of(cells, seeds=1, mode="realistic", kinds=("b5",), tracking_adaptation="off")
        # E2 Task 11: "none" reproduces the unicycle runs (the old default), so it needs no tag
        untagged = pc.runs_of(cells, seeds=1, mode="realistic", kinds=("b5",), tracking_model="none")
        assert all(r["args"][r["args"].index("--tracking-model") + 1] == "none" for r in untagged)
        with pytest.raises(ValueError, match="on|off"):
            pc.runs_of(cells, seeds=1, mode="realistic", kinds=("b5",), tracking_adaptation="yes", tag="t")
        # Task 10c: "gain" is the correction of the turn gain alone (the sideslip frozen)
        gain = pc.runs_of(cells, seeds=1, mode="realistic", kinds=("b5",), tracking_adaptation="gain", tag="g")
        assert all(r["args"][r["args"].index("--tracking-adaptation") + 1] == "gain" for r in gain)
        sha = hashlib.sha256(model.read_bytes()).hexdigest()
        for r in runs:
            d = tmp_path / "out" / r["name"]
            d.mkdir(parents=True)
            (d / "summary.json").write_text(json.dumps({"args": r["args"], "tracking_model": rel,
                                                        "tracking_model_sha256": sha}))
        assert pc.pending(runs, tmp_path / "out") == []
        model.write_text("sideslip: 0.2\n")
        assert [r["name"] for r in pc.pending(runs, tmp_path / "out")] == [r["name"] for r in runs]
    finally:
        import shutil
        shutil.rmtree(model.parent)


def test_resume_follows_the_tracking_model_the_run_loaded(tmp_path):
    # E2 Task 7 review I2, closure R2: resume reruns a run whose tracking model changed, whatever selected it (a B5
    # config variant in any YAML form, or --tracking-model): the summary records the path the C++ loader resolved and
    # its hash; a recorded model that is gone also reruns
    import hashlib
    rel_dir = f"experiment_runs/test_plant_campaign_{tmp_path.name}"
    folder = pc.ROOT / rel_dir
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "b5.yaml").write_text("slack_gate_inside: absolute\n'tracking_model': models/tracking.yaml\n")
    (folder / "models").mkdir()
    model = folder / "models" / "tracking.yaml"
    model.write_text("sideslip: 0.1\n")
    cells = pc.load_corpus(corpus(tmp_path))
    try:
        runs = pc.runs_of(cells, seeds=1, mode="ideal", kinds=("b5",), b5_config=f"{rel_dir}/b5.yaml", tag="v")
        cfg_sha = hashlib.sha256((folder / "b5.yaml").read_bytes()).hexdigest()
        sha = hashlib.sha256(model.read_bytes()).hexdigest()
        # as the session writes it: the container's view of the resolved path (relative here, absolute for the
        # installed config)
        recorded = [f"{rel_dir}/models/tracking.yaml", f"{pc.WORKSPACE}/{rel_dir}/models/tracking.yaml"]
        for r, path in zip(runs, recorded * len(runs)):
            d = tmp_path / "out" / r["name"]
            d.mkdir(parents=True)
            (d / "summary.json").write_text(json.dumps({"args": r["args"], "b5_config_sha256": cfg_sha,
                                                        "tracking_model": path, "tracking_model_sha256": sha}))
        assert pc.pending(runs, tmp_path / "out") == []
        model.write_text("sideslip: 0.2\n")
        assert [r["name"] for r in pc.pending(runs, tmp_path / "out")] == [r["name"] for r in runs]
        model.unlink()
        assert len(pc.pending(runs, tmp_path / "out")) == len(runs)
    finally:
        import shutil
        shutil.rmtree(folder)


def test_runs_take_profile_timing(tmp_path):
    # E2 Task 9 (T7): B5's solve and cycle times in the summaries (--profile-timing); not a variant, no tag needed
    cells = pc.load_corpus(corpus(tmp_path))
    timed = pc.runs_of(cells, seeds=1, mode="realistic", kinds=("b5",), profile_timing=True)
    assert all("--profile-timing" in r["args"] for r in timed)
    assert all("--profile-timing" not in r["args"] for r in pc.runs_of(cells, seeds=1, mode="realistic", kinds=("b5",)))


def test_a_run_on_the_default_model_reruns_when_the_installed_model_changes(tmp_path):
    # E2 Task 11: without --tracking-model a plant run predicts with the installed MuJoCo model; its summary records
    # that file as the container sees it. The run is done while the file is unchanged, and runs again when it changes
    runs = pc.runs_of(pc.load_corpus(corpus(tmp_path)), seeds=1, mode="realistic", kinds=("f2",))
    assert all("--tracking-model" not in r["args"] for r in runs)
    rel = f"experiment_runs/test_plant_campaign_{tmp_path.name}/install/tracking_model_mujoco.yaml"
    model = pc.ROOT / rel
    model.parent.mkdir(parents=True, exist_ok=True)
    model.write_text("turn_offset: 0.01\n")
    try:
        done = tmp_path / "out" / runs[0]["name"]
        done.mkdir(parents=True)
        (done / "summary.json").write_text(json.dumps({"args": runs[0]["args"], "tracking_model": f"{pc.WORKSPACE}/{rel}",
                                                       "tracking_model_sha256": pc.file_sha256(model)}))
        others = [r["name"] for r in runs[1:]]  # the corpus's other cells: not run yet
        assert [r["name"] for r in pc.pending(runs, tmp_path / "out")] == others
        model.write_text("turn_offset: 0.02\n")
        assert [r["name"] for r in pc.pending(runs, tmp_path / "out")] == [runs[0]["name"]] + others
    finally:
        shutil.rmtree(model.parent.parent)
