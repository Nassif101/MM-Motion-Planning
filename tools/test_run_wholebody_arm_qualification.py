import math

import run_wholebody_arm_qualification as q


def test_vertical_right_pins_the_task_6_mirror_solution():
    # carry_pose_search --mirror-of vertical_carry (Phase 4B-A Task 6): the sign flip has the mirrored panel pose.
    assert q.VERTICAL_RIGHT == [-q.VERTICAL[0]] + q.VERTICAL[1:]


def test_vertical_right_plan_covers_hold_transitions_and_stream():
    runs = q.vertical_right_plan(repeats=3, parts=["hold", "transitions", "stream"], schedules=["compact", "extended"])
    names = [r["name"] for r in runs]
    # hold: still, compact, extended x 3; transitions: 2 legs x (still, compact) x 3; stream x 3.
    assert len(runs) == 9 + 12 + 3
    assert len(set(names)) == len(names)
    holds = [r for r in runs if r["meta"]["part"] == "hold"]
    assert {r["meta"]["mode"] for r in holds} == {"still", "compact", "extended"}
    assert all(r["start"] == q.HOME and r["goal"] == q.VERTICAL_RIGHT and r["duration"] == q.REFERENCE_DURATION
               for r in holds)
    legs = [r for r in runs if r["meta"]["part"] == "transition"]
    assert {(tuple(r["start"]), tuple(r["goal"])) for r in legs} == {
        (tuple(q.CARRY), tuple(q.VERTICAL_RIGHT)), (tuple(q.VERTICAL_RIGHT), tuple(q.CARRY))}
    assert {r["meta"]["mode"] for r in legs} == {"still", "compact"}
    assert all(r["duration"] == q.limit_duration(r["start"], r["goal"]) for r in legs)
    driven = [r for r in runs if r["meta"].get("mode") in ("compact", "extended")]
    assert all("--drive-during-motion" in r["extra"] for r in driven)
    streams = [r for r in runs if r["meta"]["part"] == "stream"]
    assert all(r["start"] == q.VERTICAL_RIGHT for r in streams) and len(streams) == 3


def test_pose_table_judges_against_the_carry_acceptance():
    good = {"exit_code": 0, "report": {"max_path_error_rad": [0.03], "hold_max_error": [0.01],
                                       "max_reference_velocity_radps": [0.5], "position_jumps": 0},
            "physical": {"watchdog_or_fault": False, "max_base_tilt_degrees": 0.2, "min_panel_bottom_m": 0.7,
                         "max_panel_penetration_m": 0.0, "max_base_speed_mps": 0.3, "max_saturation_fraction": 0.0,
                         "max_wall_gap_between_physics_samples_s": 0.02}}
    bad = {**good, "report": {**good["report"], "max_path_error_rad": [0.12]}}
    results = {"vertical_right": {"vcr1-hold-still-r1": good, "vcr1-hold-still-r2": bad, "other-hold-still-r1": bad}}
    table = q.pose_table(results, "vcr1", "vertical_right").splitlines()
    assert len(table) == 2
    assert table[0].split()[1] == "ok" and table[1].split()[1] == "FAIL"


def test_each_measurement_records_the_gravity_feedforward(monkeypatch, tmp_path):
    monkeypatch.setattr(q, "RESULTS", tmp_path / "results.json.gz")
    monkeypatch.setattr(q, "RECORDINGS", tmp_path)
    facts = {"watchdog_or_fault": False, "max_base_tilt_degrees": 0.0, "min_panel_bottom_m": 1.0}
    monkeypatch.setattr(q, "physical", lambda path: facts)
    qualifier = q.Qualifier.__new__(q.Qualifier)
    qualifier.prefix, qualifier.results, qualifier.gravity_feedforward = "gffon", {}, True
    qualifier.motion_feedforward = qualifier.setpoint_preview = False
    qualifier.unity = lambda *arguments: None
    qualifier.tool = lambda *arguments: ({"max_path_error_rad": [0.01]}, 0)
    assert qualifier.measure("carry", "hold-still-r1")["gravity_feedforward"] is True


def test_each_measurement_records_the_motion_feedforward(monkeypatch, tmp_path):
    monkeypatch.setattr(q, "RESULTS", tmp_path / "results.json.gz")
    monkeypatch.setattr(q, "RECORDINGS", tmp_path)
    facts = {"watchdog_or_fault": False, "max_base_tilt_degrees": 0.0, "min_panel_bottom_m": 1.0}
    monkeypatch.setattr(q, "physical", lambda path: facts)
    qualifier = q.Qualifier.__new__(q.Qualifier)
    qualifier.prefix, qualifier.results = "mffon", {}
    qualifier.gravity_feedforward, qualifier.motion_feedforward, qualifier.setpoint_preview = True, True, True
    qualifier.unity = lambda *arguments: None
    qualifier.tool = lambda *arguments: ({"max_path_error_rad": [0.01]}, 0)
    entry = qualifier.measure("carry", "hold-still-r1")
    assert entry["motion_feedforward"] is True and entry["setpoint_preview"] is True


def test_physical_reports_the_time_aligned_tracking_error(tmp_path):
    # The command applied at a step is stamped one step earlier: row k's actual is compared with row k+1's desired.
    header = ("sim_time,wall_time,state,joint,desired_position,actual_position,position_error,command_age,"
              "base_tilt_degrees,panel_bottom,panel_penetration_m,base_velocity,estimated_saturated\n")
    rows = "".join(f"{0.02 * k},{0.02 * k},EXTERNAL_CONTROL,elbow,{0.01 * k},{0.01 * k - 0.001},0.0,0.01,0,1,0,0,0\n"
                   for k in range(5))
    path = tmp_path / "run.csv"
    path.write_text(header + rows)
    facts = q.physical(path)
    assert abs(facts["aligned_error_p95_rad"] - 0.011) < 1e-9
