"""Scoring checks for tools/summarize_nav_runs.py (run: python3 -m pytest tools/test_summarize_nav_runs.py)."""
import summarize_nav_runs as s


def drive(status="succeeded", contact=False, stats=None, task="navigate_to_pose"):
    return {"scenario": {"name": "gate", "task": task, "footprint_profile": "home"},
            "contacts": {"contact": contact},
            "footprint_stats": stats,
            "task": {"status": status, "time_s": 25.0, "path_length_m": 4.65, "final_position_error_m": 0.05,
                     "cross_track_m": {"p95": 0.01}, "min_footprint_clearance_to_static_map_m": 0.2,
                     "recoveries": 0, "cpu_percent_of_core": {}, "collision_monitor_activations": [],
                     "started_s": 10.0, "finished_s": 35.0}}


def column(row, name):
    return row[s.COLUMNS.index(name)]


def test_a_drive_with_contact_is_never_a_success():
    # Nav2 can report success while the panel touched something (a late poll or a racing cancel).
    row = s.row("gate", "rpp", [drive(contact=True), drive()])
    assert column(row, "Success") == "1/2"
    assert column(row, "Failures") == "collision 1"
    assert column(row, "Contact") == "1"


def test_plan_only_runs_show_no_footprint_updates():
    plan = drive(stats={"publish_times_s": [1.0], "compute_us": {"mean": 50.0, "max": 100.0}}, task="compute_path")
    del plan["task"]["started_s"], plan["task"]["finished_s"]
    assert s.footprint_updates([plan]) == "-"


def test_footprint_updates_count_publishes_inside_drives():
    run = drive(stats={"publish_times_s": [5.0, 12.0, 40.0], "compute_us": {"mean": 50.0, "max": 100.0}})
    assert s.footprint_updates([run]) == "1"


def wholebody_drive(**changes):
    """A B5 run: a mission scenario driven as one navigate task, with the MPC stats and 3D clearance."""
    run = drive(task="mission")
    run.update({"controller": "wholebody", "references": "none", "footprint": {"mode": "wholebody"},
                "mpc": {"cycles": 1200, "deadline_misses": 3, "near_misses": 11,
                        "total_s": {"mean": 0.011, "p99": 0.0234, "max": 0.061}},
                "clearance_3d_m": 0.137, "clearance_3d_link": "wrist_3_link", "clearance_3d_obstacle": "beam"})
    run.update(changes)
    return run


def test_summarize_wholebody_columns():
    run = wholebody_drive()
    row = s.row("gate", "wholebody", [run, wholebody_drive(mpc={"cycles": 1000, "deadline_misses": 0, "near_misses": 2,
                                                                 "total_s": {"p99": 0.030}},
                                                           clearance_3d_m=0.25)])
    columns = s.columns(wide=True)
    assert columns[-4:] == ("MPC p99 ms", "misses", "near-misses", "3D clearance m")
    assert len(row) == len(columns)
    assert row[columns.index("MPC p99 ms")] == "26.7 (23.4-30.0)"
    assert row[columns.index("misses")] == "3"
    assert row[columns.index("near-misses")] == "13"
    assert row[columns.index("3D clearance m")] == "0.194 (0.137-0.250)"


def test_baseline_tables_do_not_grow_wholebody_columns():
    assert s.columns(wide=False) == s.COLUMNS
    assert not s.has_wholebody([drive()])
    row = s.row("gate", "rpp", [drive()])
    assert len(row) == len(s.COLUMNS)
    assert s.has_wholebody([wholebody_drive()])


def test_a_wholebody_mission_drive_belongs_to_the_drive_table_and_a_reference_run_is_labelled():
    run = wholebody_drive(references="scripted")
    assert not s.is_mission_table(run)
    assert s.controller_label(run) == "wholebody [ref]"
    assert s.controller_label(wholebody_drive()) == "wholebody"
