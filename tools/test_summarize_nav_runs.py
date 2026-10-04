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
