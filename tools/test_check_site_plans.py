"""Pure-logic checks for tools/check_site_plans.py (run: python3 -m pytest tools/test_check_site_plans.py)."""
import pytest

import check_site_plans as c


def task(rates):
    return {"planners": {name: {"success_rate": rate,
                                "runs": [{"length_m": 12.34, "total_turn_deg": 95.0, "min_static_clearance_m": 0.8}]}
                         for name, rate in rates.items()}}


def test_expectations_parse_flow_lines_and_comments():
    text = ("# header\nexpectations:\n"
            "v2_a_nav: {home: true, vertical_carry: false, carry: true}  # why\n\n"
            "v2_b_nav: {home: false, vertical_carry: false, carry: false}\n")
    assert c.parse_expectations(text) == {
        "v2_a_nav": {"home": True, "vertical_carry": False, "carry": True},
        "v2_b_nav": {"home": False, "vertical_carry": False, "carry": False}}


@pytest.mark.parametrize("line", ["v2_a: {home: true, carry: true}",
                                  "v2_a: {home: yes, vertical_carry: true, carry: true}",
                                  "v2_a: {home: true, vertical_carry: true, carry: true, base_only: true}",
                                  "not an entry"])
def test_bad_expectation_lines_are_refused(line):
    with pytest.raises(ValueError):
        c.parse_expectations(line)


def test_planned_needs_every_planner_to_succeed():
    assert c.planned(task({"GridBased": 1.0, "Lattice": 1.0}))
    assert not c.planned(task({"GridBased": 1.0, "Lattice": 0.0}))
    assert not c.planned({"planners": {}})


def test_mismatches_list_only_differences():
    expected = {"v2_a": {"home": True, "vertical_carry": False, "carry": True}}
    observed = {"v2_a": {"home": True, "vertical_carry": True}}
    assert c.mismatches(expected, observed) == [("v2_a", "vertical_carry", False, True)]


def test_scenario_names_read_the_file_order():
    text = "site: x\nscenarios:\n  v2_one_nav:\n    description: a\n  v2_two_mission:\n    task: mission\n"
    assert c.scenario_names(text) == ["v2_one_nav", "v2_two_mission"]


def test_describe_shows_each_planner():
    cells = c.describe(task({"GridBased": 1.0}))
    assert cells["GridBased"] == "1.00 12.3m turn 95 clr 0.8"
