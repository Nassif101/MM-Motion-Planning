"""Overhead member heights from the robot's highest points (spec Section 4; H2 dropped).

Run: python3 -m pytest tools/site_kit -q
"""
import pytest

import overhead_heights as oh


def tops(carry, **candidates):
    return {"home": 1.565, "vertical_carry": 1.92, "carry": carry, **candidates}


def test_h1_is_carry_top_plus_clearance():
    result = oh.overhead(tops(1.20, lowered=0.80))
    assert result["h1_bottom_m"] == pytest.approx(1.30)
    assert result["h3_bottom_m"] == 2.40


def test_h1_conflict_raises():
    with pytest.raises(oh.HeightConflict, match="1.5"):
        oh.overhead(tops(1.50, lowered=0.80))


def test_no_h2_in_output():
    assert not [key for key in oh.overhead(tops(1.20)) if key.startswith("h2")]


def test_tops_are_passed_through():
    given = tops(1.20, lowered=0.80)
    assert oh.overhead(given)["tops_m"] == given
