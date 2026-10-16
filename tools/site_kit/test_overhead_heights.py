"""Overhead member heights from the robot's highest points (spec Section 4).

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


def test_h2_lowest_candidate_already_passes():
    result = oh.overhead(tops(1.20, lowered=0.80, other=0.90))
    assert result["h2_bottom_m"] == pytest.approx(1.05)
    assert result["h2_passing"] == "lowered"


def test_h2_raised_to_lowest_candidate():
    result = oh.overhead(tops(1.20, lowered=1.00, other=1.05))
    assert result["h2_bottom_m"] == pytest.approx(1.10)
    assert result["h2_passing"] == "lowered"


def test_h2_conflict_raises():
    with pytest.raises(oh.HeightConflict, match="lowered"):
        oh.overhead(tops(1.20, lowered=1.15, other=1.30))


def test_h2_conflict_without_candidates_raises():
    with pytest.raises(oh.HeightConflict):
        oh.overhead(tops(1.20))


def test_tops_are_passed_through():
    given = tops(1.20, lowered=0.80)
    assert oh.overhead(given)["tops_m"] == given
