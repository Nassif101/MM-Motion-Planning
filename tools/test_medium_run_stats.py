import medium_run_stats as mrs


def test_side_tracker_counts_flips_through_sideless_labels():
    # as mobile_manipulator_medium's ClosedLoop.SideTrackerCountsFlipsThroughSidelessLabels
    sides = mrs.SideTracker()
    for h in ("comfort", "right", "right+lowered", "lowered", "comfort", "left", "left", "right"):
        sides.observe(h)
    assert sides.first == "right" and sides.switches == 2 and sides.sequence == ["right", "left", "right"]


def test_torque_peaks_by_absolute_value_and_limit():
    samples = [[0, 1, -2, 0, 0, 0], [0, -3, 1, 0, 0, 0]]
    peaks = mrs.torque_peaks(samples, lambda q: [10.0 * v for v in q], [None, 60.0, 40.0, 24.0, 18.0, 12.0])
    assert peaks["peak_nm"] == [0.0, 30.0, 20.0, 0.0, 0.0, 0.0]
    assert peaks["ratio"] == [None, 0.5, 0.5, 0.0, 0.0, 0.0]


def test_medium_stats_from_the_window():
    cycles = [{"total_s": 0.01 * i, "capped": i == 7, "full_multistart": i in (1, 5)} for i in range(1, 11)]
    plans = [{"status": s, "homotopy": h} for s, h in (("ok", "comfort"), ("ok", "left"), ("infeasible", "left+lowered"),
                                                        ("ok", "right"))]
    stats = mrs.medium_stats(cycles, plans, [[0.0] * 6], lambda q: [1.0] * 6, [None, 2.0, 2.0, 2.0, 2.0, 2.0])
    assert stats["cycles"] == 10 and stats["capped_cycles"] == 1 and stats["full_multistarts"] == 2
    assert abs(stats["total_s"]["p99"] - 0.10) < 1e-12 and abs(stats["total_s"]["median"] - 0.05) < 1e-12
    assert stats["plans"] == 4 and stats["plan_statuses"] == {"ok": 3, "infeasible": 1}
    assert stats["first_side"] == "left" and stats["homotopy_switches_after_first"] == 1
    assert stats["torque"]["ratio"][1] == 0.5 and stats["arm_samples"] == 1
    # B5 alone: no medium messages
    empty = mrs.medium_stats([], [], [], lambda q: [0.0] * 6, [None] * 6)
    assert empty["total_s"] is None and empty["torque"] is None and empty["side_sequence"] == []
