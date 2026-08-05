from horizon_sweep import _sweep


def test_sweep_reports_monotonic_lookahead_and_finite_rms():
    rows = _sweep(seeds=1, wall_stiffness=0.0, wall_damping=0.0)
    lookaheads = [row["lookahead_s"] for row in rows]
    assert lookaheads == sorted(lookaheads)
    for row in rows:
        assert row["two_rate_rms_mm_mean"] > 0.0
        assert row["anticipatory_rms_mm_mean"] > 0.0
