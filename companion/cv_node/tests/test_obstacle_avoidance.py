"""The decision logic (`decide`) is tested directly against hand-built ZoneDensities —
no synthetic images needed for that part, since it's a pure function of three numbers.
`compute_zone_edge_densities` gets its own tests against real (tiny, synthetic) frames,
so the two responsibilities — "score a frame" and "decide from scores" — are verified
independently, matching how obstacle_avoidance.py itself separates them.
"""

import numpy as np
import pytest

from cv_node.config import CVNodeConfig
from cv_node.obstacle_avoidance import ZoneDensities, compute_zone_edge_densities, decide


def make_config(**overrides) -> CVNodeConfig:
    return CVNodeConfig(**overrides)


# -- decide(): pure decision logic --------------------------------------------------------

def test_clear_center_goes_straight():
    config = make_config()
    d = ZoneDensities(left=0.05, center=0.01, right=0.05)
    result = decide(d, config)
    assert result.blocked is False
    assert result.chosen_zone == "center"
    assert result.vx == pytest.approx(config.cruise_vx_mps)
    assert result.yaw_rate == 0.0


def test_clear_left_turns_left_with_negative_yaw_rate():
    config = make_config(clear_margin=0.02)
    d = ZoneDensities(left=0.01, center=0.10, right=0.10)
    result = decide(d, config)
    assert result.blocked is False
    assert result.chosen_zone == "left"
    assert result.yaw_rate == pytest.approx(-config.turn_yaw_rate_rps)
    assert result.vx == pytest.approx(config.cruise_vx_mps * config.turn_vx_scale)


def test_clear_right_turns_right_with_positive_yaw_rate():
    config = make_config(clear_margin=0.02)
    d = ZoneDensities(left=0.10, center=0.10, right=0.01)
    result = decide(d, config)
    assert result.blocked is False
    assert result.chosen_zone == "right"
    assert result.yaw_rate == pytest.approx(config.turn_yaw_rate_rps)


def test_everything_blocked_stops_instead_of_picking_least_bad():
    config = make_config(blocked_density_threshold=0.12)
    d = ZoneDensities(left=0.15, center=0.20, right=0.13)
    result = decide(d, config)
    assert result.blocked is True
    assert result.chosen_zone == "blocked"
    assert (result.vx, result.vy, result.vz, result.yaw_rate) == (0.0, 0.0, 0.0, 0.0)


def test_razor_thin_margin_prefers_straight_over_turning():
    # left is numerically the minimum, but only by less than clear_margin -- should NOT
    # turn on noise this small.
    config = make_config(clear_margin=0.02)
    d = ZoneDensities(left=0.049, center=0.05, right=0.06)
    result = decide(d, config)
    assert result.chosen_zone == "center"
    assert result.yaw_rate == 0.0


def test_margin_just_over_threshold_does_turn():
    config = make_config(clear_margin=0.02)
    d = ZoneDensities(left=0.02, center=0.05, right=0.06)  # gap = 0.03 > 0.02
    result = decide(d, config)
    assert result.chosen_zone == "left"


def test_vz_is_always_zero_this_heuristic_makes_no_altitude_decisions():
    config = make_config()
    for d in (
        ZoneDensities(0.01, 0.01, 0.01),
        ZoneDensities(0.5, 0.5, 0.5),
        ZoneDensities(0.01, 0.5, 0.5),
    ):
        assert decide(d, config).vz == 0.0


# -- compute_zone_edge_densities(): scoring real frames -------------------------------------

def _blank_frame(h=120, w=120, value=128) -> np.ndarray:
    return np.full((h, w), value, dtype=np.uint8)


def _stripes(h: int, w: int, period: int = 8) -> np.ndarray:
    """Alternating solid vertical stripes, `period // 2` px wide each — a strong, clean
    edge signal for Canny. Deliberately NOT a single-pixel checkerboard: a per-pixel
    alternating pattern sits right at the Sobel kernel's aliasing limit and produces a
    sparse, inconsistent response (a real quirk of that specific pattern, not something
    compute_zone_edge_densities should be judged against). Stripes well above the kernel's
    ~1px radius give an unambiguous, strongly textured region instead."""
    x = np.arange(w)
    pattern = ((x // max(period // 2, 1)) % 2 * 255).astype(np.uint8)
    return np.tile(pattern, (h, 1))


def test_blank_frame_has_near_zero_density_everywhere():
    config = make_config(roi_top_frac=0.0, roi_bottom_frac=1.0)
    densities = compute_zone_edge_densities(_blank_frame(), config)
    assert densities.left == pytest.approx(0.0, abs=1e-6)
    assert densities.center == pytest.approx(0.0, abs=1e-6)
    assert densities.right == pytest.approx(0.0, abs=1e-6)


def test_a_textured_zone_reads_denser_than_blank_zones():
    config = make_config(roi_top_frac=0.0, roi_bottom_frac=1.0, blur_ksize=1)
    frame = _blank_frame(h=120, w=120)
    frame[:, 80:120] = _stripes(120, 40)  # textured patch in the right zone only

    densities = compute_zone_edge_densities(frame, config)
    assert densities.right > densities.left
    assert densities.right > densities.center
    assert densities.left == pytest.approx(0.0, abs=1e-6)
    # Not exactly 0.0: Canny's Sobel kernel has a radius, so the hard blank/stripe boundary
    # sitting right on the center/right zone edge can legitimately bleed a pixel or two into
    # "center" too — correct edge-detector behavior, not a scoring bug. The real invariant
    # is that center (at most a boundary's worth of bleed) reads much sparser than right
    # (the whole striped interior), not that it's exactly zero.
    assert densities.center < densities.right * 0.25


def test_roi_crop_excludes_texture_outside_it():
    # Same stripes, but confined to the top 10% of the frame -- with an ROI that starts at
    # 50%, it must not affect the score at all.
    config = make_config(roi_top_frac=0.5, roi_bottom_frac=1.0, blur_ksize=1)
    frame = _blank_frame(h=120, w=120)
    frame[0:12, :] = _stripes(12, 120)  # entirely above the ROI

    densities = compute_zone_edge_densities(frame, config)
    assert densities.left == pytest.approx(0.0, abs=1e-6)
    assert densities.center == pytest.approx(0.0, abs=1e-6)
    assert densities.right == pytest.approx(0.0, abs=1e-6)


def test_rejects_color_frames():
    config = make_config()
    color_frame = np.zeros((120, 120, 3), dtype=np.uint8)
    with pytest.raises(ValueError):
        compute_zone_edge_densities(color_frame, config)


def test_rejects_inverted_roi_bounds():
    config = make_config(roi_top_frac=0.9, roi_bottom_frac=0.3)
    with pytest.raises(ValueError):
        compute_zone_edge_densities(_blank_frame(), config)


# -- end-to-end: a real frame through both stages -------------------------------------------

def test_full_pipeline_steers_away_from_a_textured_left_zone():
    config = make_config(
        roi_top_frac=0.0, roi_bottom_frac=1.0, blur_ksize=1, clear_margin=0.0,
    )
    frame = _blank_frame(h=120, w=120)
    frame[:, 0:40] = _stripes(120, 40)  # obstacle-like texture on the left

    densities = compute_zone_edge_densities(frame, config)
    decision = decide(densities, config)
    assert decision.chosen_zone in ("center", "right")
    assert decision.chosen_zone != "left"
