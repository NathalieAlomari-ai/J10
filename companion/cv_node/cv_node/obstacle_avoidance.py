"""The actual obstacle-avoidance logic: grid-based edge density, no ML, no frame history.

Both functions here are pure (frame/densities in, a dataclass out) and framework-agnostic
— no camera, no shared memory, no threading — specifically so the heuristic itself is
testable against plain numpy arrays, independent of any hardware.

The heuristic, and its real limitation
---------------------------------------
Split the frame into three vertical zones (Left / Center / Right), run Canny edge
detection in each, and treat *edge-pixel density* as a proxy for "how close/cluttered is
whatever the camera is looking at in this direction" — a textured, close-up obstacle
produces many edges; an open hallway or blank distant wall produces few. Steer toward
whichever zone reads clearest.

This is a cheap, well-known reactive heuristic (related to vanishing-point / corridor-
following navigation), and it is exactly that: a heuristic, not a distance measurement.
It has a real failure mode — a *close, blank, low-texture* surface (a plain wall
immediately ahead) can score as "clear" because it has few edges, while a *distant,
highly textured* floor or carpet can score as "cluttered". It is not a substitute for the
BOM's TFmini-S / MTF-01 range sensors, which is exactly why the safety envelope and the
final failsafe live in `mavlink_bridge`, not here — this node is one input, clamped and
failsafed independently downstream, matching the project's standing safety rule of
independent, hardware-verifiable enforcement (see docs/ARCHITECTURE.md §"Safety").

The alternative the requirements doc mentions — optical flow (time-to-contact via flow
magnitude) — is a more physically-grounded signal for exactly this problem, but needs
frame-to-frame feature tracking, which costs meaningfully more CPU on a Pi Zero 2W for a
first pass. Kept out of v1 deliberately; `compute_zone_edge_densities` is the only
function that would need a sibling (e.g. `compute_zone_flow_magnitudes`) to add it later
without touching `decide()` or anything upstream.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .config import CVNodeConfig


@dataclass(frozen=True)
class ZoneDensities:
    """Edge-pixel fraction (0..1) in each zone of the region of interest. Higher = more
    edges = read as more cluttered/close by the heuristic above."""

    left: float
    center: float
    right: float

    def as_dict(self) -> dict[str, float]:
        return {"left": self.left, "center": self.center, "right": self.right}


@dataclass(frozen=True)
class NavDecision:
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    blocked: bool
    chosen_zone: str  # "left" | "center" | "right" | "blocked"
    densities: ZoneDensities


def compute_zone_edge_densities(gray_frame: "np.ndarray", config: CVNodeConfig) -> ZoneDensities:
    """Crop to the configured ROI, blur, run Canny, and return each zone's edge density.

    ``gray_frame`` must be a single-channel (grayscale) frame — camera backends convert
    to grayscale before this is called, since color carries nothing this heuristic uses
    and converting it here on every call would just be wasted CPU.
    """
    if gray_frame.ndim != 2:
        raise ValueError(f"expected a single-channel grayscale frame, got shape {gray_frame.shape}")

    h, w = gray_frame.shape
    top = int(h * config.roi_top_frac)
    bottom = int(h * config.roi_bottom_frac)
    if bottom <= top:
        raise ValueError(
            f"roi_top_frac ({config.roi_top_frac}) must be less than "
            f"roi_bottom_frac ({config.roi_bottom_frac})"
        )
    roi = gray_frame[top:bottom, :]

    if config.blur_ksize > 1:
        roi = cv2.GaussianBlur(roi, (config.blur_ksize, config.blur_ksize), 0)
    edges = cv2.Canny(roi, config.canny_low, config.canny_high)

    zone_w = edges.shape[1] // 3
    zones = {
        "left": edges[:, :zone_w],
        "center": edges[:, zone_w : 2 * zone_w],
        "right": edges[:, 2 * zone_w :],
    }

    def density(zone: "np.ndarray") -> float:
        return float(np.count_nonzero(zone)) / zone.size if zone.size else 0.0

    return ZoneDensities(density(zones["left"]), density(zones["center"]), density(zones["right"]))


def decide(densities: ZoneDensities, config: CVNodeConfig) -> NavDecision:
    """Turn zone densities into a velocity command. Pure function — no camera, no I/O — so
    every branch below is exercised directly in tests against hand-built ``ZoneDensities``,
    with no synthetic-image plumbing required."""
    scores = densities.as_dict()

    if min(scores.values()) >= config.blocked_density_threshold:
        # Every zone reads as cluttered — don't pick the "least bad" option and push
        # forward into it. Stop and hold. (vz is always 0.0 here: this heuristic only
        # judges the horizontal plane; altitude isn't this node's decision to make.)
        return NavDecision(0.0, 0.0, 0.0, 0.0, blocked=True, chosen_zone="blocked", densities=densities)

    # Compare each side directly against center rather than taking a global min(): a
    # global min ties-break on dict/key order, which would silently turn "left" on a
    # perfectly uniform scene (all three zones equal) just because "left" happens to be
    # the first key — exactly the case (an open, blank hallway) that most needs to default
    # to straight. Only turn when a side actually beats center by more than the hysteresis
    # margin; a tie or a razor-thin edge is not evidence that turning is warranted.
    left_gap = scores["center"] - scores["left"]
    right_gap = scores["center"] - scores["right"]

    if left_gap > config.clear_margin or right_gap > config.clear_margin:
        best_zone = "left" if left_gap >= right_gap else "right"
    else:
        best_zone = "center"

    if best_zone == "center":
        return NavDecision(
            config.cruise_vx_mps, 0.0, 0.0, 0.0,
            blocked=False, chosen_zone="center", densities=densities,
        )

    # Body-frame convention (matches j10_shm_protocol): yaw_rate positive = clockwise
    # viewed from above = turning right. Assumes the camera is mounted forward-facing and
    # unmirrored, so the image's left column corresponds to the vehicle's left — verify
    # this on the bench (a flipped mount silently inverts every turn decision) before
    # trusting it in the air.
    yaw_rate = config.turn_yaw_rate_rps if best_zone == "right" else -config.turn_yaw_rate_rps
    return NavDecision(
        config.cruise_vx_mps * config.turn_vx_scale, 0.0, 0.0, yaw_rate,
        blocked=False, chosen_zone=best_zone, densities=densities,
    )
