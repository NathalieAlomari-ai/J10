"""Runtime configuration for the CV node. Same pattern as ``mavlink_bridge.config``: a
plain dataclass, overridable by ``J10_CV_*`` environment variables, no config-file library.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, fields
from typing import Any


def _bool(v: str) -> bool:
    return v.strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class CVNodeConfig:
    # --- shared-memory adapter (must match the bridge's shm_name to talk to it) ---------
    shm_name: str = "j10_cv_cmd"

    # --- camera -----------------------------------------------------------------------
    # "picamera2" (Pi Camera Module 3, real hardware — see README for the apt/venv setup
    # it needs), "opencv" (USB/dev webcam via cv2.VideoCapture, for bench-testing the
    # vision logic on a laptop before touching the Pi), or "synthetic" (tests only —
    # construct a SyntheticFrameSource directly, this backend name is not auto-creatable).
    camera_backend: str = "picamera2"
    opencv_device_index: int = 0
    # Deliberately low: this is a hardware-scaled capture size (the ISP/driver downsamples
    # for us), not a full-res frame we then shrink — keeps every downstream step cheap on
    # a Pi Zero 2W with no GPU-accelerated OpenCV build.
    frame_width: int = 320
    frame_height: int = 240

    # --- vision loop rate ---------------------------------------------------------------
    # Deliberately slower than mavlink_bridge's 20 Hz setpoint rate, mirroring the
    # two-rate cascade already established for the PC-side architecture
    # (docs/ARCHITECTURE.md §2): the slow layer decides *where to go*, the fast layer
    # (the bridge) guarantees the FC always has a fresh, bounded command regardless of
    # how long a vision tick takes. If this node stalls or crashes, the bridge's own
    # staleness failsafe (cv_stale_timeout_s, default 250 ms) takes over — nothing here
    # needs to coordinate with that directly.
    write_rate_hz: float = 8.0

    # --- region of interest (fraction of frame height, top=0.0 .. bottom=1.0) ------------
    # Crops out the ceiling (near the top) and the floor immediately under the drone (near
    # the bottom) before scoring zones, since neither is a useful "is this direction open"
    # signal and both add noise to the edge-density heuristic.
    roi_top_frac: float = 0.30
    roi_bottom_frac: float = 0.90

    # --- edge-density heuristic ---------------------------------------------------------
    blur_ksize: int = 5           # odd; Gaussian blur before Canny, cuts noise sensitivity
    canny_low: int = 60
    canny_high: int = 150
    # A zone's edge-pixel fraction at/above this reads as "close obstacle" rather than
    # "open space". This number is a starting point, not a calibrated constant — it
    # depends on the lens, lighting, and what the drone flies near; tune on the bench
    # against your actual indoor test space before trusting it in the air (see README).
    blocked_density_threshold: float = 0.12
    # Minimum density advantage a side zone needs over center before the node turns
    # instead of going straight — hysteresis against flicker when two zones are close.
    clear_margin: float = 0.02

    # --- output shaping ------------------------------------------------------------------
    cruise_vx_mps: float = 0.25       # forward speed on a clear center
    turn_vx_scale: float = 0.5        # forward speed multiplier *while* turning toward a side
    turn_yaw_rate_rps: float = 0.4    # yaw rate toward the clearer side zone
    # Exponential-moving-average smoothing applied to the *output* command, not the raw
    # per-frame decision — keeps a single noisy frame from producing a visible jerk.
    # 1.0 = no smoothing, smaller = smoother/slower to react.
    ema_alpha: float = 0.3

    log_level: str = "INFO"

    @classmethod
    def from_env(cls, prefix: str = "J10_CV_") -> "CVNodeConfig":
        kwargs: dict[str, Any] = {}
        for f in fields(cls):
            raw = os.environ.get(prefix + f.name.upper())
            if raw is None:
                continue
            if f.type in (int, "int"):
                kwargs[f.name] = int(raw)
            elif f.type in (float, "float"):
                kwargs[f.name] = float(raw)
            elif f.type in (bool, "bool"):
                kwargs[f.name] = _bool(raw)
            else:
                kwargs[f.name] = raw
        return cls(**kwargs)

    @property
    def write_period_s(self) -> float:
        return 1.0 / self.write_rate_hz
