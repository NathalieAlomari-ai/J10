"""cv_node — the Pi Camera Module 3 vision node for J10 PT1.

Grid-based, non-ML obstacle avoidance: split the frame into Left/Center/Right zones,
score each zone's edge density, steer toward whichever zone reads clearest, and write the
resulting (vx, vy, vz, yaw_rate) to shared memory for `mavlink_bridge` to pick up. See
``README.md`` in this directory for the heuristic, its known limitations, and how it fits
the rest of PT1.

This package owns exactly one job: turn a camera frame into a velocity command. It knows
nothing about MAVLink, Serial, or the flight controller — that's `mavlink_bridge`, on the
other side of the `j10_shm_protocol` adapter.
"""

from .config import CVNodeConfig
from .node import CVNode
from .obstacle_avoidance import NavDecision, ZoneDensities, compute_zone_edge_densities, decide

__all__ = [
    "CVNode",
    "CVNodeConfig",
    "NavDecision",
    "ZoneDensities",
    "compute_zone_edge_densities",
    "decide",
]

__version__ = "0.1.0"
