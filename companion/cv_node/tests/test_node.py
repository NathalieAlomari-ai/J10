"""End-to-end through CVNode, with a fake writer (records calls, touches no real shared
memory) and SyntheticFrameSource (no camera hardware). Mirrors the dependency-injection
pattern mavlink_bridge's tests use for MavlinkBridge."""

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np
import pytest

from cv_node.camera import SyntheticFrameSource
from cv_node.config import CVNodeConfig
from cv_node.node import CVNode


@dataclass
class FakeWriter:
    calls: List[Tuple[float, float, float, float, bool]] = field(default_factory=list)
    closed: bool = False

    def write(self, vx: float, vy: float, vz: float, yaw_rate: float, valid: bool = True) -> None:
        self.calls.append((vx, vy, vz, yaw_rate, valid))

    def close(self) -> None:
        self.closed = True


def _blank_frame(h=120, w=120) -> np.ndarray:
    return np.full((h, w), 128, dtype=np.uint8)


def _stripes(h=120, w=40, period=8) -> np.ndarray:
    # See the identical helper's docstring in test_obstacle_avoidance.py: coarse stripes,
    # not a single-pixel checkerboard, for a clean/unambiguous Canny response.
    x = np.arange(w)
    pattern = ((x // max(period // 2, 1)) % 2 * 255).astype(np.uint8)
    return np.tile(pattern, (h, 1))


def make_node(frames, **config_overrides) -> tuple[CVNode, FakeWriter]:
    config = CVNodeConfig(write_rate_hz=1000.0, roi_top_frac=0.0, roi_bottom_frac=1.0, **config_overrides)
    writer = FakeWriter()
    source = SyntheticFrameSource(frames)
    node = CVNode(config, frame_source=source, writer=writer)
    return node, writer


def test_run_writes_one_command_per_iteration():
    node, writer = make_node([_blank_frame()])
    node.run(max_iterations=5)
    assert len(writer.calls) == 5
    assert all(call[4] is True for call in writer.calls)  # valid=True on every real tick


def test_blank_scene_commands_forward_center():
    node, writer = make_node([_blank_frame()], clear_margin=0.0)
    node.run(max_iterations=3)
    vx, vy, vz, yaw_rate, valid = writer.calls[-1]
    assert vx > 0.0
    assert yaw_rate == 0.0


def test_camera_failure_skips_the_tick_without_crashing():
    node, writer = make_node([])  # SyntheticFrameSource with no frames -> read() is None
    node.run(max_iterations=3)
    assert writer.calls == []  # nothing written -- bridge's staleness failsafe covers this


def test_smoothing_ramps_into_a_step_change_instead_of_jumping():
    # One tick on a blank (straight-ahead, yaw_rate=0) scene to establish a baseline, then
    # switch to a scene that commands a hard turn. The very next output should move
    # *toward* the new target, not land on it in one step -- that's what EMA smoothing is
    # for. (The smoother's cold-start behavior -- no damping on the first-ever sample -- is
    # covered separately in test_smoothing.py; this test is about a change mid-flight.)
    blank = _blank_frame()
    turning = _blank_frame()
    # Obstacle spans left AND center, right is clear -- an unambiguous "turn right" case.
    # (A dense left zone alone, with center still clear, correctly stays "go straight":
    # a peripheral obstacle the vehicle isn't heading toward doesn't warrant a turn --
    # that's the hysteresis logic in decide() working as intended, not a case that
    # exercises smoothing on a step change.)
    turning[:, 0:80] = _stripes(120, 80)

    config_overrides = dict(clear_margin=0.0, ema_alpha=0.3, blur_ksize=1)
    node, writer = make_node([blank], **config_overrides)
    node.run(max_iterations=1)
    assert writer.calls[0][3] == 0.0  # baseline: straight, no yaw

    node.frame_source = SyntheticFrameSource([turning])
    node.run(max_iterations=1)
    target_yaw = node.config.turn_yaw_rate_rps
    stepped_yaw = writer.calls[1][3]

    assert 0.0 < stepped_yaw < target_yaw


def test_close_writes_invalid_marker_then_closes_everything():
    node, writer = make_node([_blank_frame()])
    node.run(max_iterations=1)
    node.close()
    assert writer.closed is True
    vx, vy, vz, yaw_rate, valid = writer.calls[-1]
    assert valid is False
    assert (vx, vy, vz, yaw_rate) == (0.0, 0.0, 0.0, 0.0)


def test_stop_halts_the_loop():
    node, writer = make_node([_blank_frame()])
    node.stop()
    node.run()  # would spin forever at max_iterations=None if stop() didn't take effect
    assert writer.calls == []
