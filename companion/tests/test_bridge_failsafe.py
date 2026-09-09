"""Exercises the failsafe decision logic (requirement: CV silence/crash -> zero-velocity
hover) and the setpoint encoding, without any real serial link or flight controller."""

import time
from dataclasses import dataclass
from typing import Optional
from unittest.mock import MagicMock

import pytest

from j10_shm_protocol import CVCommand
from mavlink_bridge.bridge import MavlinkBridge
from mavlink_bridge.config import BridgeConfig


@dataclass
class FakeCVReader:
    """Stands in for CVCommandReader so tests don't touch real /dev/shm."""

    command: Optional[CVCommand]

    def read(self) -> Optional[CVCommand]:
        return self.command


def make_bridge(cv_command: Optional[CVCommand], **config_overrides) -> MavlinkBridge:
    config = BridgeConfig(**config_overrides)
    bridge = MavlinkBridge(config, cv_reader=FakeCVReader(cv_command))
    # Bypass connect(): give it a fake mavfile-like master and a live FC heartbeat.
    bridge._master = MagicMock()
    bridge._master.target_system = 1
    bridge._master.target_component = 1
    bridge._last_fc_heartbeat_mono = time.monotonic()
    return bridge


def fresh_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, valid=True) -> CVCommand:
    return CVCommand(time.monotonic_ns(), vx, vy, vz, yaw_rate, valid)


def stale_command(age_s: float, vx=0.5) -> CVCommand:
    ts = time.monotonic_ns() - int(age_s * 1e9)
    return CVCommand(ts, vx, 0.0, 0.0, 0.0, True)


# -- failsafe engages -------------------------------------------------------------------

def test_no_cv_command_yields_zero_velocity_hover():
    bridge = make_bridge(cv_command=None)
    sp = bridge._compute_setpoint()
    assert sp.failsafe is True
    assert (sp.vx, sp.vy, sp.vz, sp.yaw_rate) == (0.0, 0.0, 0.0, 0.0)


def test_invalid_cv_command_yields_zero_velocity_hover():
    bridge = make_bridge(cv_command=fresh_command(vx=0.5, valid=False))
    sp = bridge._compute_setpoint()
    assert sp.failsafe is True
    assert sp.vx == 0.0


def test_stale_cv_command_yields_zero_velocity_hover():
    bridge = make_bridge(cv_command=stale_command(age_s=1.0), cv_stale_timeout_s=0.25)
    sp = bridge._compute_setpoint()
    assert sp.failsafe is True
    assert sp.vx == 0.0
    assert "stale" in sp.reason


def test_cv_command_just_inside_staleness_window_is_trusted():
    bridge = make_bridge(cv_command=stale_command(age_s=0.05, vx=0.3), cv_stale_timeout_s=0.25)
    sp = bridge._compute_setpoint()
    assert sp.failsafe is False
    assert sp.vx == pytest.approx(0.3)


def test_fc_link_timeout_yields_zero_velocity_hover_even_with_fresh_cv_command():
    bridge = make_bridge(cv_command=fresh_command(vx=0.5), fc_link_timeout_s=1.0)
    bridge._last_fc_heartbeat_mono = time.monotonic() - 5.0  # FC gone quiet
    sp = bridge._compute_setpoint()
    assert sp.failsafe is True
    assert "FC link" in sp.reason


# -- normal path, including safety clamps -------------------------------------------------

def test_fresh_valid_command_passes_through():
    bridge = make_bridge(cv_command=fresh_command(vx=0.1, vy=-0.1, vz=0.05, yaw_rate=0.2))
    sp = bridge._compute_setpoint()
    assert sp.failsafe is False
    assert sp.vx == pytest.approx(0.1)
    assert sp.vy == pytest.approx(-0.1)
    assert sp.vz == pytest.approx(0.05)
    assert sp.yaw_rate == pytest.approx(0.2)


def test_velocity_envelope_clamps_regardless_of_what_cv_node_requests():
    bridge = make_bridge(
        cv_command=fresh_command(vx=10.0, vy=-10.0, vz=10.0, yaw_rate=10.0),
        max_horizontal_speed_mps=0.6,
        max_vertical_speed_mps=0.4,
        max_yaw_rate_rps=0.6,
    )
    sp = bridge._compute_setpoint()
    assert sp.failsafe is False
    assert sp.vx == pytest.approx(0.6)
    assert sp.vy == pytest.approx(-0.6)
    assert sp.vz == pytest.approx(0.4)
    assert sp.yaw_rate == pytest.approx(0.6)


# -- send path: correct MAVLink call shape -------------------------------------------------

def test_send_setpoint_uses_velocity_and_yaw_rate_typemask_and_body_frame():
    from mavlink_bridge.bridge import _POSITION_TARGET_TYPEMASK_VELOCITY_AND_YAW_RATE

    bridge = make_bridge(cv_command=fresh_command(vx=0.2, vy=0.0, vz=0.0, yaw_rate=0.1))
    sp = bridge._compute_setpoint()
    bridge._send_setpoint(sp)

    call = bridge._master.mav.set_position_target_local_ned_send
    call.assert_called_once()
    args = call.call_args.args
    # time_boot_ms, target_system, target_component, coordinate_frame, type_mask,
    # x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
    _, target_system, target_component, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate = args
    assert target_system == 1
    assert target_component == 1
    assert coordinate_frame == bridge.config.coordinate_frame
    assert type_mask == _POSITION_TARGET_TYPEMASK_VELOCITY_AND_YAW_RATE
    assert (x, y, z) == (0.0, 0.0, 0.0)
    assert (vx, vy, vz) == pytest.approx((0.2, 0.0, 0.0))
    assert (afx, afy, afz) == (0.0, 0.0, 0.0)
    assert yaw == 0.0
    assert yaw_rate == pytest.approx(0.1)
