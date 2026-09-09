"""Verifies the MAVLink encoding constants against their documented values, independent of
any FC or serial link. See README.md for the citations these numbers come from."""

from mavlink_bridge.bridge import (
    _POSITION_TARGET_TYPEMASK_VELOCITY_AND_YAW_RATE,
    _clamp,
)


def test_velocity_and_yaw_rate_typemask_matches_ardupilot_wiki_value():
    # ArduPilot "Copter Commands in Guided Mode" — SET_POSITION_TARGET_LOCAL_NED,
    # "Velocity + yaw rate" row: 0b0000101111000111 == 1479 == 0x5C7.
    assert _POSITION_TARGET_TYPEMASK_VELOCITY_AND_YAW_RATE == 1479
    assert _POSITION_TARGET_TYPEMASK_VELOCITY_AND_YAW_RATE == 0x5C7


def test_typemask_ignores_position_acceleration_and_yaw_but_not_velocity_or_yaw_rate():
    mask = _POSITION_TARGET_TYPEMASK_VELOCITY_AND_YAW_RATE
    POS_X_IGNORE, POS_Y_IGNORE, POS_Z_IGNORE = 1, 2, 4
    VX_IGNORE, VY_IGNORE, VZ_IGNORE = 8, 16, 32
    AX_IGNORE, AY_IGNORE, AZ_IGNORE = 64, 128, 256
    YAW_IGNORE = 1024
    YAW_RATE_IGNORE = 2048

    for bit in (POS_X_IGNORE, POS_Y_IGNORE, POS_Z_IGNORE, AX_IGNORE, AY_IGNORE, AZ_IGNORE, YAW_IGNORE):
        assert mask & bit, f"expected bit {bit} (ignore) to be set"
    for bit in (VX_IGNORE, VY_IGNORE, VZ_IGNORE, YAW_RATE_IGNORE):
        assert not (mask & bit), f"expected bit {bit} (use velocity/yaw_rate) to be clear"


def test_clamp_limits_magnitude_but_preserves_sign():
    assert _clamp(1.5, 0.6) == 0.6
    assert _clamp(-1.5, 0.6) == -0.6
    assert _clamp(0.3, 0.6) == 0.3
    assert _clamp(-0.3, 0.6) == -0.3


def test_clamp_zero_limit_forces_zero():
    assert _clamp(5.0, 0.0) == 0.0
    assert _clamp(-5.0, 0.0) == 0.0
