"""Tests for the backend contract's shared plumbing.

Everything here is about one question: what happens when a backend misbehaves? In Phase 5
the backend is a neural network, and the only guarantee worth having is that no output it
can produce -- NaN, negative, out of range, wrong type -- reaches a setpoint as anything
other than a bounded number or a hover.
"""

import math

import pytest

from j10_vla.backend import (
    ACTION_HOLD,
    ACTION_LAND,
    ACTION_MOVE,
    HOLD,
    Decision,
    Observation,
    VehicleSnapshot,
)

MAX_DURATION = 2.0


def sanitize(**kwargs):
    return Decision(**kwargs).sanitized(MAX_DURATION)


class TestSanitizeVelocity:
    def test_finite_velocities_pass_through_untouched(self):
        out = sanitize(vx=0.3, vy=-0.2, vz=0.1, yaw_rate=-0.4)
        assert (out.vx, out.vy, out.vz, out.yaw_rate) == (0.3, -0.2, 0.1, -0.4)

    @pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
    def test_non_finite_velocity_becomes_zero(self, bad):
        out = sanitize(vx=bad, vy=bad, vz=bad, yaw_rate=bad)
        assert (out.vx, out.vy, out.vz, out.yaw_rate) == (0.0, 0.0, 0.0, 0.0)

    def test_one_bad_axis_does_not_zero_the_others(self):
        # A NaN in vy must not be treated as "the whole decision is garbage" -- the other
        # axes are still meaningful, and zeroing them would turn a partial fault into a
        # larger unrequested change in motion.
        out = sanitize(vx=0.3, vy=float('nan'), vz=0.1)
        assert out.vx == 0.3
        assert out.vy == 0.0
        assert out.vz == 0.1

    def test_non_numeric_velocity_becomes_zero_rather_than_raising(self):
        out = sanitize(vx='forward')  # type: ignore[arg-type]
        assert out.vx == 0.0

    def test_sanitize_does_not_clamp_magnitude(self):
        # Deliberate: bounding speed is j10_safety's job, and it is the only node allowed to
        # do it. Silently clamping here would hide an out-of-envelope request from the one
        # component whose entire purpose is to notice and report it.
        out = sanitize(vx=99.0)
        assert out.vx == 99.0


class TestSanitizeDuration:
    def test_duration_within_range_is_kept(self):
        assert sanitize(duration_sec=0.5).duration_sec == 0.5

    @pytest.mark.parametrize('bad', [0.0, -1.0, float('nan'), float('-inf')])
    def test_non_positive_or_broken_duration_is_floored_not_zeroed(self, bad):
        # Zero would expire instantly and read downstream as an ordinary hover, hiding the
        # arithmetic fault. A tiny positive value expires just as fast but keeps the
        # decision a real, observable event.
        out = sanitize(duration_sec=bad)
        assert out.duration_sec > 0.0
        assert out.duration_sec <= MAX_DURATION

    def test_duration_is_capped_at_the_configured_ceiling(self):
        # The cap is what stops a backend granting itself long unsupervised authority.
        assert sanitize(duration_sec=3600.0).duration_sec == MAX_DURATION

    def test_infinite_duration_expires_immediately_rather_than_lasting_the_maximum(self):
        # +inf reads as "valid forever", and the tempting fix is to cap it at the ceiling
        # like any other too-large number. That is backwards: a finite 3600.0 is a backend
        # asking for too much, while inf is a backend that is *broken*, and a broken backend
        # should get the least authority available, not the most. Flooring it expires the
        # decision on arrival, so the controller hovers -- the same outcome as NaN.
        out = sanitize(duration_sec=float('inf'))
        assert out.duration_sec < MAX_DURATION
        assert out.duration_sec > 0.0

    def test_ceiling_of_zero_still_yields_a_positive_duration(self):
        # A misconfigured ceiling must not produce an invalid decision.
        assert Decision(duration_sec=1.0).sanitized(0.0).duration_sec > 0.0


class TestSanitizeConfidence:
    @pytest.mark.parametrize('value,expected', [
        (0.0, 0.0), (0.5, 0.5), (1.0, 1.0),
        (-0.5, 0.0), (1.5, 1.0),
    ])
    def test_confidence_is_clamped_to_unit_range(self, value, expected):
        assert sanitize(confidence=value).confidence == expected

    def test_nan_confidence_becomes_zero_not_one(self):
        # The safe reading of "unknown confidence" is no confidence. Defaulting high would
        # let a broken model inherit full trust from the safety filter's scaling.
        assert sanitize(confidence=float('nan')).confidence == 0.0


class TestSanitizeAction:
    @pytest.mark.parametrize('action', [ACTION_HOLD, ACTION_MOVE, ACTION_LAND])
    def test_known_actions_are_preserved(self, action):
        assert sanitize(action_type=action).action_type == action

    @pytest.mark.parametrize('bad', [99, -1, 255])
    def test_unknown_action_falls_back_to_hold(self, bad):
        assert sanitize(action_type=bad).action_type == ACTION_HOLD

    def test_non_string_rationale_is_replaced(self):
        assert sanitize(rationale=object()).rationale == ''  # type: ignore[arg-type]


class TestObservationImageAge:
    def test_age_is_now_minus_capture(self):
        obs = Observation(image=object(), image_stamp_sec=10.0, now_sec=10.25)
        assert obs.image_age_sec == pytest.approx(0.25)

    def test_missing_image_is_infinitely_old(self):
        assert Observation(now_sec=10.0).image_age_sec == math.inf

    def test_image_without_a_stamp_is_infinitely_old(self):
        # An unstamped frame cannot be shown to be fresh, so it must not count as fresh.
        assert Observation(image=object(), now_sec=10.0).image_age_sec == math.inf

    def test_future_capture_time_clamps_to_zero_rather_than_going_negative(self):
        # Capture time is reconstructed from an RTP timestamp on another machine, so mild
        # clock skew is expected. A negative age would compare as "fresh" against every
        # threshold, which is right by luck; clamping makes it right by construction.
        obs = Observation(image=object(), image_stamp_sec=10.5, now_sec=10.0)
        assert obs.image_age_sec == 0.0


class TestDefaults:
    def test_hold_constant_is_a_zero_velocity_hold(self):
        assert HOLD.action_type == ACTION_HOLD
        assert (HOLD.vx, HOLD.vy, HOLD.vz, HOLD.yaw_rate) == (0.0, 0.0, 0.0, 0.0)
        assert HOLD.duration_sec > 0.0

    def test_default_decision_is_a_hold(self):
        # The dataclass defaults matter: a backend that constructs Decision() and forgets to
        # fill it in must produce stillness, not motion.
        default = Decision()
        assert default.action_type == ACTION_HOLD
        assert (default.vx, default.vy, default.vz, default.yaw_rate) == (0.0, 0.0, 0.0, 0.0)

    def test_default_confidence_is_zero(self):
        assert Decision().confidence == 0.0

    def test_vehicle_snapshot_defaults_to_stale_and_disarmed(self):
        snap = VehicleSnapshot()
        assert snap.armed is False
        assert snap.age_sec == math.inf
