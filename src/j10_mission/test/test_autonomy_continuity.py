"""Tests for the continuous autonomy claim.

Entering VLA_ACTIVE is checked once, by the entry guards. Staying there is checked on every
tick by ``autonomy_still_valid``, and the two are deliberately not the same predicate. These
tests pin down both the revocation conditions and -- just as important -- the conditions
that must *not* revoke, since a manager that yanks autonomy on every transient would be
unusable in exactly the flights it exists to supervise.
"""

import math

import pytest

from j10_mission.state_machine import (
    Guards,
    MissionStateMachine,
    VehicleFacts,
)

from test_state_machine import healthy, machine_at  # noqa: I100 -- shared fixtures


def flying(**overrides):
    return machine_at('VLA_ACTIVE'), healthy(**overrides)


class TestRevocation:
    @pytest.mark.parametrize('broken,expected', [
        ({'estop_latched': True}, 'E-stop'),
        ({'connected': False}, 'disconnected'),
        ({'armed': False}, 'disarmed'),
        ({'guided': False}, 'guided'),
        ({'ekf_healthy': False}, 'EKF'),
        ({'age_sec': 5.0}, 'stale'),
    ])
    def test_each_lost_condition_revokes_autonomy(self, broken, expected):
        machine, facts = flying(**broken)
        reason = machine.autonomy_still_valid(facts)
        assert reason is not None
        assert expected in reason

    def test_healthy_flight_keeps_autonomy(self):
        machine, facts = flying()
        assert machine.autonomy_still_valid(facts) is None

    def test_clearing_the_instruction_revokes_autonomy(self):
        machine, facts = flying()
        machine.set_instruction('')
        assert machine.autonomy_still_valid(facts) == 'instruction cleared'

    def test_never_received_state_reads_clearly(self):
        machine, facts = flying(age_sec=math.inf)
        assert 'never received' in machine.autonomy_still_valid(facts)

    def test_video_loss_does_not_revoke_until_the_guard_is_enabled(self):
        # j10_video does not exist yet. Revoking on it by default would make Phase 4
        # autonomy impossible to hold.
        machine = machine_at('VLA_ACTIVE')
        assert machine.autonomy_still_valid(healthy(video_ok=False)) is None

    def test_video_loss_revokes_once_the_guard_is_enabled(self):
        machine = machine_at('VLA_ACTIVE', require_video_for_autonomy=True)
        assert 'video' in machine.autonomy_still_valid(healthy(video_ok=False))


class TestNonRevocation:
    def test_descending_does_not_revoke_autonomy(self):
        # The asymmetry that matters: refusing to *hand over* control on the ground is
        # right, but revoking mid-flight because the vehicle went low would fight the very
        # behaviour autonomy was granted to perform -- including a commanded descent.
        machine, facts = flying(altitude_m=0.0)
        assert machine.autonomy_still_valid(facts) is None

    def test_a_flat_battery_does_not_revoke_autonomy(self):
        # Battery is a preflight gate, not a continuous one. Revoking here would hand
        # control back at the worst moment; escalating on battery is j10_safety's job, and
        # it lands the vehicle rather than merely stopping the model.
        machine, facts = flying(battery_percentage=0.05)
        assert machine.autonomy_still_valid(facts) is None

    def test_state_age_inside_the_limit_is_tolerated(self):
        machine = machine_at('VLA_ACTIVE', max_state_age_sec=1.0)
        assert machine.autonomy_still_valid(healthy(age_sec=0.9)) is None

    def test_the_guided_check_can_be_relaxed(self):
        machine = machine_at('VLA_ACTIVE', require_guided_for_autonomy=False)
        assert machine.autonomy_still_valid(healthy(guided=False)) is None


class TestDemotionIsAlwaysPossible:
    @pytest.mark.parametrize('broken', [
        {'estop_latched': True},
        {'connected': False},
        {'ekf_healthy': False},
        {'age_sec': math.inf},
        {'armed': False},
    ])
    def test_hold_is_reachable_under_every_revocation_condition(self, broken):
        # The node demotes to HOLD whenever autonomy is revoked. If any revocation condition
        # could also block the demotion, the manager would keep publishing
        # autonomy_enabled=true while claiming it had stopped -- the worst possible outcome.
        machine = machine_at('VLA_ACTIVE')
        facts = healthy(**broken)
        assert machine.autonomy_still_valid(facts) is not None
        result = machine.request('HOLD', facts)
        assert result.accepted, result.message
        assert machine.autonomy_enabled() is False


class TestGuardsAccessor:
    def test_guards_are_readable_without_touching_privates(self):
        machine = MissionStateMachine(Guards(max_state_age_sec=2.5))
        assert machine.guards.max_state_age_sec == 2.5
