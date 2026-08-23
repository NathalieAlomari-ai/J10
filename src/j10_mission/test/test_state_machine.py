"""Tests for the mission state machine.

Two things are being pinned down here. First, that the path into autonomy is narrow and
every guard on it actually holds. Second -- and easier to get wrong -- that the path *out*
is always open: a transition table that can trap a flying vehicle is a worse bug than any
it prevents.
"""

import math

import pytest

from j10_mission.state_machine import (
    ARMED,
    DISARM,
    HOLD,
    IDLE,
    LAND,
    PREFLIGHT,
    TAKEOFF,
    VLA_ACTIVE,
    Guards,
    MissionStateMachine,
    VehicleFacts,
)


def healthy(**overrides) -> VehicleFacts:
    """A vehicle that passes every guard, so each test can break exactly one thing."""
    facts = dict(
        connected=True, armed=True, guided=True, ekf_healthy=True,
        battery_percentage=0.9, altitude_m=1.5, age_sec=0.05,
        estop_latched=False, video_ok=True,
    )
    facts.update(overrides)
    return VehicleFacts(**facts)


def machine_at(state: str, instruction: str = 'fly the pattern', **guard_kwargs):
    """A machine walked to *state* through the legal sequence, with guards satisfied."""
    m = MissionStateMachine(Guards(**guard_kwargs) if guard_kwargs else None)
    if instruction:
        m.set_instruction(instruction)
    route = {
        IDLE: [],
        PREFLIGHT: [PREFLIGHT],
        ARMED: [PREFLIGHT, ARMED],
        TAKEOFF: [PREFLIGHT, ARMED, TAKEOFF],
        VLA_ACTIVE: [PREFLIGHT, ARMED, TAKEOFF, VLA_ACTIVE],
        HOLD: [PREFLIGHT, ARMED, TAKEOFF, HOLD],
        LAND: [PREFLIGHT, ARMED, TAKEOFF, LAND],
        DISARM: [PREFLIGHT, ARMED, TAKEOFF, LAND, DISARM],
    }[state]
    for step in route:
        facts = healthy(altitude_m=0.0) if step == DISARM else healthy()
        result = m.request(step, facts)
        assert result.accepted, f'setup could not reach {state}: {result.message}'
    assert m.state == state
    return m


class TestHappyPath:
    def test_the_documented_sequence_runs_end_to_end(self):
        m = MissionStateMachine()
        m.set_instruction('fly toward the doorway')
        for step in (PREFLIGHT, ARMED, TAKEOFF, VLA_ACTIVE, HOLD, LAND):
            assert m.request(step, healthy()).accepted, step
        assert m.request(DISARM, healthy(altitude_m=0.0)).accepted
        assert m.state == DISARM
        assert m.request(IDLE, healthy(altitude_m=0.0)).accepted

    def test_history_records_every_transition(self):
        m = machine_at(ARMED)
        assert m.history == [(IDLE, PREFLIGHT), (PREFLIGHT, ARMED)]

    def test_history_is_a_copy_not_the_live_list(self):
        m = machine_at(PREFLIGHT)
        m.history.append(('bogus', 'entry'))
        assert len(m.history) == 1


class TestAutonomyGate:
    def test_autonomy_is_disabled_in_every_state_but_one(self):
        # The single most consequential assertion in this file: it enumerates exactly when a
        # model may drive. Widening AUTONOMOUS_STATES should break this test loudly.
        for state in (IDLE, PREFLIGHT, ARMED, TAKEOFF, HOLD, LAND, DISARM):
            assert machine_at(state).autonomy_enabled() is False, state
        assert machine_at(VLA_ACTIVE).autonomy_enabled() is True

    def test_a_fresh_machine_starts_with_autonomy_off(self):
        assert MissionStateMachine().autonomy_enabled() is False

    def test_leaving_vla_active_revokes_autonomy_immediately(self):
        m = machine_at(VLA_ACTIVE)
        assert m.autonomy_enabled() is True
        m.request(HOLD, healthy())
        assert m.autonomy_enabled() is False


class TestIllegalTransitions:
    @pytest.mark.parametrize('start', [IDLE, PREFLIGHT, ARMED])
    def test_autonomy_cannot_be_reached_without_taking_off(self, start):
        # No shortcut into VLA_ACTIVE. "The model is flying" must never be one call away
        # from a cold start, however healthy the vehicle looks.
        m = machine_at(start)
        result = m.request(VLA_ACTIVE, healthy())
        assert not result.accepted
        assert 'illegal transition' in result.message
        assert m.state == start

    def test_idle_cannot_jump_straight_to_armed(self):
        assert not MissionStateMachine().request(ARMED, healthy()).accepted

    def test_unknown_state_is_rejected(self):
        result = MissionStateMachine().request('LAUNCH_MISSILES', healthy())
        assert not result.accepted
        assert 'unknown state' in result.message

    def test_rejected_transition_leaves_state_untouched(self):
        m = machine_at(ARMED)
        m.request(VLA_ACTIVE, healthy())
        assert m.state == ARMED
        assert m.history[-1] == (PREFLIGHT, ARMED)

    def test_requesting_the_current_state_succeeds_without_recording_history(self):
        # Idempotent: a retried or duplicated request is not a failure.
        m = machine_at(ARMED)
        before = m.history
        result = m.request(ARMED, healthy())
        assert result.accepted
        assert 'already in' in result.message
        assert m.history == before

    def test_the_illegal_transition_message_lists_what_is_allowed(self):
        result = machine_at(IDLE).request(ARMED, healthy())
        assert PREFLIGHT in result.message


class TestEscapeHatchesAlwaysOpen:
    @pytest.mark.parametrize('start', [IDLE, PREFLIGHT, ARMED, TAKEOFF, VLA_ACTIVE, HOLD])
    def test_land_is_reachable_from_anywhere(self, start):
        assert machine_at(start).request(LAND, healthy()).accepted

    @pytest.mark.parametrize('start', [TAKEOFF, VLA_ACTIVE, LAND])
    def test_hold_is_reachable_from_anywhere_airborne(self, start):
        assert machine_at(start).request(HOLD, healthy()).accepted

    @pytest.mark.parametrize('broken', [
        {'connected': False},
        {'ekf_healthy': False},
        {'battery_percentage': 0.01},
        {'age_sec': math.inf},
        {'estop_latched': True},
    ])
    def test_land_is_never_blocked_by_a_guard(self, broken):
        # The inversion worth testing explicitly: bad telemetry is a reason to get down, not
        # a reason to refuse to. A guard on LAND would strand the vehicle exactly when
        # things are already going wrong.
        assert machine_at(VLA_ACTIVE).request(LAND, healthy(**broken)).accepted

    def test_hold_is_never_blocked_by_a_guard(self):
        assert machine_at(VLA_ACTIVE).request(HOLD, healthy(connected=False)).accepted


class TestGuards:
    def test_preflight_needs_a_connected_vehicle(self):
        result = MissionStateMachine().request(PREFLIGHT, healthy(connected=False))
        assert not result.accepted and result.guard_failed
        assert 'not connected' in result.message

    def test_preflight_refuses_a_flat_battery(self):
        result = MissionStateMachine().request(PREFLIGHT, healthy(battery_percentage=0.1))
        assert not result.accepted and result.guard_failed
        assert 'battery' in result.message

    def test_stale_vehicle_state_blocks_progress(self):
        result = MissionStateMachine().request(PREFLIGHT, healthy(age_sec=5.0))
        assert not result.accepted
        assert 'stale' in result.message

    def test_never_received_state_reads_clearly_rather_than_as_inf(self):
        result = MissionStateMachine().request(PREFLIGHT, healthy(age_sec=math.inf))
        assert 'never received' in result.message

    def test_arming_requires_a_healthy_ekf(self):
        result = machine_at(PREFLIGHT).request(ARMED, healthy(ekf_healthy=False))
        assert not result.accepted
        assert 'EKF' in result.message

    def test_the_ekf_arming_guard_can_be_relaxed_for_sitl(self):
        m = machine_at(PREFLIGHT, require_ekf_for_arm=False)
        assert m.request(ARMED, healthy(ekf_healthy=False)).accepted

    def test_takeoff_requires_an_armed_vehicle(self):
        result = machine_at(ARMED).request(TAKEOFF, healthy(armed=False))
        assert not result.accepted
        assert 'not armed' in result.message

    def test_guard_failure_is_distinguished_from_an_illegal_transition(self):
        # Different causes deserve different reactions: an illegal transition is a caller
        # bug, a failed guard is the vehicle saying "not yet".
        guard = machine_at(PREFLIGHT).request(ARMED, healthy(ekf_healthy=False))
        illegal = machine_at(IDLE).request(ARMED, healthy())
        assert guard.guard_failed is True
        assert illegal.guard_failed is False


class TestAutonomyGuards:
    def test_a_latched_estop_blocks_entry_to_autonomy(self):
        result = machine_at(TAKEOFF).request(VLA_ACTIVE, healthy(estop_latched=True))
        assert not result.accepted
        assert 'E-stop' in result.message

    def test_resuming_autonomy_from_hold_also_respects_the_estop(self):
        # The resume path is the one that gets forgotten; an E-stop that only blocks the
        # first entry is not an E-stop.
        result = machine_at(HOLD).request(VLA_ACTIVE, healthy(estop_latched=True))
        assert not result.accepted
        assert 'E-stop' in result.message

    def test_autonomy_requires_a_guided_mode(self):
        result = machine_at(TAKEOFF).request(VLA_ACTIVE, healthy(guided=False))
        assert not result.accepted
        assert 'guided' in result.message

    def test_autonomy_requires_a_healthy_ekf(self):
        result = machine_at(TAKEOFF).request(VLA_ACTIVE, healthy(ekf_healthy=False))
        assert not result.accepted

    def test_autonomy_is_refused_on_the_ground(self):
        # ArduPilot ignores velocity setpoints until the vehicle is flying, so entering
        # VLA_ACTIVE at zero altitude would report autonomy while doing nothing at all.
        result = machine_at(TAKEOFF).request(VLA_ACTIVE, healthy(altitude_m=0.0))
        assert not result.accepted
        assert 'altitude' in result.message

    def test_autonomy_requires_an_instruction(self):
        m = machine_at(TAKEOFF, instruction='')
        result = m.request(VLA_ACTIVE, healthy())
        assert not result.accepted
        assert 'instruction' in result.message

    def test_clearing_the_instruction_blocks_re_entry_to_autonomy(self):
        m = machine_at(HOLD)
        m.set_instruction('')
        assert not m.request(VLA_ACTIVE, healthy()).accepted

    def test_video_guard_is_off_until_phase_3(self):
        # j10_video does not exist yet; requiring it by default would make VLA_ACTIVE
        # unreachable in the Phase 4 configuration this is being built for.
        assert machine_at(TAKEOFF).request(VLA_ACTIVE, healthy(video_ok=False)).accepted

    def test_video_guard_blocks_autonomy_once_enabled(self):
        m = machine_at(TAKEOFF, require_video_for_autonomy=True)
        result = m.request(VLA_ACTIVE, healthy(video_ok=False))
        assert not result.accepted
        assert 'blind' in result.message


class TestDisarmGuard:
    def test_disarm_is_refused_in_flight(self):
        m = machine_at(LAND)
        result = m.request(DISARM, healthy(altitude_m=3.0))
        assert not result.accepted
        assert 'land first' in result.message

    def test_disarm_is_allowed_on_the_ground(self):
        assert machine_at(LAND).request(DISARM, healthy(altitude_m=0.0)).accepted

    def test_forced_disarm_in_flight_is_permitted_as_a_deliberate_act(self):
        # Cutting motors in flight is a real emergency action. It must be reachable, but
        # only when asked for explicitly -- never by walking the normal sequence.
        assert machine_at(LAND).request(DISARM, healthy(altitude_m=3.0), force=True).accepted


class TestForce:
    def test_force_skips_guards(self):
        m = machine_at(PREFLIGHT)
        result = m.request(ARMED, healthy(ekf_healthy=False, connected=False), force=True)
        assert result.accepted
        assert 'forced' in result.message

    def test_force_does_not_unlock_illegal_transitions(self):
        # The bound that keeps force from being a way into autonomy: it relaxes the
        # vehicle's readiness checks, never the sequence itself.
        result = MissionStateMachine().request(VLA_ACTIVE, healthy(), force=True)
        assert not result.accepted
        assert 'illegal transition' in result.message

    def test_force_cannot_reach_autonomy_from_a_cold_start(self):
        m = MissionStateMachine()
        m.set_instruction('go')
        for target in (ARMED, TAKEOFF, VLA_ACTIVE):
            assert not m.request(target, healthy(), force=True).accepted
        assert m.autonomy_enabled() is False


class TestInstruction:
    def test_instruction_round_trips_and_is_stripped(self):
        m = MissionStateMachine()
        ok, _ = m.set_instruction('  fly toward the doorway  ')
        assert ok
        assert m.instruction == 'fly toward the doorway'

    def test_instruction_can_change_mid_flight(self):
        # Changing what is asked of the vehicle is not the same as changing whether it may
        # act. Blocking this would mean the only way to redirect is to drop out of autonomy.
        m = machine_at(VLA_ACTIVE)
        assert m.set_instruction('turn left at the corridor')[0]
        assert m.state == VLA_ACTIVE
        assert m.autonomy_enabled() is True

    def test_overlong_instruction_is_rejected(self):
        ok, message = MissionStateMachine().set_instruction('x' * 513)
        assert not ok and '512' in message

    def test_non_string_instruction_is_rejected(self):
        assert not MissionStateMachine().set_instruction(None)[0]

    def test_a_fresh_machine_has_no_instruction(self):
        assert MissionStateMachine().instruction == ''


class TestConstruction:
    def test_unknown_initial_state_is_rejected(self):
        with pytest.raises(ValueError, match='unknown initial state'):
            MissionStateMachine(initial_state='FLYING')

    def test_default_initial_state_is_idle(self):
        assert MissionStateMachine().state == IDLE

    def test_default_guards_are_conservative(self):
        g = Guards()
        assert g.require_ekf_for_arm is True
        assert g.require_guided_for_autonomy is True
        assert g.min_battery_percentage > 0.0
        assert g.min_autonomy_altitude_m > 0.0
