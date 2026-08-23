"""Tests for the Phase 4 scripted policy.

The pattern's value is that it is *predictable*: a Phase 4 run that flies the wrong shape
must implicate the plumbing, never the policy. These tests are what earn that claim, so
they pin the timing behaviour precisely rather than sampling it loosely.
"""

import pytest

from j10_vla.backend import ACTION_HOLD, ACTION_MOVE, ACTION_TURN, Observation
from j10_vla.scripted_backend import (
    DEFAULT_PATTERN,
    ScriptedBackend,
    Step,
    parse_steps,
)

INSTRUCTION = 'fly the test pattern'


def observe(t, instruction=INSTRUCTION):
    return Observation(instruction=instruction, now_sec=t)


def simple_pattern():
    """forward 2 s, then yaw 1 s -- a 3 s cycle with two distinguishable legs."""
    return [
        Step(action_type=ACTION_MOVE, vx=0.3, hold_sec=2.0, label='fwd'),
        Step(action_type=ACTION_TURN, yaw_rate=0.5, hold_sec=1.0, label='turn'),
    ]


class TestStepValidation:
    @pytest.mark.parametrize('bad', [0.0, -1.0])
    def test_non_positive_hold_is_rejected(self, bad):
        # A zero-length step is unreachable by the time-based lookup, so it would silently
        # vanish from the flown shape. Rejecting at construction makes that a startup error.
        with pytest.raises(ValueError, match='hold_sec'):
            Step(hold_sec=bad)

    def test_unknown_action_is_rejected(self):
        with pytest.raises(ValueError, match='action_type'):
            Step(action_type=42, hold_sec=1.0)


class TestParseSteps:
    def test_parses_a_well_formed_line(self):
        steps = parse_steps(['MOVE 0.3 -0.1 0.2 0.4 2.5'])
        assert len(steps) == 1
        step = steps[0]
        assert step.action_type == ACTION_MOVE
        assert (step.vx, step.vy, step.vz, step.yaw_rate) == (0.3, -0.1, 0.2, 0.4)
        assert step.hold_sec == 2.5

    def test_action_name_is_case_insensitive(self):
        assert parse_steps(['move 0.1 0 0 0 1'])[0].action_type == ACTION_MOVE

    def test_comments_and_blank_lines_are_skipped(self):
        steps = parse_steps([
            '# a leading comment',
            '',
            '   ',
            'MOVE 0.3 0 0 0 2   # trailing comment',
        ])
        assert len(steps) == 1

    def test_the_shipped_default_pattern_parses(self):
        # Guards against the documented default and the parser drifting apart.
        steps = parse_steps(DEFAULT_PATTERN)
        assert len(steps) == len(DEFAULT_PATTERN)

    @pytest.mark.parametrize('bad,match', [
        ('MOVE 0.3 0 0 0', '6 fields'),
        ('MOVE 0.3 0 0 0 1 2', '6 fields'),
        ('FLY 0.3 0 0 0 1', 'unknown action'),
        ('MOVE fast 0 0 0 1', 'non-numeric'),
    ])
    def test_malformed_lines_raise_with_a_useful_message(self, bad, match):
        with pytest.raises(ValueError, match=match):
            parse_steps([bad])

    def test_non_string_entry_is_rejected(self):
        with pytest.raises(ValueError, match='expected a string'):
            parse_steps([42])

    def test_empty_pattern_is_rejected(self):
        # Silently accepting an empty pattern would produce a backend that holds forever and
        # looks, from the outside, exactly like a working one that never got an instruction.
        with pytest.raises(ValueError, match='empty'):
            parse_steps([])

    def test_pattern_of_only_comments_is_rejected(self):
        with pytest.raises(ValueError, match='empty'):
            parse_steps(['# nothing here', ''])


class TestPatternTiming:
    def test_cycle_is_the_sum_of_step_holds(self):
        assert ScriptedBackend(simple_pattern()).cycle_sec == pytest.approx(3.0)

    def test_first_call_anchors_the_clock_wherever_it_lands(self):
        # Node startup, weight loading, and the first frame are seconds apart. Anchoring at
        # construction instead of first inference would skip whatever legs elapsed in
        # between -- the pattern would start mid-shape.
        backend = ScriptedBackend(simple_pattern())
        first = backend.infer(observe(1000.0))
        assert first.action_type == ACTION_MOVE
        assert backend.infer(observe(1002.5)).action_type == ACTION_TURN

    @pytest.mark.parametrize('t,expected', [
        (0.0, ACTION_MOVE),
        (1.9, ACTION_MOVE),
        (2.0, ACTION_TURN),   # boundary belongs to the next step
        (2.9, ACTION_TURN),
        (3.0, ACTION_MOVE),   # wrapped
        (5.0, ACTION_TURN),
    ])
    def test_step_boundaries_are_exact(self, t, expected):
        backend = ScriptedBackend(simple_pattern())
        backend.infer(observe(0.0))
        assert backend.infer(observe(t)).action_type == expected

    def test_velocities_come_from_the_active_step(self):
        backend = ScriptedBackend(simple_pattern())
        backend.infer(observe(0.0))
        moving = backend.infer(observe(0.5))
        assert moving.vx == 0.3
        assert moving.yaw_rate == 0.0
        turning = backend.infer(observe(2.5))
        assert turning.vx == 0.0
        assert turning.yaw_rate == 0.5

    def test_looping_reports_completed_cycles(self):
        backend = ScriptedBackend(simple_pattern(), loop=True)
        backend.infer(observe(0.0))
        backend.infer(observe(7.0))
        assert backend.completed_cycles == 2

    def test_non_looping_pattern_holds_after_the_last_step(self):
        backend = ScriptedBackend(simple_pattern(), loop=False)
        backend.infer(observe(0.0))
        assert backend.infer(observe(2.5)).action_type == ACTION_TURN
        done = backend.infer(observe(3.1))
        assert done.action_type == ACTION_HOLD
        assert (done.vx, done.yaw_rate) == (0.0, 0.0)
        assert 'complete' in done.rationale

    def test_a_finished_pattern_stays_finished(self):
        backend = ScriptedBackend(simple_pattern(), loop=False)
        backend.infer(observe(0.0))
        backend.infer(observe(10.0))
        assert backend.infer(observe(100.0)).action_type == ACTION_HOLD

    def test_time_going_backwards_does_not_produce_a_negative_elapsed(self):
        # Wall clocks step backwards (NTP correction, a paused simulator resuming). The
        # lookup must stay inside the pattern rather than indexing off the front of it.
        backend = ScriptedBackend(simple_pattern())
        backend.infer(observe(1000.0))
        recovered = backend.infer(observe(999.0))
        assert recovered.action_type in (ACTION_MOVE, ACTION_TURN)


class TestInstructionGating:
    @pytest.mark.parametrize('instruction', ['', '   ', '\n\t'])
    def test_no_instruction_means_hold(self, instruction):
        # Unrequested autonomy is the failure this guards. A backend that flies whenever it
        # happens to be running is a backend nobody asked to fly.
        backend = ScriptedBackend(simple_pattern())
        decision = backend.infer(observe(0.0, instruction=instruction))
        assert decision.action_type == ACTION_HOLD
        assert (decision.vx, decision.yaw_rate) == (0.0, 0.0)
        assert 'no instruction' in decision.rationale

    def test_the_clock_does_not_start_until_an_instruction_arrives(self):
        # Otherwise the pattern would burn through its opening legs while idle and begin
        # mid-shape the moment an instruction landed.
        backend = ScriptedBackend(simple_pattern())
        for t in (0.0, 1.0, 2.0, 3.0):
            assert backend.infer(observe(t, instruction='')).action_type == ACTION_HOLD
        assert backend.infer(observe(4.0)).action_type == ACTION_MOVE

    def test_gating_can_be_disabled_for_bench_runs(self):
        backend = ScriptedBackend(simple_pattern(), require_instruction=False)
        assert backend.infer(observe(0.0, instruction='')).action_type == ACTION_MOVE


class TestDecisionShape:
    def test_duration_is_a_short_validity_window_not_the_step_length(self):
        # The distinction that makes silence fail safe: a 2 s step must not hand out 2 s of
        # unsupervised authority, or a backend that dies mid-leg keeps the vehicle moving.
        backend = ScriptedBackend(simple_pattern())
        backend.infer(observe(0.0))
        decision = backend.infer(observe(0.1))
        assert decision.duration_sec < 2.0

    def test_confidence_is_reported_and_configurable(self):
        backend = ScriptedBackend(simple_pattern(), confidence=0.42)
        backend.infer(observe(0.0))
        assert backend.infer(observe(0.5)).confidence == 0.42

    def test_rationale_names_the_step_and_the_time(self):
        backend = ScriptedBackend(simple_pattern())
        backend.infer(observe(0.0))
        assert 'fwd' in backend.infer(observe(0.5)).rationale

    def test_every_decision_survives_sanitizing_unchanged(self):
        # The scripted policy must itself be well-formed; if sanitizing has to correct it,
        # the pattern flown would differ from the pattern written.
        backend = ScriptedBackend(parse_steps(DEFAULT_PATTERN))
        backend.infer(observe(0.0))
        for t in [x * 0.25 for x in range(80)]:
            decision = backend.infer(observe(t))
            assert decision.sanitized(2.0) == decision


class TestBackendInfo:
    def test_reports_itself_as_not_a_model(self):
        # So a scripted run is never mistaken for a model run when reading logs back.
        info = ScriptedBackend(simple_pattern()).info()
        assert info.is_model is False
        assert info.name == 'scripted'

    def test_details_describe_the_pattern(self):
        info = ScriptedBackend(simple_pattern(), loop=False).info()
        assert info.details['steps'] == 2
        assert info.details['cycle_sec'] == pytest.approx(3.0)
        assert info.details['loop'] is False


class TestSimulatedLatency:
    def test_zero_latency_does_not_sleep(self):
        import time
        backend = ScriptedBackend(simple_pattern(), simulated_latency_sec=0.0)
        started = time.monotonic()
        backend.infer(observe(0.0))
        assert time.monotonic() - started < 0.05

    def test_configured_latency_is_actually_spent(self):
        # The cascade exists to absorb a slow producer, so the stub has to be able to *be*
        # a slow producer -- otherwise Phase 4 passes on timings Phase 5 will never see.
        import time
        backend = ScriptedBackend(simple_pattern(), simulated_latency_sec=0.05)
        started = time.monotonic()
        backend.infer(observe(0.0))
        assert time.monotonic() - started >= 0.045

    def test_negative_latency_is_treated_as_zero(self):
        backend = ScriptedBackend(simple_pattern(), simulated_latency_sec=-1.0)
        backend.infer(observe(0.0))  # must not raise
