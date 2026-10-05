"""Tests for the latency aggregation.

A latency monitor that is subtly wrong is worse than none: it turns "we don't know" into a
number people then trust. So these tests care less about the happy path than about the ways
a summary can look healthy while being meaningless -- rejected samples hidden, an empty
window reported as zero, a p99 computed from eleven samples, a tail flattered by
interpolation.
"""

import math

import pytest

from j10_telemetry.percentiles import (
    RollingWindow,
    StageAggregator,
    Summary,
    check_budget,
    nearest_rank,
)


class TestNearestRank:
    def test_matches_hand_computed_ranks(self):
        values = list(range(1, 101))  # 1..100
        assert nearest_rank(values, 50.0) == 50
        assert nearest_rank(values, 95.0) == 95
        assert nearest_rank(values, 99.0) == 99

    def test_endpoints_give_min_and_max(self):
        values = [10.0, 20.0, 30.0]
        assert nearest_rank(values, 0.0) == 10.0
        assert nearest_rank(values, 100.0) == 30.0

    def test_returns_a_value_that_actually_occurred(self):
        # The reason for nearest-rank over interpolation. With interpolation p95 of this
        # list would land between 4 and 100 -- a figure no sample ever produced, and always
        # optimistic at the tail, which is the one place optimism is unwanted.
        values = [1.0, 2.0, 3.0, 4.0, 100.0]
        assert nearest_rank(values, 95.0) in values
        assert nearest_rank(values, 95.0) == 100.0

    def test_single_sample_is_every_percentile(self):
        assert nearest_rank([42.0], 50.0) == 42.0
        assert nearest_rank([42.0], 99.0) == 42.0

    def test_out_of_range_percentile_is_clamped(self):
        values = [1.0, 2.0, 3.0]
        assert nearest_rank(values, -10.0) == 1.0
        assert nearest_rank(values, 500.0) == 3.0

    def test_empty_input_raises_rather_than_inventing_a_value(self):
        with pytest.raises(ValueError):
            nearest_rank([], 50.0)


class TestRollingWindow:
    def test_summary_of_a_known_set(self):
        window = RollingWindow(capacity=100)
        window.extend(float(i) for i in range(1, 101))
        s = window.summarize()
        assert s.count == 100
        assert s.p50 == 50.0
        assert s.p95 == 95.0
        assert s.p99 == 99.0
        assert s.minimum == 1.0
        assert s.maximum == 100.0
        assert s.mean == pytest.approx(50.5)

    def test_empty_window_summarizes_to_none_not_zero(self):
        # A monitor reporting p95=0.0 for a stage that never produced a sample reads as an
        # excellent result rather than a missing one.
        assert RollingWindow().summarize() is None

    def test_window_evicts_oldest_samples(self):
        window = RollingWindow(capacity=3)
        window.extend([1.0, 2.0, 3.0, 4.0])
        assert len(window) == 3
        assert window.summarize().minimum == 2.0

    def test_a_bad_first_second_stops_haunting_the_numbers(self):
        # The point of a bounded window: an early spike must age out, or a rough startup
        # would depress the reported p95 for the rest of the flight.
        window = RollingWindow(capacity=10)
        window.extend([9999.0] * 10)
        assert window.summarize().p95 == 9999.0
        window.extend([10.0] * 10)
        assert window.summarize().p95 == 10.0

    def test_is_full_reports_whether_a_p99_is_trustworthy(self):
        # A p99 over eleven samples is the maximum wearing a more authoritative name.
        window = RollingWindow(capacity=5)
        window.extend([1.0, 2.0])
        assert window.is_full is False
        window.extend([3.0, 4.0, 5.0])
        assert window.is_full is True

    def test_capacity_below_one_is_rejected(self):
        with pytest.raises(ValueError):
            RollingWindow(capacity=0)

    def test_clear_resets_samples_and_rejections(self):
        window = RollingWindow()
        window.extend([1.0, float('nan')])
        window.clear()
        assert len(window) == 0
        assert window.rejected == 0
        assert window.summarize() is None


class TestSampleRejection:
    @pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
    def test_non_finite_samples_are_rejected(self, bad):
        window = RollingWindow()
        assert window.add(bad) is False
        assert len(window) == 0

    def test_negative_latency_is_rejected_not_clamped(self):
        # A negative latency means the two clocks disagree. Clamping to zero would fold a
        # clock problem into the statistics as a suspiciously fast frame -- exactly the kind
        # of thing that makes a budget look met when it is not.
        window = RollingWindow()
        assert window.add(-5.0) is False
        assert len(window) == 0

    def test_zero_latency_is_accepted(self):
        # Zero is implausible but not impossible, and unlike a negative it is not evidence
        # of a broken clock.
        assert RollingWindow().add(0.0) is True

    def test_non_numeric_sample_is_rejected_rather_than_raising(self):
        window = RollingWindow()
        assert window.add('slow') is False

    def test_rejections_are_counted_and_surfaced(self):
        # A stage quietly dropping half its samples would otherwise show a healthy p95
        # computed from the few that survived.
        window = RollingWindow()
        window.extend([10.0, float('nan'), -1.0, 20.0])
        s = window.summarize()
        assert s.count == 2
        assert s.rejected == 2

    def test_a_window_of_only_bad_samples_summarizes_to_none(self):
        window = RollingWindow()
        window.extend([float('nan'), -1.0])
        assert window.summarize() is None


class TestStageAggregator:
    def test_stages_are_tracked_independently(self):
        agg = StageAggregator()
        agg.add('INFERENCE', 120.0)
        agg.add('SAFETY', 2.0)
        assert agg.stages() == ['INFERENCE', 'SAFETY']
        assert agg.summary('INFERENCE').p50 == 120.0
        assert agg.summary('SAFETY').p50 == 2.0

    def test_unknown_stage_is_recorded_rather_than_dropped(self):
        # Silently dropping an unrecognised stage would hide the case most worth seeing: a
        # new stage emitting reports nobody is reading.
        agg = StageAggregator()
        agg.add('SOME_NEW_STAGE', 5.0)
        assert 'SOME_NEW_STAGE' in agg.stages()

    def test_summary_for_an_unseen_stage_is_none(self):
        assert StageAggregator().summary('NOPE') is None

    def test_end_to_end_uses_reported_cumulative_not_a_sum_of_stages(self):
        # Summing per-stage p95s assumes every stage hits its worst case on the same frame,
        # which is not what p95 means and overstates the total badly.
        agg = StageAggregator()
        agg.add('DECODE', 15.0, cumulative_latency_ms=40.0)
        agg.add('INFERENCE', 120.0, cumulative_latency_ms=160.0)
        e2e = agg.end_to_end()
        assert e2e.count == 2
        assert e2e.maximum == 160.0
        assert e2e.maximum < 15.0 + 120.0 + 160.0

    def test_end_to_end_is_none_before_any_cumulative_report(self):
        agg = StageAggregator()
        agg.add('SAFETY', 2.0)
        assert agg.end_to_end() is None

    def test_summaries_skips_stages_with_no_valid_samples(self):
        agg = StageAggregator()
        agg.add('GOOD', 10.0)
        agg.add('BAD', float('nan'))
        summaries = agg.summaries()
        assert 'GOOD' in summaries
        assert 'BAD' not in summaries

    def test_reset_clears_every_stage(self):
        agg = StageAggregator()
        agg.add('INFERENCE', 100.0, cumulative_latency_ms=200.0)
        agg.reset()
        assert agg.summary('INFERENCE') is None
        assert agg.end_to_end() is None


class TestBudget:
    def _summary(self, p95: float) -> Summary:
        return Summary(count=100, p50=p95 / 2, p95=p95, p99=p95,
                       minimum=0.0, maximum=p95, mean=p95 / 2)

    def test_within_budget_is_not_flagged(self):
        verdict = check_budget('INFERENCE', self._summary(120.0), 150.0)
        assert verdict.over is False
        assert verdict.headroom_ms == pytest.approx(30.0)
        assert verdict.utilization == pytest.approx(0.8)

    def test_over_budget_is_flagged(self):
        verdict = check_budget('INFERENCE', self._summary(200.0), 150.0)
        assert verdict.over is True
        assert verdict.headroom_ms < 0

    def test_exactly_at_budget_is_not_over(self):
        assert check_budget('X', self._summary(150.0), 150.0).over is False

    def test_judged_on_p95_not_the_mean(self):
        # The distinction the whole monitor exists for: a mean comfortably inside budget
        # while one frame in twenty misses it is a system that intermittently reacts late,
        # and the mean is exactly the statistic that hides it.
        summary = Summary(count=100, p50=50.0, p95=280.0, p99=290.0,
                          minimum=40.0, maximum=300.0, mean=60.0)
        verdict = check_budget('END_TO_END', summary, 263.0)
        assert verdict.over is True
        assert summary.mean < 263.0

    def test_no_samples_yields_no_verdict(self):
        assert check_budget('INFERENCE', None, 150.0) is None

    @pytest.mark.parametrize('bad', [0.0, -1.0, float('nan')])
    def test_a_meaningless_budget_yields_no_verdict(self, bad):
        assert check_budget('X', self._summary(10.0), bad) is None


class TestArchitectureBudget:
    """Sanity-check the aggregation against the table in docs/ARCHITECTURE.md section 6."""

    def test_a_nominal_pipeline_fits_the_end_to_end_budget(self):
        agg = StageAggregator()
        # Mid-range values from the table.
        for _ in range(50):
            agg.add('CAPTURE', 32.0)
            agg.add('TRANSPORT', 15.0)
            agg.add('DECODE', 15.0)
            agg.add('INFERENCE', 115.0)
            agg.add('SAFETY', 5.0)
            agg.add('BRIDGE', 15.0, cumulative_latency_ms=197.0)

        verdict = check_budget('END_TO_END', agg.end_to_end(), 300.0)
        assert verdict.over is False
        assert verdict.utilization < 1.0

    def test_a_slow_model_alone_can_blow_the_budget(self):
        # The documented failure mode: inference is the dominant term, and the thin headroom
        # to 300 ms means it does not need to be much slower than target to exceed it.
        agg = StageAggregator()
        for _ in range(50):
            agg.add('INFERENCE', 260.0, cumulative_latency_ms=340.0)
        assert check_budget('END_TO_END', agg.end_to_end(), 300.0).over is True

    def test_occasional_spikes_show_in_p95_but_not_the_median(self):
        agg = StageAggregator()
        for i in range(100):
            agg.add('INFERENCE', 400.0 if i % 10 == 0 else 100.0)
        s = agg.summary('INFERENCE')
        assert s.p50 == 100.0
        assert s.p95 == 400.0
        assert check_budget('INFERENCE', s, 150.0).over is True


class TestSummarySerialization:
    def test_as_dict_exposes_every_field(self):
        summary = Summary(count=10, p50=1.0, p95=2.0, p99=3.0,
                          minimum=0.5, maximum=4.0, mean=1.5, rejected=2)
        d = summary.as_dict()
        assert d['count'] == 10.0
        assert d['p95'] == 2.0
        assert d['rejected'] == 2.0
        assert all(isinstance(v, float) for v in d.values())

    def test_every_value_is_finite_for_a_real_summary(self):
        window = RollingWindow()
        window.extend([1.0, 2.0, 3.0])
        assert all(math.isfinite(v) for v in window.summarize().as_dict().values())
