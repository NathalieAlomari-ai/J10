"""Rolling percentile aggregation, with no ROS in it.

``docs/ARCHITECTURE.md`` section 6 sets a 300 ms end-to-end budget and then says the part
that matters:

    End-to-end 133-263 ms. Headroom to 300 ms is thin -- measure, don't assume.

This module is the measuring. It is deliberately boring and deliberately tested, because a
latency monitor that is subtly wrong is worse than none at all: it converts "we don't know"
into a number people then trust.

Two decisions worth stating up front:

* **Nearest-rank percentiles, not interpolated.** p95 here is a value that actually
  occurred, not an average of two neighbours. For a latency budget the honest question is
  "what did the 95th-worst frame actually cost", and interpolation invents a figure no
  frame ever produced -- always slightly optimistic at the tail, which is the one place
  optimism is not wanted.
* **A bounded window, evaluated on demand.** Percentiles over all history would let a bad
  first ten seconds haunt the numbers for the rest of a flight, and would make the monitor's
  memory grow without limit on a long run.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, List, Optional


@dataclass(frozen=True)
class Summary:
    """A window's worth of samples, reduced."""

    count: int
    p50: float
    p95: float
    p99: float
    minimum: float
    maximum: float
    mean: float
    #: Samples rejected as non-finite or negative since the window was created. Surfaced
    #: rather than hidden: a stage quietly dropping half its samples would otherwise show a
    #: healthy p95 computed from the few that survived.
    rejected: int = 0

    def as_dict(self) -> Dict[str, float]:
        return {
            'count': float(self.count),
            'p50': self.p50,
            'p95': self.p95,
            'p99': self.p99,
            'min': self.minimum,
            'max': self.maximum,
            'mean': self.mean,
            'rejected': float(self.rejected),
        }


def nearest_rank(sorted_values: List[float], percentile: float) -> float:
    """The nearest-rank percentile of an already-sorted, non-empty list.

    Rank is ``ceil(p/100 * n)``, clamped to ``[1, n]``. p0 and p100 give the min and max.
    """
    if not sorted_values:
        raise ValueError('nearest_rank on an empty list')
    p = min(max(percentile, 0.0), 100.0)
    rank = math.ceil(p / 100.0 * len(sorted_values))
    index = min(max(rank, 1), len(sorted_values)) - 1
    return sorted_values[index]


class RollingWindow:
    """Fixed-capacity sample window with percentile reduction.

    :param capacity: samples retained. At the 30 Hz the video path runs at, 300 is ten
        seconds -- long enough for a stable p99, short enough that the numbers still
        describe now rather than the whole flight.
    """

    def __init__(self, capacity: int = 300) -> None:
        if capacity < 1:
            raise ValueError(f'capacity must be >= 1, got {capacity}')
        self._samples: Deque[float] = deque(maxlen=capacity)
        self._rejected = 0

    def add(self, value: float) -> bool:
        """Record one sample. Returns False if it was rejected.

        Non-finite and negative values are rejected rather than clamped. A negative latency
        means the two clocks involved disagree, and clamping it to zero would fold a clock
        problem into the timing statistics as a suspiciously fast frame -- which is exactly
        the kind of thing that makes a budget look met when it is not.
        """
        try:
            sample = float(value)
        except (TypeError, ValueError):
            self._rejected += 1
            return False
        if not math.isfinite(sample) or sample < 0.0:
            self._rejected += 1
            return False
        self._samples.append(sample)
        return True

    def extend(self, values: Iterable[float]) -> None:
        for value in values:
            self.add(value)

    def clear(self) -> None:
        self._samples.clear()
        self._rejected = 0

    def __len__(self) -> int:
        return len(self._samples)

    @property
    def rejected(self) -> int:
        return self._rejected

    @property
    def is_full(self) -> bool:
        """True once the window holds a full capacity of samples.

        Worth checking before trusting a p99: a p99 over eleven samples is just the maximum
        wearing a more authoritative name.
        """
        return len(self._samples) == self._samples.maxlen

    def summarize(self) -> Optional[Summary]:
        """Reduce the window, or None when it is empty.

        None rather than a zero-filled Summary: no samples is not the same as samples of
        zero, and a monitor reporting p95=0.0 for a stage that has never produced a sample
        would read as an excellent result instead of a missing one.
        """
        if not self._samples:
            return None
        ordered = sorted(self._samples)
        return Summary(
            count=len(ordered),
            p50=nearest_rank(ordered, 50.0),
            p95=nearest_rank(ordered, 95.0),
            p99=nearest_rank(ordered, 99.0),
            minimum=ordered[0],
            maximum=ordered[-1],
            mean=sum(ordered) / len(ordered),
            rejected=self._rejected,
        )


class StageAggregator:
    """One :class:`RollingWindow` per pipeline stage, created on first sight.

    Stage names are not validated against a list. The set in LatencyReport.msg is the
    contract, but a monitor that silently dropped an unrecognised stage would hide exactly
    the case worth seeing -- a new stage emitting reports nobody is reading.
    """

    def __init__(self, capacity: int = 300) -> None:
        self._capacity = capacity
        self._stages: Dict[str, RollingWindow] = {}
        self._end_to_end = RollingWindow(capacity)

    def add(self, stage: str, stage_latency_ms: float,
            cumulative_latency_ms: Optional[float] = None) -> None:
        window = self._stages.get(stage)
        if window is None:
            window = RollingWindow(self._capacity)
            self._stages[stage] = window
        window.add(stage_latency_ms)
        if cumulative_latency_ms is not None:
            self._end_to_end.add(cumulative_latency_ms)

    def stages(self) -> List[str]:
        return sorted(self._stages)

    def summary(self, stage: str) -> Optional[Summary]:
        window = self._stages.get(stage)
        return window.summarize() if window else None

    def summaries(self) -> Dict[str, Summary]:
        out: Dict[str, Summary] = {}
        for name, window in self._stages.items():
            summary = window.summarize()
            if summary is not None:
                out[name] = summary
        return out

    def end_to_end(self) -> Optional[Summary]:
        """Cumulative latency across all stages that reported one.

        Taken from the reports' own ``cumulative_latency_ms`` rather than by summing stage
        p95s. Summing per-stage percentiles overstates the total badly -- it assumes every
        stage hits its worst case on the same frame, which is not what p95 means.
        """
        return self._end_to_end.summarize()

    def reset(self) -> None:
        for window in self._stages.values():
            window.clear()
        self._end_to_end.clear()


@dataclass(frozen=True)
class BudgetVerdict:
    """Whether a measured p95 meets its target."""

    stage: str
    p95_ms: float
    budget_ms: float
    #: True when p95 exceeds the budget.
    over: bool
    #: Fraction of budget consumed. 1.0 is exactly at target.
    utilization: float

    @property
    def headroom_ms(self) -> float:
        return self.budget_ms - self.p95_ms


def check_budget(stage: str, summary: Optional[Summary],
                 budget_ms: float) -> Optional[BudgetVerdict]:
    """Judge *summary* against *budget_ms*, or None when there is nothing to judge.

    Judged on p95, not the mean. A mean comfortably inside budget while one frame in twenty
    misses it is a system that intermittently reacts late, and the mean is precisely the
    statistic that hides it.
    """
    if summary is None:
        return None
    if budget_ms <= 0.0 or not math.isfinite(budget_ms):
        return None
    return BudgetVerdict(
        stage=stage,
        p95_ms=summary.p95,
        budget_ms=budget_ms,
        over=summary.p95 > budget_ms,
        utilization=summary.p95 / budget_ms,
    )
