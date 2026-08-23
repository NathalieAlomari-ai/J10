"""The Phase 4 backend: a fixed pattern, no model.

Phase 4 in ``docs/ARCHITECTURE.md`` is the integration milestone, and its whole point is
that the model is not a variable yet:

    Stub VLA backend emitting a fixed intent sequence. Full chain live.
    Exit: the drone flies the scripted pattern in Gazebo with fresh timing at every stage.

So this backend is deterministic on purpose. Given the same clock it produces the same
decisions in the same order, which means a failed run points at the plumbing -- timing,
QoS, arbitration, the safety envelope -- and never at "maybe the model had a bad day".

It also stands in for the real thing in two lasting ways: it exercises the exact
:class:`~j10_vla.backend.Backend` interface Phase 5 plugs into, and it can be told to spend
a realistic amount of wall-clock time per call, so the cascade is tested against a slow
producer from the start rather than discovering the latency behaviour later.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

from .backend import (
    ACTION_HOLD,
    ACTION_MOVE,
    ACTION_NAMES,
    ACTION_TURN,
    Backend,
    BackendInfo,
    Decision,
    Observation,
)

_ACTION_BY_NAME = {name: value for value, name in ACTION_NAMES.items()}


@dataclass(frozen=True)
class Step:
    """One leg of the pattern: a body-frame velocity held for a fixed time."""

    action_type: int = ACTION_HOLD
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    hold_sec: float = 1.0
    label: str = ''

    def __post_init__(self) -> None:
        if self.hold_sec <= 0.0:
            raise ValueError(f'step hold_sec must be > 0, got {self.hold_sec!r}')
        if self.action_type not in ACTION_NAMES:
            raise ValueError(f'step action_type {self.action_type!r} is not a known ACTION_*')


def parse_steps(spec: Sequence) -> List[Step]:
    """Build a pattern from a list of ``"ACTION vx vy vz yaw_rate seconds"`` strings.

    A flat list of strings rather than nested maps because ROS 2 parameters cannot express
    a list of dictionaries -- only flat arrays of a single scalar type survive the YAML
    round-trip. Keeping the pattern editable from ``config/vla.yaml`` without a code change
    is worth the small parser.

        MOVE 0.3 0.0 0.0 0.0 3.0     # forward at 0.3 m/s for 3 s
        TURN 0.0 0.0 0.0 0.4 2.0     # yaw left at 0.4 rad/s for 2 s
        HOLD 0.0 0.0 0.0 0.0 1.0     # settle for 1 s

    Raises ValueError with the offending line on any malformed entry. Failing loudly at
    startup is correct here: a pattern that silently loses a leg would fly a different shape
    than the one written down, and the run would look like a control bug.
    """
    steps: List[Step] = []
    for index, raw in enumerate(spec):
        if not isinstance(raw, str):
            raise ValueError(f'step {index}: expected a string, got {type(raw).__name__}')
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 6:
            raise ValueError(
                f'step {index}: expected "ACTION vx vy vz yaw_rate seconds" '
                f'(6 fields), got {len(parts)} in {raw!r}'
            )
        name = parts[0].upper()
        if name not in _ACTION_BY_NAME:
            raise ValueError(
                f'step {index}: unknown action {parts[0]!r}; '
                f'expected one of {sorted(_ACTION_BY_NAME)}'
            )
        try:
            vx, vy, vz, yaw_rate, hold = (float(p) for p in parts[1:])
        except ValueError as exc:
            raise ValueError(f'step {index}: non-numeric field in {raw!r}') from exc
        steps.append(
            Step(
                action_type=_ACTION_BY_NAME[name],
                vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate,
                hold_sec=hold,
                label=f'{name.lower()}[{index}]',
            )
        )
    if not steps:
        raise ValueError('pattern is empty; at least one step is required')
    return steps


#: A closed square with a settle between legs, at Phase 7 indoor speeds (0.3 m/s).
#:
#: Closed on purpose: it returns to roughly where it started, so a loop can run unattended
#: inside a small geofence without walking the vehicle into a wall. The 1 s HOLD after each
#: leg is what makes the shape readable in a log -- without it, accel shaping smears the
#: corners together and it becomes hard to tell a turn from a drift.
DEFAULT_PATTERN: Sequence[str] = (
    'MOVE 0.3 0.0 0.0 0.0 3.0',
    'HOLD 0.0 0.0 0.0 0.0 1.0',
    'TURN 0.0 0.0 0.0 0.5 3.14',
    'HOLD 0.0 0.0 0.0 0.0 1.0',
    'MOVE 0.3 0.0 0.0 0.0 3.0',
    'HOLD 0.0 0.0 0.0 0.0 1.0',
    'TURN 0.0 0.0 0.0 0.5 3.14',
    'HOLD 0.0 0.0 0.0 0.0 1.0',
)


class ScriptedBackend(Backend):
    """Walks a fixed pattern on a wall clock.

    :param steps: the pattern; defaults to :data:`DEFAULT_PATTERN`.
    :param loop: restart at the first step after the last, instead of holding forever.
    :param confidence: reported on every decision. 1.0 is honest for a deterministic
        policy -- there is nothing uncertain about it -- but it is configurable so the
        downstream low-confidence paths (the safety filter scales its limits by this) can be
        exercised without a real model.
    :param simulated_latency_sec: sleep this long inside :meth:`infer`, standing in for a
        checkpoint's forward pass. 0 disables it. Non-zero is the more honest test: the
        two-rate cascade exists precisely to absorb a slow producer.
    :param require_instruction: refuse to move until the mission manager has published an
        instruction. On by default -- see :meth:`infer`.
    """

    def __init__(
        self,
        steps: Optional[Sequence[Step]] = None,
        *,
        loop: bool = True,
        confidence: float = 1.0,
        simulated_latency_sec: float = 0.0,
        require_instruction: bool = True,
    ) -> None:
        self._steps: List[Step] = list(steps) if steps is not None else parse_steps(DEFAULT_PATTERN)
        if not self._steps:
            raise ValueError('ScriptedBackend needs at least one step')
        self._loop = loop
        self._confidence = confidence
        self._simulated_latency_sec = max(0.0, simulated_latency_sec)
        self._require_instruction = require_instruction

        self._cycle_sec = sum(step.hold_sec for step in self._steps)
        #: Wall time the pattern started, set on the first infer() rather than at
        #: construction. Node startup, backend load, and the first frame are seconds apart,
        #: and anchoring at construction would silently skip the opening legs.
        self._start_sec: Optional[float] = None
        self._completed_cycles = 0

    # -- Backend interface ---------------------------------------------------------------

    def info(self) -> BackendInfo:
        return BackendInfo(
            name='scripted',
            is_model=False,
            details={
                'steps': len(self._steps),
                'cycle_sec': round(self._cycle_sec, 3),
                'loop': self._loop,
                'simulated_latency_sec': self._simulated_latency_sec,
            },
        )

    def infer(self, observation: Observation) -> Decision:
        if self._simulated_latency_sec:
            time.sleep(self._simulated_latency_sec)

        # No instruction means the mission manager has not asked for autonomy. A real model
        # given an empty prompt would produce *something*, and that something would be
        # unrequested motion; the scripted policy refuses on the same principle rather than
        # flying whenever it happens to be running.
        if self._require_instruction and not observation.instruction.strip():
            return Decision(
                action_type=ACTION_HOLD,
                duration_sec=0.5,
                confidence=self._confidence,
                rationale='no instruction set; holding',
            )

        if self._start_sec is None:
            self._start_sec = observation.now_sec

        elapsed = max(0.0, observation.now_sec - self._start_sec)
        step = self._step_at(elapsed)
        if step is None:
            return Decision(
                action_type=ACTION_HOLD,
                duration_sec=0.5,
                confidence=self._confidence,
                rationale=f'pattern complete after {self._completed_cycles} cycle(s); holding',
            )

        # duration_sec is deliberately *not* the step's remaining time. It is a validity
        # window: how long this decision may be acted on before the consumer must assume the
        # producer is gone. Tying it to the step would hand a 3 s step 3 s of unsupervised
        # authority, so a stalled backend would keep the vehicle moving for the rest of the
        # leg. A short window re-asserted every cycle is what makes silence fail to hover.
        return Decision(
            action_type=step.action_type,
            vx=step.vx, vy=step.vy, vz=step.vz, yaw_rate=step.yaw_rate,
            duration_sec=0.5,
            confidence=self._confidence,
            rationale=f'scripted {step.label} (t+{elapsed:.1f}s)',
        )

    # -- Internals -----------------------------------------------------------------------

    def _step_at(self, elapsed: float) -> Optional[Step]:
        """The step covering *elapsed* seconds into the pattern, or None once finished."""
        if self._loop and self._cycle_sec > 0.0:
            self._completed_cycles = int(elapsed // self._cycle_sec)
            elapsed = elapsed % self._cycle_sec
        elif elapsed >= self._cycle_sec:
            self._completed_cycles = 1
            return None

        boundary = 0.0
        for step in self._steps:
            boundary += step.hold_sec
            if elapsed < boundary:
                return step
        # Only reachable on floating-point equality with the cycle end; the last step owns it.
        return self._steps[-1]

    @property
    def cycle_sec(self) -> float:
        """Total wall time of one pass through the pattern."""
        return self._cycle_sec

    @property
    def completed_cycles(self) -> int:
        """How many full passes have elapsed, for logging progress on a long run."""
        return self._completed_cycles
