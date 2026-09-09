"""Exponential-moving-average smoothing for the node's output commands.

Stateful (needs the previous output), so it can't be a pure function like the decision
logic in ``obstacle_avoidance.py`` — kept as its own tiny class instead so it's still
directly unit-testable without a camera, a node, or shared memory.
"""

from __future__ import annotations


class EmaSmoother:
    """``alpha=1.0`` disables smoothing (each update passes straight through); smaller
    values react more slowly to a new value but suppress single-frame noise/jitter."""

    def __init__(self, alpha: float):
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha must be in (0, 1], got {alpha}")
        self._alpha = alpha
        self._value = 0.0
        self._initialized = False

    def update(self, new_value: float) -> float:
        if not self._initialized:
            self._value = new_value
            self._initialized = True
        else:
            self._value = self._alpha * new_value + (1.0 - self._alpha) * self._value
        return self._value

    def reset(self) -> None:
        self._value = 0.0
        self._initialized = False
