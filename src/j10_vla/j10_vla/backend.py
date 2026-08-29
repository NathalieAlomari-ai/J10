"""The VLA backend contract, and the plumbing every backend shares.

Deliberately free of ROS imports. A backend takes an :class:`Observation` and returns a
:class:`Decision`; neither type knows what a topic is. That keeps the interesting logic
testable with plain pytest and no simulator, and it means swapping the Phase 4 scripted
policy for a real checkpoint in Phase 5 touches exactly one class.

The node owns everything the backend must not: threading, QoS, message conversion, and the
freshness rules that decide whether a decision is even allowed to be published.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Optional

# Action taxonomy, mirrored from j10_interfaces/msg/NavIntent.msg.
#
# Duplicated rather than imported so this module stays ROS-free and unit-testable. The node
# asserts these against the generated message constants at import time, so the two cannot
# drift silently -- see inference_node._assert_action_constants_match.
ACTION_HOLD = 0
ACTION_MOVE = 1
ACTION_TURN = 2
ACTION_EXPLORE = 3
ACTION_LAND = 4

ACTION_NAMES = {
    ACTION_HOLD: 'HOLD',
    ACTION_MOVE: 'MOVE',
    ACTION_TURN: 'TURN',
    ACTION_EXPLORE: 'EXPLORE',
    ACTION_LAND: 'LAND',
}


def _finite(value: Any, default: float = 0.0) -> float:
    """Coerce to a finite float, mapping NaN/inf/garbage to *default*.

    Every number crossing the backend boundary goes through this. A model that emits NaN is
    a real possibility -- an unlucky softmax, a bad checkpoint, a division by a zero depth --
    and NaN propagates silently through arithmetic all the way to a setpoint. Catching it
    here means the worst a broken backend can produce is a zero, which is hover.
    """
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


@dataclass(frozen=True)
class VehicleSnapshot:
    """What a policy is allowed to know about the vehicle.

    A deliberately thin slice of j10_interfaces/VehicleState. A policy that needs more than
    this is reaching for authority it should not have -- arming, mode, and geometry are the
    mission manager's and the safety filter's business, not the model's.
    """

    armed: bool = False
    mode: str = ''
    altitude_m: float = 0.0
    #: Seconds since this snapshot's source message. The node fills it in; a backend may
    #: use it to lower confidence when flying on stale state.
    age_sec: float = math.inf


@dataclass(frozen=True)
class Observation:
    """One inference input: what the vehicle sees, is told, and is doing."""

    #: Opaque to this module -- the node passes whatever its image bridge produced (a numpy
    #: array in practice). None means no frame has arrived yet, or the last one went stale.
    image: Optional[Any] = None
    #: Capture time of :attr:`image` in seconds, reconstructed upstream from the RTP
    #: timestamp rather than arrival time. None when there is no image.
    image_stamp_sec: Optional[float] = None
    #: The active natural-language instruction. Empty means the mission manager has not
    #: issued one; a backend should treat that as "no autonomy requested", not as freedom.
    instruction: str = ''
    vehicle: Optional[VehicleSnapshot] = None
    #: Current time in the same clock as :attr:`image_stamp_sec`.
    now_sec: float = 0.0

    @property
    def image_age_sec(self) -> float:
        """Seconds between the frame's capture and now; inf when there is no frame."""
        if self.image is None or self.image_stamp_sec is None:
            return math.inf
        return max(0.0, self.now_sec - self.image_stamp_sec)


@dataclass(frozen=True)
class Decision:
    """One inference output: a *request*, never a command.

    The velocity here is body-FLU (x forward, y left, z up) and is what the policy would
    like to happen. Between this and the motors sit the motion controller's accel shaping
    and the safety filter's veto, and neither is optional.
    """

    action_type: int = ACTION_HOLD
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    #: Seconds this decision stays valid. The motion controller decays to hover once it
    #: elapses, so a backend cannot win indefinite authority by going silent.
    duration_sec: float = 0.5
    confidence: float = 0.0
    #: Free text for logs and post-flight review. Never parsed by anything.
    rationale: str = ''

    def sanitized(self, max_duration_sec: float) -> 'Decision':
        """Return a copy with every field forced into a range the rest of the stack accepts.

        Applied by the node to *every* decision, including its own scripted one. A backend
        is untrusted input by construction: in Phase 5 it is a neural network, and the
        contract downstream (positive duration, confidence in [0, 1], finite velocities)
        has to hold no matter what came out of it.
        """
        action = self.action_type if self.action_type in ACTION_NAMES else ACTION_HOLD

        # Note the ordering: _finite maps NaN *and* inf to 0.0 first, so both land on the
        # floor below rather than the ceiling. That is deliberate. A finite 3600.0 is a
        # backend asking for more authority than it may have, and capping it at the ceiling
        # is the right answer. inf is a backend that is broken, and a broken backend should
        # get the least authority available, not the most -- so it expires on arrival and
        # the controller hovers, exactly as it does for NaN.
        duration = _finite(self.duration_sec, 0.0)
        # The floor keeps the value positive: a literal 0.0 would expire instantly too, but
        # it reads downstream as an ordinary hover and hides the arithmetic fault. The
        # ceiling stops any single decision from outliving the evidence it was made from.
        duration = min(max(duration, 1e-3), max(max_duration_sec, 1e-3))

        confidence = min(max(_finite(self.confidence, 0.0), 0.0), 1.0)

        return Decision(
            action_type=action,
            vx=_finite(self.vx),
            vy=_finite(self.vy),
            vz=_finite(self.vz),
            yaw_rate=_finite(self.yaw_rate),
            duration_sec=duration,
            confidence=confidence,
            rationale=self.rationale if isinstance(self.rationale, str) else '',
        )


#: The one decision that is always safe to publish, whatever went wrong.
HOLD = Decision(action_type=ACTION_HOLD, duration_sec=0.5, confidence=1.0,
                rationale='hold')


@dataclass
class BackendInfo:
    """What a backend reports about itself, for logs and the telemetry record."""

    name: str
    #: False for anything that does not run a learned model -- the scripted Phase 4 policy,
    #: a replay harness, a test double. Surfaced in logs so a scripted run is never mistaken
    #: for a model run when reading them back later.
    is_model: bool = False
    details: dict = field(default_factory=dict)


class Backend(ABC):
    """A source of navigation decisions.

    Implementations must be synchronous and self-contained: the node calls :meth:`infer`
    from a dedicated worker thread and does not call it again until it returns. Blocking is
    therefore allowed and expected -- a real checkpoint takes 80-150 ms -- but the call must
    always terminate. Nothing upstream can interrupt it.
    """

    @abstractmethod
    def info(self) -> BackendInfo:
        """Describe this backend. Called once at startup and written to the log."""

    def load(self) -> None:
        """Prepare for inference: read weights, warm up kernels, allocate buffers.

        Called once, before any :meth:`infer`, and allowed to be slow. Separate from
        ``__init__`` so construction stays cheap and a failure to load is reported by a node
        that already exists and can log properly.
        """

    @abstractmethod
    def infer(self, observation: Observation) -> Decision:
        """Produce a decision for *observation*.

        Raising is permitted -- the node catches, logs, and publishes hover. Returning a
        malformed :class:`Decision` is also survivable; see :meth:`Decision.sanitized`.
        """

    def shutdown(self) -> None:
        """Release anything :meth:`load` acquired. Called once, best effort."""
