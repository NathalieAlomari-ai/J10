"""The mission state machine, with no ROS in it.

``docs/ARCHITECTURE.md`` gives this node one job and one prohibition:

    State machine: IDLE -> PREFLIGHT -> ARMED -> TAKEOFF -> VLA_ACTIVE -> HOLD -> LAND ->
    DISARM. Owns the natural-language instruction and gates whether VLA output may reach
    the controller. **Cannot bypass the safety filter.**

The prohibition is structural, not a rule anyone has to remember: this module produces a
state and a single ``autonomy_enabled`` boolean, and neither can raise a limit, clear the
E-stop, or reach the flight controller. The most permissive thing it can say is "autonomy
is allowed", and j10_safety independently decides what to do with that.

Keeping it ROS-free means the transition table -- the part with the interesting edge cases
-- is testable with plain pytest, no simulator and no running graph.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Tuple

# State names, mirrored from j10_interfaces/srv/SetMissionState.srv. Strings rather than an
# enum because the service carries them as strings, and a round-trip through int would add
# a mapping that could drift.
IDLE = 'IDLE'
PREFLIGHT = 'PREFLIGHT'
ARMED = 'ARMED'
TAKEOFF = 'TAKEOFF'
VLA_ACTIVE = 'VLA_ACTIVE'
HOLD = 'HOLD'
LAND = 'LAND'
DISARM = 'DISARM'

ALL_STATES: FrozenSet[str] = frozenset(
    {IDLE, PREFLIGHT, ARMED, TAKEOFF, VLA_ACTIVE, HOLD, LAND, DISARM})

#: The only state in which VLA output may reach the motion controller.
#:
#: Deliberately a single state, not a set. Every widening of this is a widening of when a
#: model may drive the vehicle, and it should require editing this line and reading this
#: comment.
AUTONOMOUS_STATES: FrozenSet[str] = frozenset({VLA_ACTIVE})

#: Legal transitions. Anything absent is rejected -- the table is a whitelist.
#:
#: Note what is deliberately *not* here: IDLE -> VLA_ACTIVE, ARMED -> VLA_ACTIVE, or any
#: other shortcut into autonomy. Reaching VLA_ACTIVE requires passing through PREFLIGHT and
#: TAKEOFF, so "the model is flying" is never one call away from a cold start.
_TRANSITIONS: Dict[str, FrozenSet[str]] = {
    IDLE:       frozenset({PREFLIGHT}),
    PREFLIGHT:  frozenset({ARMED, IDLE}),
    ARMED:      frozenset({TAKEOFF, DISARM}),
    TAKEOFF:    frozenset({VLA_ACTIVE, HOLD, LAND}),
    VLA_ACTIVE: frozenset({HOLD, LAND}),
    HOLD:       frozenset({VLA_ACTIVE, LAND}),
    LAND:       frozenset({DISARM, HOLD}),
    DISARM:     frozenset({IDLE}),
}

#: Reachable from anywhere, including from themselves. Getting down and getting safe must
#: never be blocked by which state the machine happens to be in -- a transition table that
#: can trap the vehicle in the air is a worse bug than any it prevents.
_ALWAYS_ALLOWED: FrozenSet[str] = frozenset({LAND, HOLD})


@dataclass(frozen=True)
class VehicleFacts:
    """What the manager knows about the vehicle when judging a transition.

    A snapshot, passed in rather than read, so the state machine has no clock and no
    subscriptions and every test can state the world exactly.
    """

    connected: bool = False
    armed: bool = False
    guided: bool = False
    ekf_healthy: bool = False
    battery_percentage: float = 0.0
    altitude_m: float = 0.0
    #: Age of the underlying VehicleState message. inf means none has ever arrived.
    age_sec: float = math.inf
    #: True when the safety filter has an E-stop latched. Nothing may enter or resume
    #: autonomy while this is set.
    estop_latched: bool = False
    #: True when the video link is healthy enough for a model to see. Only consulted when
    #: require_video_for_autonomy is on -- it is off until Phase 3 lands j10_video.
    video_ok: bool = False


@dataclass(frozen=True)
class Guards:
    """Preflight thresholds. Deliberately conservative; Phase 7 tightens, never loosens."""

    min_battery_percentage: float = 0.30
    max_state_age_sec: float = 1.0
    require_ekf_for_arm: bool = True
    require_guided_for_autonomy: bool = True
    require_video_for_autonomy: bool = False
    #: Refuse to hand control to the model while still on the ground. Velocity setpoints are
    #: ignored by ArduPilot until the vehicle is flying, so entering VLA_ACTIVE at zero
    #: altitude would look like autonomy while doing nothing -- a state worth refusing
    #: rather than debugging in the air.
    min_autonomy_altitude_m: float = 0.3


@dataclass
class TransitionResult:
    accepted: bool
    state: str
    message: str = ''
    #: True when the request was refused by a guard rather than by the table. Worth
    #: distinguishing in logs: an illegal transition is a caller bug, a failed guard is the
    #: vehicle saying "not yet".
    guard_failed: bool = False


class MissionStateMachine:
    """Validated transitions plus the instruction the VLA acts on."""

    def __init__(self, guards: Optional[Guards] = None,
                 initial_state: str = IDLE) -> None:
        if initial_state not in ALL_STATES:
            raise ValueError(f'unknown initial state {initial_state!r}')
        self._guards = guards or Guards()
        self._state = initial_state
        self._instruction = ''
        self._history: List[Tuple[str, str]] = []

    # -- Observation ----------------------------------------------------------------------

    @property
    def state(self) -> str:
        return self._state

    @property
    def guards(self) -> Guards:
        return self._guards

    def autonomy_still_valid(self, facts: VehicleFacts) -> Optional[str]:
        """None if autonomy may continue under *facts*, else the reason it may not.

        Entering autonomy is a decision; *staying* in it is a continuous claim. Conditions
        that held at the transition can stop holding a second later, and nothing else in the
        system would notice that the mission is still nominally VLA_ACTIVE. The caller
        re-checks this every tick.

        Deliberately not the same as the entry guard in :meth:`_check_guards`. Altitude is
        absent here: a vehicle that has legitimately descended -- because the model asked it
        to, or the safety filter is braking it down -- is not a reason to revoke autonomy,
        whereas refusing to *hand over* control on the ground is. Re-checking the entry
        guard verbatim would fight the very behaviour autonomy was granted to perform.
        """
        if facts.estop_latched:
            return 'E-stop latched'
        if not facts.connected:
            return 'flight controller disconnected'
        if facts.age_sec > self._guards.max_state_age_sec:
            age = 'never received' if math.isinf(facts.age_sec) else f'{facts.age_sec:.2f}s'
            return f'vehicle state stale ({age})'
        if not facts.armed:
            return 'vehicle disarmed'
        if self._guards.require_guided_for_autonomy and not facts.guided:
            # An operator flipping the FC out of GUIDED is taking control back. The mission
            # should follow rather than keep asserting that a model is driving.
            return 'flight controller left guided mode'
        if not facts.ekf_healthy:
            return 'EKF estimate lost'
        if self._guards.require_video_for_autonomy and not facts.video_ok:
            return 'video link lost'
        if not self._instruction:
            return 'instruction cleared'
        return None

    @property
    def instruction(self) -> str:
        return self._instruction

    @property
    def history(self) -> List[Tuple[str, str]]:
        """(from, to) pairs, oldest first. For the post-flight record."""
        return list(self._history)

    def autonomy_enabled(self) -> bool:
        """Whether VLA output may currently reach the motion controller.

        The single boolean this whole module exists to produce. j10_safety consumes it and
        decides independently what to allow -- this is a permission, never an instruction.
        """
        return self._state in AUTONOMOUS_STATES

    # -- Instruction ----------------------------------------------------------------------

    def set_instruction(self, instruction: str) -> Tuple[bool, str]:
        """Set the natural-language instruction the VLA acts on.

        Allowed in any state, including mid-flight: changing what the vehicle is being asked
        to do is not the same as changing whether it may act, and blocking it would mean the
        only way to redirect a flying vehicle is to drop out of autonomy first.
        """
        if not isinstance(instruction, str):
            return False, 'instruction must be a string'
        cleaned = instruction.strip()
        if len(cleaned) > 512:
            # A bound, not a judgement about content. An unbounded string ends up in a
            # latched topic, every log line, and every recorded intent.
            return False, 'instruction exceeds 512 characters'
        self._instruction = cleaned
        return True, f'instruction set to {cleaned!r}' if cleaned else 'instruction cleared'

    # -- Transitions ----------------------------------------------------------------------

    def can_transition(self, target: str) -> bool:
        """Whether the *table* permits target from the current state, ignoring guards."""
        if target not in ALL_STATES:
            return False
        if target in _ALWAYS_ALLOWED:
            return True
        return target in _TRANSITIONS.get(self._state, frozenset())

    def request(self, target: str, facts: VehicleFacts,
                force: bool = False) -> TransitionResult:
        """Attempt a transition to *target*.

        :param force: skip the preflight guards. Intended for SITL only -- the caller is
            responsible for refusing it against a real autopilot, since this module has no
            way to tell one from the other. It does **not** unlock illegal transitions: the
            table still applies, so force cannot jump straight into autonomy.
        """
        if target not in ALL_STATES:
            return TransitionResult(False, self._state,
                                    f'unknown state {target!r}')

        if target == self._state:
            # Idempotent rather than an error: a retried or duplicated request should not
            # look like a failure to the caller.
            return TransitionResult(True, self._state, f'already in {target}')

        if not self.can_transition(target):
            allowed = sorted(_TRANSITIONS.get(self._state, frozenset()) | _ALWAYS_ALLOWED)
            return TransitionResult(
                False, self._state,
                f'illegal transition {self._state} -> {target}; allowed: {allowed}')

        if not force:
            ok, reason = self._check_guards(target, facts)
            if not ok:
                return TransitionResult(False, self._state, reason, guard_failed=True)

        previous = self._state
        self._state = target
        self._history.append((previous, target))
        note = ' (forced, guards skipped)' if force else ''
        return TransitionResult(True, target, f'{previous} -> {target}{note}')

    # -- Guards ---------------------------------------------------------------------------

    def _battery_too_low(self, facts: VehicleFacts) -> Tuple[bool, str]:
        """(True, reason) when the battery is known to be below the minimum.

        A negative percentage means *unknown*, not empty -- MAVROS reports -0.01 when the
        autopilot sends no battery telemetry at all, which is the normal case in SITL and on
        any vehicle without a monitored pack. Comparing it against the minimum directly
        would read "unknown" as "flat" and refuse every transition.

        j10_safety already draws exactly this distinction in its battery failsafe ("Unknown
        charge (negative) is not treated as empty"). Two safety-relevant components
        disagreeing about what a negative battery means is itself a hazard, so this matches
        it deliberately rather than by coincidence.

        This does weaken preflight where telemetry is genuinely missing: the check can only
        catch a battery it can read. That is a limit of the evidence, not a bypass -- and a
        vehicle whose pack is unmonitored has no low-battery protection to give up.
        """
        if facts.battery_percentage < 0.0:
            return False, ''
        if facts.battery_percentage < self._guards.min_battery_percentage:
            return True, (f'battery {facts.battery_percentage:.0%} below minimum '
                          f'{self._guards.min_battery_percentage:.0%}')
        return False, ''

    def _check_guards(self, target: str, facts: VehicleFacts) -> Tuple[bool, str]:
        g = self._guards

        # LAND and HOLD are never guarded. Refusing to descend because telemetry looks bad
        # is precisely backwards -- bad telemetry is a reason to get down, not to stay up.
        if target in _ALWAYS_ALLOWED:
            return True, ''

        # Everything below needs a live picture of the vehicle. A stale snapshot is not
        # evidence about the present, so it is treated as no evidence at all.
        if not facts.connected:
            return False, 'flight controller not connected'
        if facts.age_sec > g.max_state_age_sec:
            age = 'never received' if math.isinf(facts.age_sec) else f'{facts.age_sec:.2f}s old'
            return False, f'vehicle state is stale ({age}, limit {g.max_state_age_sec:.2f}s)'

        if target == PREFLIGHT:
            low, why = self._battery_too_low(facts)
            if low:
                return False, why
            return True, ''

        if target == ARMED:
            if g.require_ekf_for_arm and not facts.ekf_healthy:
                return False, 'EKF is not reporting a usable estimate'
            low, why = self._battery_too_low(facts)
            if low:
                return False, why
            return True, ''

        if target == TAKEOFF:
            if not facts.armed:
                return False, 'vehicle is not armed'
            return True, ''

        if target == VLA_ACTIVE:
            # The narrowest gate in the machine, because it is the one that hands a model
            # authority over a flying vehicle.
            if facts.estop_latched:
                return False, 'E-stop is latched; reset it before resuming autonomy'
            if not facts.armed:
                return False, 'vehicle is not armed'
            if g.require_guided_for_autonomy and not facts.guided:
                return False, 'flight controller is not in a guided mode'
            if not facts.ekf_healthy:
                return False, 'EKF is not reporting a usable estimate'
            if facts.altitude_m < g.min_autonomy_altitude_m:
                return False, (f'altitude {facts.altitude_m:.2f}m below autonomy minimum '
                               f'{g.min_autonomy_altitude_m:.2f}m; take off first')
            if g.require_video_for_autonomy and not facts.video_ok:
                return False, 'video link is not healthy; the model would be flying blind'
            if not self._instruction:
                return False, 'no instruction set; nothing has been asked of the vehicle'
            return True, ''

        if target == DISARM:
            if facts.altitude_m > g.min_autonomy_altitude_m:
                # Disarming in flight cuts the motors. There is a legitimate need for that,
                # but it is an emergency action and must be asked for explicitly with force,
                # never reached by walking the normal state sequence.
                return False, (f'refusing to disarm at {facts.altitude_m:.2f}m altitude; '
                               'land first, or pass force to cut motors deliberately')
            return True, ''

        return True, ''
