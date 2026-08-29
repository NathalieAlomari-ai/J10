"""``mission_manager_node`` -- owns the mission state and the instruction.

Per ``docs/ARCHITECTURE.md`` this node runs at 5 Hz and gates whether VLA output may reach
the controller. Its authority is deliberately shaped like a permission slip: it publishes a
state, an instruction, and one ``autonomy_enabled`` boolean. It holds no client to the
flight controller, cannot raise a safety limit, and cannot clear an E-stop. j10_safety reads
the boolean and decides independently what to allow -- so "cannot bypass the safety filter"
is a property of the wiring, not a rule someone has to remember.

The tick exists for one reason: entering autonomy is a decision, but *staying* in it is a
continuous claim. Conditions that were true at the transition -- a healthy EKF, no E-stop,
a live link -- can stop being true a second later, and nothing else in the system would
notice that the mission is still nominally VLA_ACTIVE. So the same guards are re-checked on
every tick and a failure demotes to HOLD.
"""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String

from j10_interfaces.msg import SafetyStatus, StreamStatus, VehicleState
from j10_interfaces.srv import SetInstruction, SetMissionState

from .state_machine import (
    ALL_STATES,
    HOLD,
    IDLE,
    LAND,
    VLA_ACTIVE,
    Guards,
    MissionStateMachine,
    VehicleFacts,
)


def _assert_state_names_match() -> None:
    """Fail at import if state_machine.py's names drift from the .srv definition.

    state_machine.py defines its own string constants so it can stay ROS-free. That is only
    safe if the two cannot diverge silently -- a mismatch would mean a service request
    naming a state the machine has never heard of, rejected as "unknown" with no clue why.
    """
    for attr in dir(SetMissionState.Request):
        if not attr.startswith('STATE_'):
            continue
        value = getattr(SetMissionState.Request, attr)
        if value not in ALL_STATES:
            raise AssertionError(
                f'SetMissionState.srv declares {attr}={value!r}, which j10_mission.'
                'state_machine does not know. Add it to ALL_STATES and the transition table.'
            )


_assert_state_names_match()


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager_node')

        # --- Parameters ------------------------------------------------------------------
        rate_hz = self.declare_parameter('tick_rate_hz', 5.0).value

        self._state_topic = self.declare_parameter(
            'vehicle_state_topic', '/j10/vehicle/state').value
        self._safety_topic = self.declare_parameter(
            'safety_status_topic', '/j10/safety/status').value
        self._video_topic = self.declare_parameter(
            'video_status_topic', '/j10/video/status').value
        self._instruction_topic = self.declare_parameter(
            'instruction_topic', '/j10/mission/instruction').value
        self._autonomy_topic = self.declare_parameter(
            'autonomy_topic', '/j10/mission/autonomy_enabled').value
        self._state_pub_topic = self.declare_parameter(
            'mission_state_topic', '/j10/mission/state').value

        guards = Guards(
            min_battery_percentage=self.declare_parameter(
                'min_battery_percentage', 0.30).value,
            max_state_age_sec=self.declare_parameter('max_state_age_sec', 1.0).value,
            require_ekf_for_arm=self.declare_parameter('require_ekf_for_arm', True).value,
            require_guided_for_autonomy=self.declare_parameter(
                'require_guided_for_autonomy', True).value,
            require_video_for_autonomy=self.declare_parameter(
                'require_video_for_autonomy', False).value,
            min_autonomy_altitude_m=self.declare_parameter(
                'min_autonomy_altitude_m', 0.3).value,
        )

        # The state machine cannot tell SITL from a real autopilot, so honouring `force` is
        # decided here, by configuration, rather than inferred. Default off: a request that
        # skips preflight checks should have to be enabled deliberately.
        self._allow_force = self.declare_parameter('allow_force', False).value

        self._machine = MissionStateMachine(guards)
        self._lock = threading.Lock()

        # --- Inputs ----------------------------------------------------------------------
        self._vehicle: Optional[VehicleState] = None
        self._vehicle_stamp_sec = 0.0
        self._estop_latched = False
        self._video_ok = False

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        # Latched: j10_safety and j10_vla must see the current instruction and permission
        # even if they start, or restart, after the mission has already moved on.
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(VehicleState, self._state_topic, self._on_state, reliable)
        self.create_subscription(SafetyStatus, self._safety_topic, self._on_safety, reliable)
        self.create_subscription(StreamStatus, self._video_topic, self._on_video, reliable)

        self._instruction_pub = self.create_publisher(
            String, self._instruction_topic, latched)
        self._autonomy_pub = self.create_publisher(Bool, self._autonomy_topic, latched)
        self._mission_state_pub = self.create_publisher(
            String, self._state_pub_topic, latched)

        self.create_service(
            SetMissionState, '/j10/mission/set_state', self._on_set_state)
        self.create_service(
            SetInstruction, '/j10/mission/set_instruction', self._on_set_instruction)

        # Publish the initial values immediately. A latched topic with nothing on it is
        # indistinguishable from a node that has not started, and j10_safety would sit
        # waiting rather than defaulting to autonomy-off.
        self._publish_all()

        self.create_timer(1.0 / max(rate_hz, 0.1), self._tick)

        self.get_logger().info(
            f'mission_manager_node up at {rate_hz:.1f} Hz, state={self._machine.state}')
        self.get_logger().info(
            f'  publishing: {self._instruction_topic} {self._autonomy_topic} '
            f'{self._state_pub_topic}')
        if self._allow_force:
            self.get_logger().warning(
                'allow_force is TRUE -- transitions may skip preflight guards. '
                'This is a SITL setting and must be false against a real autopilot.')

    # -- Subscriptions ---------------------------------------------------------------------

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg: VehicleState) -> None:
        with self._lock:
            self._vehicle = msg
            self._vehicle_stamp_sec = self._now_sec()

    def _on_safety(self, msg: SafetyStatus) -> None:
        with self._lock:
            self._estop_latched = msg.state == SafetyStatus.STATE_ESTOP

    def _on_video(self, msg: StreamStatus) -> None:
        with self._lock:
            self._video_ok = msg.state == StreamStatus.STATE_STREAMING

    def _facts(self) -> VehicleFacts:
        """Snapshot the world for the state machine, which has no clock of its own."""
        with self._lock:
            vehicle = self._vehicle
            stamp = self._vehicle_stamp_sec
            estop = self._estop_latched
            video_ok = self._video_ok
        if vehicle is None:
            # inf age, not zero: never having heard from the vehicle must read as maximally
            # stale, or every freshness guard would pass before the first message arrives.
            return VehicleFacts(age_sec=math.inf, estop_latched=estop, video_ok=video_ok)
        return VehicleFacts(
            connected=vehicle.connected,
            armed=vehicle.armed,
            guided=vehicle.guided,
            ekf_healthy=vehicle.ekf_healthy,
            battery_percentage=vehicle.battery_percentage,
            altitude_m=vehicle.pose.pose.position.z,
            age_sec=max(0.0, self._now_sec() - stamp),
            estop_latched=estop,
            video_ok=video_ok,
        )

    # -- Services ----------------------------------------------------------------------------

    def _on_set_state(self, request, response):
        force = bool(request.force)
        if force and not self._allow_force:
            response.success = False
            response.message = ('force refused: allow_force is false on this node. '
                                'It is a SITL-only setting.')
            response.current_state = self._machine.state
            self.get_logger().warning(
                f'refused forced transition to {request.state}: allow_force is false')
            return response

        result = self._machine.request(request.state, self._facts(), force=force)
        response.success = result.accepted
        response.message = result.message
        response.current_state = result.state

        if result.accepted:
            self.get_logger().info(f'mission: {result.message}')
            self._publish_all()
        else:
            # Guard failures are routine (the vehicle saying "not yet") while illegal
            # transitions indicate a caller bug, so they get different levels.
            log = self.get_logger().warning if result.guard_failed else self.get_logger().error
            log(f'mission: transition to {request.state} refused: {result.message}')
        return response

    def _on_set_instruction(self, request, response):
        ok, message = self._machine.set_instruction(request.instruction)
        response.success = ok
        response.message = message
        if not ok:
            self.get_logger().warning(f'instruction rejected: {message}')
            return response

        self.get_logger().info(f'mission: {message}')
        self._publish_instruction()

        # An instruction cleared mid-autonomy leaves the model with nothing to act on, so
        # continuing to permit autonomy would mean flying on a prompt that no longer exists.
        if not self._machine.instruction and self._machine.state == VLA_ACTIVE:
            self._demote('instruction was cleared while autonomous')
        return response

    # -- Tick --------------------------------------------------------------------------------

    def _tick(self) -> None:
        """Re-check that the current state is still *earned*, not merely entered."""
        state = self._machine.state
        if state != VLA_ACTIVE:
            # Only autonomy makes a continuous claim about vehicle health. Demoting out of
            # LAND or HOLD on a stale message would fight the operator during exactly the
            # situations those states exist for.
            self._publish_autonomy()
            return

        reason = self._machine.autonomy_still_valid(self._facts())
        if reason is not None:
            self._demote(reason)
        else:
            self._publish_autonomy()

    def _demote(self, reason: str) -> None:
        """Drop out of autonomy into HOLD.

        HOLD rather than LAND: losing the *permission* to fly autonomously is not the same
        as losing the ability to fly. HOLD stops the model driving while leaving the vehicle
        where it is, which is recoverable; automatically landing on a one-frame telemetry
        gap would turn a blip into an aborted flight. The safety filter is the component
        that escalates to landing, and it does so on its own evidence.
        """
        result = self._machine.request(HOLD, self._facts())
        self.get_logger().warning(f'autonomy revoked: {reason}; -> {self._machine.state}')
        if not result.accepted:
            # Should be unreachable: HOLD is unguarded and reachable from everywhere. Log
            # loudly rather than silently continuing to publish autonomy_enabled=true.
            self.get_logger().error(
                f'FAILED to demote out of autonomy: {result.message}')
        self._publish_all()

    # -- Publishing --------------------------------------------------------------------------

    def _publish_all(self) -> None:
        self._publish_instruction()
        self._publish_autonomy()
        msg = String()
        msg.data = self._machine.state
        self._mission_state_pub.publish(msg)

    def _publish_instruction(self) -> None:
        msg = String()
        msg.data = self._machine.instruction
        self._instruction_pub.publish(msg)

    def _publish_autonomy(self) -> None:
        msg = Bool()
        msg.data = self._machine.autonomy_enabled()
        self._autonomy_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MissionManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
