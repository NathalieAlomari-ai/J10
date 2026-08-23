"""``vla_inference_node`` -- runs a backend and publishes ``NavIntent``.

The node owns everything the backend must not, and the division matters. Per
``docs/ARCHITECTURE.md`` this node is the slow half of a two-rate cascade:

    VLA (5-10 Hz, semantic intent) -> motion_controller (30 Hz) -> safety (30 Hz) -> FC

Three properties make that split safe, and all three live here rather than in any backend:

1. **Inference never runs on the executor thread.** A 150 ms forward pass on the callback
   thread would stall every subscription behind it, including the vehicle state the policy
   reads. A worker thread does the work; callbacks only ever drop a value into a box.
2. **The frame box holds one frame, and newer always wins.** Never a queue. Queuing would
   trade latency for completeness in a loop where a stale frame is worthless -- by the time
   the backlog cleared, the vehicle would be acting on where it used to be.
3. **Every published intent is bounded and expiring.** Sanitized on the way out, with a
   short validity window, so a backend that hangs or dies decays the vehicle to hover
   instead of leaving the last command standing.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Image
from std_msgs.msg import String

from j10_interfaces.msg import LatencyReport, NavIntent, VehicleState

from .backend import (
    ACTION_EXPLORE,
    ACTION_HOLD,
    ACTION_LAND,
    ACTION_MOVE,
    ACTION_TURN,
    Backend,
    Decision,
    Observation,
    VehicleSnapshot,
)
from .scripted_backend import DEFAULT_PATTERN, ScriptedBackend, parse_steps


def _assert_action_constants_match() -> None:
    """Fail at import if backend.py's action IDs drift from the .msg definition.

    backend.py duplicates the taxonomy so it can stay ROS-free and unit-testable. That is a
    worthwhile trade only if the duplicate cannot silently diverge -- a mismatch would mean
    the policy asking for MOVE and the controller reading LAND, with nothing in between
    noticing. This is the C++ side's static_assert, in the one place Python can do it.
    """
    expected = {
        'ACTION_HOLD': (ACTION_HOLD, NavIntent.ACTION_HOLD),
        'ACTION_MOVE': (ACTION_MOVE, NavIntent.ACTION_MOVE),
        'ACTION_TURN': (ACTION_TURN, NavIntent.ACTION_TURN),
        'ACTION_EXPLORE': (ACTION_EXPLORE, NavIntent.ACTION_EXPLORE),
        'ACTION_LAND': (ACTION_LAND, NavIntent.ACTION_LAND),
    }
    for name, (ours, theirs) in expected.items():
        if ours != theirs:
            raise AssertionError(
                f'{name} disagrees: j10_vla.backend has {ours}, NavIntent.msg has {theirs}. '
                'Fix backend.py to match the message definition.'
            )


_assert_action_constants_match()


def _to_seconds(stamp: TimeMsg) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _to_stamp(seconds: float) -> TimeMsg:
    stamp = TimeMsg()
    stamp.sec = int(seconds)
    stamp.nanosec = int(round((seconds - stamp.sec) * 1e9))
    # Rounding can carry nanosec to exactly 1e9, which is not a valid Time.
    if stamp.nanosec >= 1_000_000_000:
        stamp.sec += 1
        stamp.nanosec -= 1_000_000_000
    return stamp


class _Box:
    """A one-slot mailbox. Writers overwrite; the reader takes a consistent snapshot.

    Not a queue, and not a Condition either: the worker runs on its own timer and should
    read whatever is current when it gets there, never wait for a frame that may not come.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: Any = None
        self._stamp_sec: float = 0.0

    def put(self, value: Any, stamp_sec: float) -> None:
        with self._lock:
            self._value = value
            self._stamp_sec = stamp_sec

    def get(self):
        with self._lock:
            return self._value, self._stamp_sec


class VlaInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__('vla_inference_node')

        # --- Parameters ------------------------------------------------------------------
        self._rate_hz = self.declare_parameter('inference_rate_hz', 6.0).value
        self._backend_name = self.declare_parameter('backend', 'scripted').value

        self._intent_topic = self.declare_parameter(
            'intent_topic', '/j10/vla/intent').value
        self._image_topic = self.declare_parameter(
            'image_topic', '/j10/camera/image_raw').value
        self._instruction_topic = self.declare_parameter(
            'instruction_topic', '/j10/mission/instruction').value
        self._vehicle_state_topic = self.declare_parameter(
            'vehicle_state_topic', '/j10/vehicle/state').value
        self._latency_topic = self.declare_parameter(
            'latency_topic', '/j10/telemetry/latency').value

        # A frame older than this is not evidence about the present. Past it the node stops
        # treating the image as usable, which -- with require_image true -- means it stops
        # asking for motion.
        self._max_image_age_sec = self.declare_parameter('max_image_age_sec', 0.5).value
        # Off by default because Phase 4's exit criterion is flown with the scripted policy
        # and no camera. It must be ON for anything running a real model: a VLA with no
        # image is not navigating, it is guessing.
        self._require_image = self.declare_parameter('require_image', False).value
        self._max_intent_duration_sec = self.declare_parameter(
            'max_intent_duration_sec', 1.0).value

        # Scripted backend settings.
        self._pattern = self.declare_parameter('pattern', list(DEFAULT_PATTERN)).value
        self._loop_pattern = self.declare_parameter('loop_pattern', True).value
        self._scripted_confidence = self.declare_parameter('scripted_confidence', 1.0).value
        self._simulated_latency_sec = self.declare_parameter(
            'simulated_inference_latency_sec', 0.0).value
        self._require_instruction = self.declare_parameter('require_instruction', True).value

        # --- State -----------------------------------------------------------------------
        self._image_box = _Box()
        self._instruction_lock = threading.Lock()
        self._instruction = ''
        self._state_lock = threading.Lock()
        self._vehicle: Optional[VehicleSnapshot] = None
        self._vehicle_stamp_sec = 0.0
        self._sequence = 0
        self._running = threading.Event()
        self._running.set()

        self._backend = self._build_backend()

        # --- Interfaces ------------------------------------------------------------------
        # BEST_EFFORT + KEEP_LAST(1): the camera stream must never build a backlog. A
        # retransmitted stale frame is worse than a dropped one here.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # TRANSIENT_LOCAL so a restart of this node picks up the instruction the mission
        # manager published before it started, instead of sitting idle waiting for a repeat.
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._intent_pub = self.create_publisher(
            NavIntent, self._intent_topic, state_qos)
        self._latency_pub = self.create_publisher(
            LatencyReport, self._latency_topic, QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ))

        self.create_subscription(Image, self._image_topic, self._on_image, image_qos)
        self.create_subscription(
            String, self._instruction_topic, self._on_instruction, latched_qos)
        self.create_subscription(
            VehicleState, self._vehicle_state_topic, self._on_vehicle_state, state_qos)

        info = self._backend.info()
        self.get_logger().info(
            f'vla_inference_node up: backend={info.name} '
            f'(model={info.is_model}) at {self._rate_hz:.1f} Hz -> {self._intent_topic}'
        )
        self.get_logger().info(f'  backend details: {info.details}')
        self.get_logger().info(
            f'  sources: image={self._image_topic} '
            f'instruction={self._instruction_topic} state={self._vehicle_state_topic}'
        )
        if not info.is_model:
            # Loud on purpose. A scripted run that gets mistaken for a model run is a
            # misleading result, and this line is what makes the log unambiguous later.
            self.get_logger().warning(
                'Backend is NOT a model -- intents are a fixed scripted pattern. '
                'This is the Phase 4 integration configuration.'
            )
        if not self._require_image:
            self.get_logger().warning(
                'require_image is FALSE -- intents are produced with no camera frame. '
                'Correct for the Phase 4 scripted pattern; must be TRUE for a real model.'
            )

        # The worker is a plain thread, not a ROS timer, so a slow inference delays only
        # itself. A timer callback would be serialized with the subscriptions.
        self._worker = threading.Thread(target=self._worker_loop, daemon=True,
                                        name='vla-inference')
        self._worker.start()

    # -- Construction ----------------------------------------------------------------------

    def _build_backend(self) -> Backend:
        if self._backend_name != 'scripted':
            # Phase 5 registers real backends here. Failing loudly beats falling back to the
            # scripted policy: a silent downgrade would look like a working model run.
            raise ValueError(
                f"unknown backend {self._backend_name!r}; only 'scripted' is implemented. "
                'A real checkpoint backend lands in Phase 5 (see docs/ARCHITECTURE.md).'
            )
        backend = ScriptedBackend(
            parse_steps(self._pattern),
            loop=self._loop_pattern,
            confidence=self._scripted_confidence,
            simulated_latency_sec=self._simulated_latency_sec,
            require_instruction=self._require_instruction,
        )
        backend.load()
        return backend

    # -- Subscriptions ---------------------------------------------------------------------

    def _on_image(self, msg: Image) -> None:
        # Capture time, not arrival time: video_receiver_node reconstructs the header stamp
        # from the RTP timestamp, and that is the only clock that makes the end-to-end
        # latency number mean anything.
        self._image_box.put(msg, _to_seconds(msg.header.stamp))

    def _on_instruction(self, msg: String) -> None:
        with self._instruction_lock:
            changed = msg.data != self._instruction
            self._instruction = msg.data
        if changed:
            self.get_logger().info(f'instruction set: {msg.data!r}')

    def _on_vehicle_state(self, msg: VehicleState) -> None:
        with self._state_lock:
            self._vehicle = VehicleSnapshot(
                armed=msg.armed,
                mode=msg.mode,
                altitude_m=msg.pose.pose.position.z,
                age_sec=0.0,
            )
            self._vehicle_stamp_sec = self._now_sec()

    # -- Worker ----------------------------------------------------------------------------

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _worker_loop(self) -> None:
        period = 1.0 / max(self._rate_hz, 0.1)
        while self._running.is_set() and rclpy.ok():
            started = time.monotonic()
            try:
                self._step()
            except Exception as exc:  # noqa: BLE001 -- a backend may raise anything
                # Publishing hover rather than nothing is the point: going silent would let
                # the previous intent live out its validity window before the controller
                # noticed. An explicit hold stops the vehicle now.
                self.get_logger().error(f'inference failed, publishing hold: {exc}')
                try:
                    self._publish(Decision(rationale=f'backend error: {exc}'), None, 0.0)
                except Exception as publish_exc:  # noqa: BLE001
                    self.get_logger().error(f'could not publish hold: {publish_exc}')
            # Sleep the remainder of the period. If inference overran it, go straight round
            # again -- the rate is a ceiling, not a promise.
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                self._running.wait(remaining)

    def _step(self) -> None:
        now = self._now_sec()

        image_msg, image_stamp = self._image_box.get()
        image_age = (now - image_stamp) if image_msg is not None else float('inf')
        fresh = image_msg is not None and 0 <= image_age <= self._max_image_age_sec

        if self._require_image and not fresh:
            self._publish(
                Decision(rationale=f'no fresh camera frame (age {image_age:.2f}s)'),
                None, 0.0)
            return

        with self._instruction_lock:
            instruction = self._instruction
        with self._state_lock:
            vehicle = self._vehicle
            vehicle_stamp = self._vehicle_stamp_sec
        if vehicle is not None:
            vehicle = VehicleSnapshot(
                armed=vehicle.armed, mode=vehicle.mode,
                altitude_m=vehicle.altitude_m,
                age_sec=max(0.0, now - vehicle_stamp),
            )

        observation = Observation(
            image=self._convert(image_msg) if fresh else None,
            image_stamp_sec=image_stamp if fresh else None,
            instruction=instruction,
            vehicle=vehicle,
            now_sec=now,
        )

        inference_started = time.monotonic()
        decision = self._backend.infer(observation)
        inference_ms = (time.monotonic() - inference_started) * 1000.0

        self._publish(decision, image_stamp if fresh else None, inference_ms)

    def _convert(self, msg: Image):
        """Hand the backend the image in whatever form is available.

        cv_bridge is the normal path, but it is an optional dependency here: the Phase 4
        scripted policy ignores the image entirely, and requiring OpenCV to fly the
        integration pattern would be a needless install on the companion. Without it the
        raw ``sensor_msgs/Image`` is passed through -- the :class:`Observation` contract
        types ``image`` as ``Any`` precisely so this stays the backend's problem to declare,
        not this node's to guess.
        """
        try:
            from cv_bridge import CvBridge
        except ImportError:
            return msg
        if not hasattr(self, '_bridge'):
            self._bridge = CvBridge()
        try:
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'image conversion failed ({exc}); passing the raw message through',
                throttle_duration_sec=5.0)
            return msg

    # -- Publishing ------------------------------------------------------------------------

    def _publish(self, decision: Decision, source_stamp_sec: Optional[float],
                 inference_ms: float) -> None:
        # Sanitize unconditionally, including the scripted policy's own output. The backend
        # is untrusted input by construction -- in Phase 5 it is a neural network -- and the
        # guarantees downstream depends on have to hold whatever came out of it.
        decision = decision.sanitized(self._max_intent_duration_sec)

        now = self._now_sec()
        msg = NavIntent()
        msg.header.stamp = _to_stamp(now)
        msg.header.frame_id = 'base_link'
        with self._instruction_lock:
            msg.instruction = self._instruction
        msg.action_type = decision.action_type
        msg.velocity.linear.x = decision.vx
        msg.velocity.linear.y = decision.vy
        msg.velocity.linear.z = decision.vz
        msg.velocity.angular.z = decision.yaw_rate
        msg.duration = float(decision.duration_sec)
        msg.confidence = float(decision.confidence)
        msg.inference_latency_ms = float(inference_ms)
        # Zero when there was no frame -- an honest "this decision traces to no image"
        # rather than a fabricated timestamp the latency monitor would take at face value.
        msg.source_stamp = _to_stamp(source_stamp_sec) if source_stamp_sec else TimeMsg()
        msg.rationale = decision.rationale
        self._intent_pub.publish(msg)

        # Only report latency for decisions that trace back to a real frame. Reporting
        # frameless ones would fold meaningless cumulative numbers into the percentiles the
        # 300 ms budget is judged on.
        if source_stamp_sec:
            report = LatencyReport()
            report.header.stamp = msg.header.stamp
            report.stage = 'INFERENCE'
            report.source_stamp = msg.source_stamp
            report.stage_latency_ms = float(inference_ms)
            report.cumulative_latency_ms = float((now - source_stamp_sec) * 1000.0)
            report.sequence = self._sequence
            self._sequence += 1
            self._latency_pub.publish(report)

    # -- Teardown --------------------------------------------------------------------------

    def destroy_node(self) -> bool:
        self._running.clear()
        worker = getattr(self, '_worker', None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        try:
            self._backend.shutdown()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'backend shutdown raised: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VlaInferenceNode()
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
