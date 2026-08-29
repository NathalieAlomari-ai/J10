"""``dataset_recorder_node`` -- synchronized rosbag2 capture for fine-tuning and review.

``docs/ARCHITECTURE.md``:

    Synchronized rosbag2 capture of image + intent + state + safety verdict, for VLA
    fine-tuning and post-flight review.

Recording is deliberately *not* automatic. A recorder that starts with the node would fill
the companion's disk across a day of bench runs, and the resulting bags would be mostly
idle ground time. It is armed by service call, and it stops on request or when the mission
leaves an autonomous state.

The node subscribes with BEST_EFFORT on the image topic for the same reason j10_vla does:
recording must never apply back-pressure to the path being recorded. A dropped frame in the
dataset is a small loss; a stalled control loop is not.
"""

from __future__ import annotations

import os
from datetime import datetime
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message

from std_srvs.srv import SetBool
from std_msgs.msg import String

from sensor_msgs.msg import Image
from j10_interfaces.msg import NavIntent, SafetyStatus, VehicleState

try:
    import rosbag2_py
    HAVE_ROSBAG2 = True
except ImportError:  # pragma: no cover - depends on the install
    HAVE_ROSBAG2 = False


class DatasetRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('dataset_recorder_node')

        self._output_dir = os.path.expanduser(
            self.declare_parameter('output_dir', '~/j10_datasets').value)
        self._image_topic = self.declare_parameter(
            'image_topic', '/j10/camera/image_raw').value
        self._intent_topic = self.declare_parameter(
            'intent_topic', '/j10/vla/intent').value
        self._state_topic = self.declare_parameter(
            'vehicle_state_topic', '/j10/vehicle/state').value
        self._safety_topic = self.declare_parameter(
            'safety_status_topic', '/j10/safety/status').value
        self._mission_state_topic = self.declare_parameter(
            'mission_state_topic', '/j10/mission/state').value

        # Stop when the mission leaves autonomy. The interesting data is the model flying;
        # everything after is the operator getting the vehicle down.
        self._stop_on_mission_exit = self.declare_parameter(
            'stop_on_mission_exit', True).value
        self._autonomous_state = self.declare_parameter(
            'autonomous_state', 'VLA_ACTIVE').value

        self._writer = None
        self._bag_path: Optional[str] = None
        self._counts = {'image': 0, 'intent': 0, 'state': 0, 'safety': 0}

        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(
            Image, self._image_topic,
            lambda m: self._write(self._image_topic, m, 'image'), best_effort)
        self.create_subscription(
            NavIntent, self._intent_topic,
            lambda m: self._write(self._intent_topic, m, 'intent'), reliable)
        self.create_subscription(
            VehicleState, self._state_topic,
            lambda m: self._write(self._state_topic, m, 'state'), reliable)
        self.create_subscription(
            SafetyStatus, self._safety_topic,
            lambda m: self._write(self._safety_topic, m, 'safety'), reliable)
        self.create_subscription(
            String, self._mission_state_topic, self._on_mission_state, reliable)

        self.create_service(SetBool, '/j10/telemetry/record', self._on_record)

        if not HAVE_ROSBAG2:
            self.get_logger().error(
                'rosbag2_py is not importable -- recording is unavailable. '
                'Install ros-humble-rosbag2-py.')
        self.get_logger().info(
            f'dataset_recorder_node up (idle). Call /j10/telemetry/record to start. '
            f'Bags go to {self._output_dir}')

    # -- Recording control ------------------------------------------------------------

    def _on_record(self, request, response):
        if request.data:
            response.success, response.message = self._start()
        else:
            response.success, response.message = self._stop()
        return response

    def _start(self):
        if self._writer is not None:
            return True, f'already recording to {self._bag_path}'
        if not HAVE_ROSBAG2:
            return False, 'rosbag2_py is not available in this install'

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._bag_path = os.path.join(self._output_dir, f'j10_{stamp}')
        try:
            os.makedirs(self._output_dir, exist_ok=True)
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=self._bag_path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
            for topic, type_name in (
                (self._image_topic, 'sensor_msgs/msg/Image'),
                (self._intent_topic, 'j10_interfaces/msg/NavIntent'),
                (self._state_topic, 'j10_interfaces/msg/VehicleState'),
                (self._safety_topic, 'j10_interfaces/msg/SafetyStatus'),
            ):
                writer.create_topic(rosbag2_py.TopicMetadata(
                    name=topic, type=type_name, serialization_format='cdr'))
        except Exception as exc:  # noqa: BLE001
            self._writer = None
            self.get_logger().error(f'could not open bag: {exc}')
            return False, f'could not open bag: {exc}'

        self._writer = writer
        self._counts = {k: 0 for k in self._counts}
        self.get_logger().info(f'recording to {self._bag_path}')
        return True, f'recording to {self._bag_path}'

    def _stop(self):
        if self._writer is None:
            return True, 'not recording'
        path = self._bag_path
        counts = dict(self._counts)
        # Drop the reference before anything else can raise: a writer left half-closed
        # would keep accepting messages into a bag nobody is going to read.
        self._writer = None
        self._bag_path = None
        self.get_logger().info(f'stopped recording {path}: {counts}')
        return True, f'stopped: {counts}'

    def _on_mission_state(self, msg: String) -> None:
        if (self._stop_on_mission_exit and self._writer is not None and
                msg.data != self._autonomous_state):
            self.get_logger().info(
                f'mission left {self._autonomous_state} (now {msg.data}); stopping capture')
            self._stop()

    # -- Writing ------------------------------------------------------------------------

    def _write(self, topic: str, message, kind: str) -> None:
        writer = self._writer
        if writer is None:
            return
        try:
            writer.write(topic, serialize_message(message),
                         self.get_clock().now().nanoseconds)
            self._counts[kind] += 1
        except Exception as exc:  # noqa: BLE001
            # A failed write (disk full is the realistic one) stops the recording rather
            # than logging once per frame for the rest of the flight.
            self.get_logger().error(f'bag write failed, stopping recording: {exc}')
            self._stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DatasetRecorderNode()
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
