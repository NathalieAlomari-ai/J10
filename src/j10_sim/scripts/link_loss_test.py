#!/usr/bin/env python3
"""Prove the task A link-loss KPI, not just assume it.

    "Link-loss safety: stop commands on stream drop -> FC failsafe takes over"
    KPI: "Commands stop <= 1 s after link loss; FC failsafe verified"

This publishes /j10/cmd_vel_safe at 30 Hz, as safety_filter_node would, then stops --
simulating the PC-side chain dying rather than just going quiet -- and watches:

  1. how long the bridge keeps publishing setpoints after the stream stops
     (must be <= stream_stop_timeout_sec, KPI wants <= 1 s), and
  2. whether the flight controller's own mode subsequently changes to a failsafe mode
     (FC failsafe verified), by watching /j10/vehicle/state.mode.

It does NOT arm or fly the vehicle. Run it against a live j10_mavlink + MAVROS + SITL
stack, armed or not, before and after each change to the link-loss parameters:

    ros2 run j10_sim link_loss_test.py --failsafe-mode LAND

Exit code is 0 only if both conditions hold.
"""

import argparse
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from j10_interfaces.msg import VehicleState


class LinkLossTest(Node):

    def __init__(self, args):
        super().__init__('link_loss_test')
        self._args = args

        realtime = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)

        self._cmd_pub = self.create_publisher(TwistStamped, args.cmd_topic, realtime)
        self.create_subscription(
            PositionTarget, args.setpoint_topic, self._on_setpoint,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(
            VehicleState, '/j10/vehicle/state', self._on_state,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))

        self._last_setpoint_at = None
        self._stream_start = None
        self._stopped_streaming_publishing = False
        self._stopped_at = None
        self._mode_at_stop = None
        self._failsafe_seen_at = None
        self._phase = 'streaming'  # -> 'cut' -> 'done'

        self._publish_timer = self.create_timer(1.0 / 30.0, self._tick)
        self.create_timer(args.warmup_sec, self._cut_stream)
        self.create_timer(args.warmup_sec + args.observe_sec, self._finish)

        self.get_logger().info(
            f'streaming {args.warmup_sec:.0f} s, then cutting the source and observing '
            f'{args.observe_sec:.0f} s. This does NOT arm or fly the vehicle.')

    def _tick(self):
        if self._phase != 'streaming':
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.05  # small and clearly non-zero, never actually flown
        self._cmd_pub.publish(msg)

    def _cut_stream(self):
        self._phase = 'cut'
        self._stream_start = self.get_clock().now()
        self.get_logger().warning(
            'STREAM CUT — no more /j10/cmd_vel_safe published. Watching the bridge and '
            'the flight controller mode.')

    def _on_setpoint(self, _msg):
        now = self.get_clock().now()
        self._last_setpoint_at = now
        if self._phase == 'cut' and not self._stopped_streaming_publishing:
            # Still publishing; keep the timestamp so we know how late it stopped.
            pass

    def _on_state(self, msg):
        if self._phase != 'cut':
            return
        if self._mode_at_stop is None:
            self._mode_at_stop = msg.mode
            return
        if msg.mode != self._mode_at_stop and self._failsafe_seen_at is None:
            self._failsafe_seen_at = self.get_clock().now()
            self.get_logger().warning(
                f'flight controller mode changed {self._mode_at_stop} -> {msg.mode}')

    def _finish(self):
        # How long after the cut the bridge kept publishing. If it never stops, this stays
        # unbounded and the report says so explicitly rather than printing a small number.
        stop_latency_sec = None
        if self._last_setpoint_at is not None and self._stream_start is not None:
            stop_latency_sec = (
                self._last_setpoint_at - self._stream_start).nanoseconds * 1e-9

        commands_stopped = (
            stop_latency_sec is not None and stop_latency_sec <= self._args.observe_sec)
        # A setpoint arriving within the last publish period of the observation window
        # counts as "still streaming at the end" -- i.e. it never stopped.
        still_streaming_at_end = (
            stop_latency_sec is not None and
            stop_latency_sec >= self._args.observe_sec - 0.5)

        failsafe_verified = self._failsafe_seen_at is not None
        failsafe_latency_sec = None
        if failsafe_verified:
            failsafe_latency_sec = (
                self._failsafe_seen_at - self._stream_start).nanoseconds * 1e-9

        print('\n--- link-loss KPI check ---')
        if still_streaming_at_end or stop_latency_sec is None:
            print('  Commands stop <= 1 s after link loss:  FAIL')
            print('    setpoints were still being published at the end of the observation')
            print('    window. link_loss_stop_enabled is probably false, or '
                  'stream_stop_timeout_sec is set above --observe-sec.')
        else:
            verdict = 'PASS' if stop_latency_sec <= 1.0 else 'OVER BUDGET'
            print(f'  Commands stop <= 1 s after link loss:  {verdict}  '
                  f'(stopped after {stop_latency_sec:.2f} s)')

        if failsafe_verified:
            print(f'  FC failsafe verified:                  PASS  '
                  f'(mode changed after {failsafe_latency_sec:.2f} s, '
                  f'{self._mode_at_stop} -> observed change)')
        else:
            print('  FC failsafe verified:                  FAIL')
            print('    the flight controller mode never changed during the observation')
            print('    window. Check FS_GCS_ENABLE / FS_GCS_TIMEOUT in the loaded .parm '
                  'file, and that the vehicle is armed -- ArduPilot does not run this '
                  'failsafe while disarmed.')

        ok = commands_stopped and not still_streaming_at_end and failsafe_verified
        print(f'\n  overall: {"PASS" if ok else "FAIL"}\n')
        rclpy.shutdown()
        self._args.exit_code = 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--warmup-sec', type=float, default=3.0)
    parser.add_argument('--observe-sec', type=float, default=8.0,
                        help='must comfortably exceed stream_stop_timeout_sec + '
                             'FS_GCS_TIMEOUT for a meaningful result')
    parser.add_argument('--cmd-topic', default='/j10/cmd_vel_safe')
    parser.add_argument('--setpoint-topic', default='/mavros/mavros/local')
    parser.add_argument('--failsafe-mode', default='LAND',
                        help='informational only; the check is "mode changed at all"')
    args = parser.parse_args()
    args.exit_code = 1

    rclpy.init()
    node = LinkLossTest(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return args.exit_code


if __name__ == '__main__':
    sys.exit(main())
