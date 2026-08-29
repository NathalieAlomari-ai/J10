#!/usr/bin/env python3
"""Measure the command leg of the end-to-end latency budget.

Task A's KPI is "end-to-end latency (frame -> command at FC) <= 300 ms". The frame half of
that needs j10_video, which lands in Phase 3. This measures the half that exists today:

    /j10/cmd_vel_safe  ->  setpoint published on the MAVROS topic
    /j10/cmd_vel_safe  ->  the vehicle's measured velocity actually responds

The first number is the bridge's own contribution and should be a couple of milliseconds.
The second includes MAVROS, the MAVLink hop, ArduPilot's controller and the vehicle's own
inertia, so it is not latency in the same sense -- it is reaction time, and it is reported
separately for exactly that reason.

Run it against a flying vehicle, after arm and takeoff:

    ros2 run j10_sim latency_probe.py --duration 30
"""

import argparse
import statistics
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from j10_interfaces.msg import VehicleState


def percentile(values, p):
    """p50/p95/p99 without a numpy dependency."""
    if not values:
        return float('nan')
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round((p / 100.0) * (len(ordered) - 1))))
    return ordered[index]


class LatencyProbe(Node):

    def __init__(self, args):
        super().__init__('latency_probe')
        self._args = args

        realtime = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._cmd_pub = self.create_publisher(TwistStamped, args.cmd_topic, realtime)
        # The bridge publishes RELIABLE, because the MAVROS setpoint_raw subscription is
        # reliable; a BEST_EFFORT subscription here would be silently incompatible and see
        # nothing at all.
        self.create_subscription(
            PositionTarget, args.setpoint_topic, self._on_setpoint,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(
            VehicleState, '/j10/vehicle/state', self._on_state,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))

        self._bridge_ms = []
        self._response_ms = []
        self._sent_at = None
        self._pending_response = False
        self._commanded = 0.0
        self._setpoints_seen = 0

        self._period = 1.0 / args.rate
        self.create_timer(self._period, self._tick)
        self.create_timer(args.duration, self._finish)

        self.get_logger().info(
            f'probing for {args.duration:.0f} s at {args.rate:.0f} Hz, '
            f'{args.speed:.2f} m/s alternating on body x')

    # -- publish -------------------------------------------------------------------------

    def _tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # Alternate the sign so there is always an edge for the response measurement to
        # latch onto. A constant command would be indistinguishable from a stuck value.
        self._commanded = self._args.speed if self._commanded <= 0.0 else -self._args.speed
        msg.twist.linear.x = self._commanded
        self._sent_at = self.get_clock().now()
        self._pending_response = True
        self._cmd_pub.publish(msg)

    # -- measure -------------------------------------------------------------------------

    def _on_setpoint(self, msg):
        self._setpoints_seen += 1
        if self._sent_at is None:
            return
        # The bridge stamps the outgoing PositionTarget with its own publish time, so the
        # difference is the bridge's queue-to-publish contribution, not a round trip.
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        delta = (stamp - self._sent_at).nanoseconds * 1e-6
        if 0.0 <= delta < 1000.0:
            self._bridge_ms.append(delta)

    def _on_state(self, msg):
        if not self._pending_response or self._sent_at is None:
            return
        # First moment the measured body-x velocity moves decisively toward the command.
        measured = msg.velocity.twist.linear.x
        if abs(measured) < self._args.response_threshold:
            return
        if (measured > 0.0) != (self._commanded > 0.0):
            return
        delta = (self.get_clock().now() - self._sent_at).nanoseconds * 1e-6
        if 0.0 <= delta < 5000.0:
            self._response_ms.append(delta)
        self._pending_response = False

    # -- report --------------------------------------------------------------------------

    def _finish(self):
        def line(name, values, budget=None):
            if not values:
                print(f'  {name:<34} no samples')
                return
            p95 = percentile(values, 95)
            verdict = ''
            if budget is not None:
                verdict = '  PASS' if p95 <= budget else f'  OVER BUDGET ({budget:.0f} ms)'
            print(f'  {name:<34} n={len(values):<5} '
                  f'p50={percentile(values, 50):6.1f}  '
                  f'p95={p95:6.1f}  '
                  f'p99={percentile(values, 99):6.1f}  '
                  f'max={max(values):6.1f} ms{verdict}')

        print('\n--- latency probe ---')
        print(f'  setpoints observed on {self._args.setpoint_topic}: {self._setpoints_seen}')
        if self._setpoints_seen == 0:
            print('\n  NOTHING WAS PUBLISHED ON THE SETPOINT TOPIC.')
            print('  This is the silent failure described in src/j10_sim/README.md: the')
            print('  bridge publishes to a topic nothing subscribes to, the vehicle never')
            print(f'  moves, and no error is raised. Check `ros2 topic info '
                  f'{self._args.setpoint_topic}`.')
        line('cmd_vel_safe -> setpoint out', self._bridge_ms, budget=20.0)
        line('cmd_vel_safe -> velocity responds', self._response_ms)
        print('\n  The second row is reaction time, not latency: it includes MAVROS, the')
        print('  MAVLink hop, ArduPilot\'s controller and the airframe\'s own inertia.')
        print('  The frame -> command half of the 300 ms KPI needs j10_video (Phase 3).\n')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--rate', type=float, default=30.0)
    parser.add_argument('--speed', type=float, default=0.2,
                        help='m/s, alternating sign on body x')
    parser.add_argument('--response-threshold', type=float, default=0.05,
                        help='m/s at which the vehicle counts as having responded')
    parser.add_argument('--cmd-topic', default='/j10/cmd_vel_safe')
    parser.add_argument('--setpoint-topic', default='/mavros/mavros/local')
    args = parser.parse_args()

    rclpy.init()
    node = LatencyProbe(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
