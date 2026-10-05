"""``latency_monitor_node`` -- turns per-stage LatencyReports into a verdict.

``docs/ARCHITECTURE.md`` on why this node exists:

    Aggregates per-stage LatencyReport into p50/p95/p99 and publishes
    diagnostic_msgs/DiagnosticArray. **This is how the 300 ms number gets proven rather
    than assumed.**

Every stage on the critical path emits a LatencyReport per item; this node collects them,
reduces each stage over a rolling window, and compares p95 against the budget table in
section 6. It publishes at 1 Hz and holds no authority over anything -- a monitor that
could change behaviour would stop being a measurement.

All the arithmetic lives in percentiles.py, which has no ROS in it and is unit-tested.
"""

from __future__ import annotations

import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from j10_interfaces.msg import LatencyReport

from .percentiles import StageAggregator, Summary, check_budget

#: Per-stage p95 targets, from the latency budget table in docs/ARCHITECTURE.md section 6.
#: Upper end of each documented range, since the budget is what must not be exceeded.
DEFAULT_BUDGETS_MS: Dict[str, float] = {
    'CAPTURE': 40.0,
    'TRANSPORT': 25.0,
    'DECODE': 20.0,
    'INFERENCE': 150.0,
    'CONTROL': 8.0,
    'SAFETY': 8.0,
    'BRIDGE': 20.0,
}

#: The number the whole budget exists to defend.
DEFAULT_END_TO_END_BUDGET_MS = 300.0

#: Reported as the end-to-end stage name in diagnostics.
END_TO_END = 'END_TO_END'


class LatencyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('latency_monitor_node')

        rate_hz = self.declare_parameter('publish_rate_hz', 1.0).value
        self._latency_topic = self.declare_parameter(
            'latency_topic', '/j10/telemetry/latency').value
        self._diagnostics_topic = self.declare_parameter(
            'diagnostics_topic', '/diagnostics').value
        window = int(self.declare_parameter('window_samples', 300).value)

        self._end_to_end_budget_ms = self.declare_parameter(
            'end_to_end_budget_ms', DEFAULT_END_TO_END_BUDGET_MS).value

        # Budgets come in as two parallel lists because ROS 2 parameters cannot express a
        # map of string to double. Empty means "use the documented defaults", which is what
        # anyone reading section 6 would expect the monitor to be checking.
        names = list(self.declare_parameter('budget_stage_names', []).value or [])
        values = list(self.declare_parameter('budget_stage_ms', []).value or [])
        if names and len(names) != len(values):
            raise ValueError(
                f'budget_stage_names has {len(names)} entries but budget_stage_ms has '
                f'{len(values)}; they must be the same length')
        self._budgets = dict(zip(names, (float(v) for v in values))) if names \
            else dict(DEFAULT_BUDGETS_MS)

        # Log a stage's first breach once rather than every second, but re-log if it
        # recovers and breaches again -- a monitor that spams is a monitor people mute.
        self._breaching: Dict[str, bool] = {}

        self._lock = threading.Lock()
        self._aggregator = StageAggregator(capacity=window)
        self._reports_seen = 0
        self._sequence_gaps = 0
        self._last_sequence: Dict[str, int] = {}

        # BEST_EFFORT with a deep queue: these are measurements, and losing one under load
        # is far better than adding back-pressure to the path being measured.
        self.create_subscription(
            LatencyReport, self._latency_topic, self._on_report,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=50))

        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._diagnostics_topic, QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, depth=1))

        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish)

        self.get_logger().info(
            f'latency_monitor_node up at {rate_hz:.1f} Hz, window={window} samples')
        self.get_logger().info(
            f'  budgets (p95, ms): {self._budgets}, end-to-end '
            f'{self._end_to_end_budget_ms:.0f}')

    # -- Intake -------------------------------------------------------------------------

    def _on_report(self, msg: LatencyReport) -> None:
        stage = msg.stage or 'UNKNOWN'
        with self._lock:
            self._reports_seen += 1

            # Sequence gaps catch a stage dropping work entirely -- a failure the timing
            # alone would hide, because the reports that *do* arrive can look perfectly
            # healthy while most frames never produced one.
            previous = self._last_sequence.get(stage)
            if previous is not None and msg.sequence > previous + 1:
                self._sequence_gaps += int(msg.sequence - previous - 1)
            self._last_sequence[stage] = int(msg.sequence)

            # cumulative_latency_ms is only meaningful when it traces to a real source
            # frame. j10_vla publishes 0.0 for frameless decisions, and folding those in
            # would report an end-to-end latency far better than anything real.
            cumulative = None
            if msg.cumulative_latency_ms > 0.0:
                cumulative = float(msg.cumulative_latency_ms)
            self._aggregator.add(stage, float(msg.stage_latency_ms), cumulative)

    # -- Output -------------------------------------------------------------------------

    def _publish(self) -> None:
        with self._lock:
            summaries = self._aggregator.summaries()
            end_to_end = self._aggregator.end_to_end()
            reports_seen = self._reports_seen
            sequence_gaps = self._sequence_gaps

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        for stage in sorted(summaries):
            array.status.append(
                self._status(stage, summaries[stage], self._budgets.get(stage)))

        if end_to_end is not None:
            array.status.append(
                self._status(END_TO_END, end_to_end, self._end_to_end_budget_ms))
        else:
            # Say so explicitly. A missing end-to-end row would read as "not checked yet",
            # which is indistinguishable from "checked and fine" at a glance.
            status = DiagnosticStatus()
            status.name = f'j10_telemetry: {END_TO_END}'
            status.hardware_id = 'j10'
            status.level = DiagnosticStatus.WARN
            status.message = 'no end-to-end samples yet (no stage reported a source frame)'
            array.status.append(status)

        overview = DiagnosticStatus()
        overview.name = 'j10_telemetry: pipeline'
        overview.hardware_id = 'j10'
        overview.level = DiagnosticStatus.OK if summaries else DiagnosticStatus.WARN
        overview.message = (f'{len(summaries)} stage(s) reporting'
                            if summaries else 'no latency reports received')
        overview.values = [
            KeyValue(key='reports_seen', value=str(reports_seen)),
            KeyValue(key='sequence_gaps', value=str(sequence_gaps)),
            KeyValue(key='stages', value=','.join(sorted(summaries)) or 'none'),
        ]
        array.status.insert(0, overview)

        self._diagnostics_pub.publish(array)

    def _status(self, stage: str, summary: Summary,
                budget_ms) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f'j10_telemetry: {stage}'
        status.hardware_id = 'j10'

        verdict = check_budget(stage, summary, budget_ms) if budget_ms else None

        if verdict is None:
            # No budget configured for this stage: report the numbers, claim nothing.
            status.level = DiagnosticStatus.OK
            status.message = f'p95 {summary.p95:.1f} ms (no budget configured)'
        elif verdict.over:
            status.level = DiagnosticStatus.WARN
            status.message = (f'p95 {verdict.p95_ms:.1f} ms exceeds '
                              f'{verdict.budget_ms:.0f} ms budget '
                              f'({verdict.utilization:.0%})')
        else:
            status.level = DiagnosticStatus.OK
            status.message = (f'p95 {verdict.p95_ms:.1f} ms of '
                              f'{verdict.budget_ms:.0f} ms ({verdict.utilization:.0%})')

        self._log_breach_edge(stage, verdict)

        for key, value in summary.as_dict().items():
            status.values.append(KeyValue(key=key, value=f'{value:.3f}'))
        if verdict is not None:
            status.values.append(
                KeyValue(key='budget_ms', value=f'{verdict.budget_ms:.1f}'))
            status.values.append(
                KeyValue(key='headroom_ms', value=f'{verdict.headroom_ms:.1f}'))
        if summary.rejected:
            # Already in as_dict, but worth a human-readable note: a stage dropping samples
            # can show a healthy p95 computed from the few that survived.
            status.values.append(
                KeyValue(key='note',
                         value=f'{summary.rejected} sample(s) rejected as invalid'))
        return status

    def _log_breach_edge(self, stage: str, verdict) -> None:
        """Log only on the transition into or out of a breach."""
        now_breaching = bool(verdict and verdict.over)
        was_breaching = self._breaching.get(stage, False)
        if now_breaching and not was_breaching:
            self.get_logger().warning(
                f'{stage} p95 {verdict.p95_ms:.1f} ms exceeds its '
                f'{verdict.budget_ms:.0f} ms budget')
        elif was_breaching and not now_breaching:
            self.get_logger().info(f'{stage} is back within budget')
        self._breaching[stage] = now_breaching


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LatencyMonitorNode()
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
