"""What happens to a detection once there is one: a log line when the picture changes,
a small JSON status file other processes can poll, and optional annotated snapshots.

Nothing here feeds back into flight. Detection is an inspection output, not a navigation
input — the velocity command still comes from ``obstacle_avoidance`` alone.
"""

from __future__ import annotations

import json
import logging
import os
import time
from collections import Counter
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np

from .config import CVNodeConfig
from .detection import Detection
from .detection_worker import DetectionResult

log = logging.getLogger("j10.cv_node.detection")


def annotate(frame: "np.ndarray", detections: Sequence[Detection]) -> "np.ndarray":
    """Returns a BGR copy of ``frame`` with each detection's box and label drawn on it."""
    out = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy()
    h, w = out.shape[:2]
    for d in detections:
        p1 = (int(d.x_min * w), int(d.y_min * h))
        p2 = (int(d.x_max * w), int(d.y_max * h))
        cv2.rectangle(out, p1, p2, (0, 255, 0), 2)
        cv2.putText(out, f"{d.label} {d.score:.2f}", (p1[0], max(p1[1] - 5, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
    return out


class DetectionPublisher:
    def __init__(self, config: CVNodeConfig):
        self._status_path: Optional[Path] = None
        if config.detect_status_path:
            path = Path(config.detect_status_path)
            if path.parent.is_dir():
                self._status_path = path
            else:
                log.warning("status file disabled: %s does not exist", path.parent)

        self._snapshot_dir: Optional[Path] = None
        if config.detect_snapshot_dir:
            self._snapshot_dir = Path(config.detect_snapshot_dir).expanduser()
            self._snapshot_dir.mkdir(parents=True, exist_ok=True)
        self._snapshot_min_interval_s = config.detect_snapshot_min_interval_s
        self._snapshot_max_files = config.detect_snapshot_max_files
        self._last_snapshot_at = float("-inf")

        self._last_summary: Counter = Counter()

    def publish(self, result: DetectionResult, frame: "np.ndarray") -> None:
        self._log_changes(result)
        if self._status_path is not None:
            self._write_status(result)
        if self._snapshot_dir is not None and result.detections:
            self._maybe_snapshot(result, frame)

    def _log_changes(self, result: DetectionResult) -> None:
        # One line when what's in view changes, not one per inference: at 2 Hz a person
        # standing in frame would otherwise be 120 identical lines a minute.
        summary = Counter(d.label for d in result.detections)
        if summary == self._last_summary:
            return
        self._last_summary = summary
        if not summary:
            log.info("nothing detected")
            return
        best = max(d.score for d in result.detections)
        counts = ", ".join(f"{n} {label}" for label, n in sorted(summary.items()))
        log.info("detected: %s (best score %.2f, %.0f ms)", counts, best, result.inference_ms)

    def _write_status(self, result: DetectionResult) -> None:
        # Write-then-rename so a reader never sees half a file.
        tmp = self._status_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(result.as_dict()), encoding="utf-8")
        os.replace(tmp, self._status_path)

    def _maybe_snapshot(self, result: DetectionResult, frame: "np.ndarray") -> None:
        now = time.monotonic()
        if now - self._last_snapshot_at < self._snapshot_min_interval_s:
            return
        self._last_snapshot_at = now

        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(result.wall_time))
        millis = int(result.wall_time * 1000) % 1000
        path = self._snapshot_dir / f"detect-{stamp}-{millis:03d}.jpg"
        cv2.imwrite(str(path), annotate(frame, result.detections))

        # Bounded on purpose: this is an SD card on a drone, not an archive.
        existing = sorted(self._snapshot_dir.glob("detect-*.jpg"))
        for old in existing[: max(len(existing) - self._snapshot_max_files, 0)]:
            old.unlink(missing_ok=True)
