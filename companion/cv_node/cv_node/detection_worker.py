"""DetectionWorker: runs the detector on its own thread, off the vision loop's clock.

Inference on a Pi Zero 2W takes several hundred milliseconds — longer than the whole
125 ms vision tick, and longer than ``mavlink_bridge``'s 250 ms staleness timeout. Run
inline, every inference would trip the bridge's failsafe hover. So the vision loop only
*hands over* its newest frame (``submit``, never blocks) and carries on; this thread
picks up whichever frame is newest when it's next free. Frames in between are dropped on
purpose — a detection of where a person was 600 ms ago is worth less than one of where
they are now.

Two things keep the Pi from cooking itself: a rate ceiling (``detect_max_rate_hz``) so the
thread idles between inferences instead of running back-to-back, and a thermal gate that
skips inference entirely while the SoC is too hot.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol

import numpy as np

from .config import CVNodeConfig
from .detection import Detection, Detector

log = logging.getLogger("j10.cv_node.detection")

_THERMAL_ZONE = "/sys/class/thermal/thermal_zone0/temp"


def read_cpu_temp_c(path: str = _THERMAL_ZONE) -> Optional[float]:
    """SoC temperature in Celsius, or ``None`` where there's no such sensor (a dev
    machine) — callers treat ``None`` as "can't tell, don't gate on it"."""
    try:
        with open(path, encoding="ascii") as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


@dataclass(frozen=True)
class DetectionResult:
    wall_time: float                      # time.time() when the result was produced
    detections: tuple[Detection, ...]
    inference_ms: float                   # 0.0 when thermal_paused
    cpu_temp_c: Optional[float]
    thermal_paused: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "time": round(self.wall_time, 3),
            "detections": [d.as_dict() for d in self.detections],
            "inference_ms": round(self.inference_ms, 1),
            "cpu_temp_c": self.cpu_temp_c,
            "thermal_paused": self.thermal_paused,
        }


class DetectionSink(Protocol):
    def publish(self, result: DetectionResult, frame: "np.ndarray") -> None: ...


class DetectionWorker:
    def __init__(
        self,
        detector: Detector,
        config: CVNodeConfig,
        sink: Optional[DetectionSink] = None,
        read_temp_c: Callable[[], Optional[float]] = read_cpu_temp_c,
    ):
        self.config = config
        self._detector = detector
        self._sink = sink
        self._read_temp_c = read_temp_c
        self._thermal_paused = False

        self._lock = threading.Lock()
        self._pending: Optional["np.ndarray"] = None
        self._latest: Optional[DetectionResult] = None
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="j10-detect", daemon=True)
        self._thread.start()

    def submit(self, frame: "np.ndarray") -> None:
        """Offer a frame. Never blocks; replaces any frame still waiting."""
        with self._lock:
            self._pending = frame
        self._wake.set()

    def latest(self) -> Optional[DetectionResult]:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._detector.close()

    def process(self, frame: "np.ndarray") -> DetectionResult:
        """One synchronous detection pass: thermal gate, inference, publish. The thread
        loop calls this; tests and the ``j10-detect`` CLI call it directly."""
        temp = self._read_temp_c()
        self._update_thermal_gate(temp)

        if self._thermal_paused:
            result = DetectionResult(time.time(), (), 0.0, temp, thermal_paused=True)
        else:
            started = time.perf_counter()
            detections = self._detector.detect(frame)
            inference_ms = (time.perf_counter() - started) * 1000.0
            result = DetectionResult(time.time(), tuple(detections), inference_ms, temp, thermal_paused=False)

        with self._lock:
            self._latest = result
        if self._sink is not None:
            try:
                self._sink.publish(result, frame)
            except Exception:
                log.exception("detection sink failed; result still available via latest()")
        return result

    def _update_thermal_gate(self, temp: Optional[float]) -> None:
        if temp is None:
            return
        if not self._thermal_paused and temp >= self.config.detect_temp_pause_c:
            self._thermal_paused = True
            log.warning("CPU at %.1f C; pausing detection until it cools to %.1f C",
                        temp, self.config.detect_temp_resume_c)
        elif self._thermal_paused and temp <= self.config.detect_temp_resume_c:
            self._thermal_paused = False
            log.info("CPU back down to %.1f C; resuming detection", temp)

    def _loop(self) -> None:
        min_period = 1.0 / self.config.detect_max_rate_hz
        while not self._stop.is_set():
            if not self._wake.wait(timeout=0.5):
                continue
            self._wake.clear()
            with self._lock:
                frame, self._pending = self._pending, None
            if frame is None:
                continue

            started = time.monotonic()
            try:
                self.process(frame)
            except Exception:
                log.exception("detection failed on this frame; continuing")
            remaining = min_period - (time.monotonic() - started)
            if remaining > 0:
                self._stop.wait(remaining)
