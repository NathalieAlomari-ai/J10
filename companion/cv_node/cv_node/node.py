"""CVNode: wires camera -> obstacle_avoidance -> smoothing -> j10_shm_protocol together.

Mirrors ``mavlink_bridge.bridge.MavlinkBridge`` on purpose: same dependency-injectable
constructor (a real default, override for tests), same fixed-rate loop with drift
correction, same "a tick that fails doesn't crash the process" posture, same
signal-driven ``stop()``.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from j10_shm_protocol import CVCommandWriter

from .camera import FrameSource, create_frame_source
from .config import CVNodeConfig
from .obstacle_avoidance import compute_zone_edge_densities, decide
from .smoothing import EmaSmoother

log = logging.getLogger("j10.cv_node")


class CVNode:
    def __init__(
        self,
        config: CVNodeConfig,
        frame_source: Optional[FrameSource] = None,
        writer: Optional[CVCommandWriter] = None,
    ):
        self.config = config
        self.frame_source = frame_source or create_frame_source(config)
        self.writer = writer or CVCommandWriter(name=config.shm_name)
        self._vx_smoother = EmaSmoother(config.ema_alpha)
        self._yaw_smoother = EmaSmoother(config.ema_alpha)
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def run(self, max_iterations: Optional[int] = None) -> None:
        """Blocks, running the vision loop until ``stop()`` is called. ``max_iterations``
        is for tests/dry-runs only — the real entrypoint never sets it, and relies on a
        signal handler calling ``stop()`` instead (see ``__main__.py``). When set, this
        also skips the inter-tick sleep so a test doesn't pay for real wall-clock time."""
        period = self.config.write_period_s
        next_tick = time.monotonic()
        overrun_warned_at = 0.0
        iterations = 0

        while not self._stop:
            self._tick()
            iterations += 1
            if max_iterations is not None and iterations >= max_iterations:
                break

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                now = time.monotonic()
                if now - overrun_warned_at > 1.0:
                    log.warning("vision loop overrun by %.1f ms", -sleep_for * 1000)
                    overrun_warned_at = now
                next_tick = now

    def _tick(self) -> None:
        frame = self.frame_source.read()
        if frame is None:
            log.warning("camera read failed; skipping this tick (bridge will fail safe on staleness)")
            return

        try:
            densities = compute_zone_edge_densities(frame, self.config)
            decision = decide(densities, self.config)
        except Exception:
            log.exception("vision pipeline error; skipping this tick")
            return

        vx = self._vx_smoother.update(decision.vx)
        yaw_rate = self._yaw_smoother.update(decision.yaw_rate)
        self.writer.write(vx=vx, vy=decision.vy, vz=decision.vz, yaw_rate=yaw_rate, valid=True)

        log.debug(
            "zone=%s blocked=%s densities=%s -> vx=%.2f yaw_rate=%.2f",
            decision.chosen_zone, decision.blocked, decision.densities.as_dict(),
            vx, yaw_rate,
        )

    def close(self) -> None:
        """Best-effort: mark the last command explicitly invalid before closing, so the
        bridge doesn't have to wait out the full staleness timeout to notice a clean
        shutdown. Mirrors MavlinkBridge._shutdown_settle()'s reasoning in the other
        direction — this is the writer side telling the reader "I'm done", not the
        reader assuming silence means failure."""
        try:
            self.writer.write(0.0, 0.0, 0.0, 0.0, valid=False)
        except Exception:
            log.exception("failed to write shutdown marker")
        self.frame_source.close()
        self.writer.close()
