"""The MAVLink bridge: Serial UART <-> CUAV V7 Nano, fed by the shared-memory adapter.

Responsibilities, and nothing else (see module docstring in ``__init__.py``):

  1. Open and hold a pymavlink Serial connection to the flight controller, send a companion
     -computer HEARTBEAT, and track the FC's own HEARTBEAT to know the link is alive.
  2. On every setpoint tick, read the latest CV command from shared memory.
  3. Translate it into a ``SET_POSITION_TARGET_LOCAL_NED`` velocity+yaw-rate message and
     send it at a fixed rate.
  4. If the CV command is missing, stale, or marked invalid — or the FC link itself has
     gone quiet — send zero velocity (hover) instead. This is the failsafe path and it is
     the default: everything else has to actively earn a non-zero setpoint.

References for every MAVLink message/enum/bitmask used below are in ``README.md``.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil

from .config import BridgeConfig
from .shm_protocol import CVCommandReader

log = logging.getLogger("j10.mavlink_bridge")

# POSITION_TARGET_TYPEMASK bits (MAVLink common.xml POSITION_TARGET_TYPEMASK enum), each a
# single "ignore this field" flag in this fixed order (bit -> field):
#   1 X_IGNORE, 2 Y_IGNORE, 4 Z_IGNORE, 8 VX_IGNORE, 16 VY_IGNORE, 32 VZ_IGNORE,
#   64 AX_IGNORE, 128 AY_IGNORE, 256 AZ_IGNORE, 512 FORCE_SET, 1024 YAW_IGNORE,
#   2048 YAW_RATE_IGNORE
# We want velocity + yaw rate control: ignore position, acceleration, and absolute yaw;
# use vx/vy/vz and yaw_rate. That's every "ignore" bit EXCEPT VX/VY/VZ_IGNORE and
# YAW_RATE_IGNORE:
#   X|Y|Z|AX|AY|AZ|YAW = 1+2+4+64+128+256+1024 = 1479 = 0x5C7 = 0b0000010111000111
# This matches the "Velocity + yaw rate" row of the ArduPilot wiki's Guided-mode
# SET_POSITION_TARGET_LOCAL_NED bitmask table — verify it against the current page before
# first flight (see README references; this constant is worth re-deriving from the enum
# above rather than trusting a copied bit string, which is exactly how it's written here).
_POSITION_TARGET_TYPEMASK_VELOCITY_AND_YAW_RATE = 0b0000_0101_1100_0111  # 1479 / 0x5C7


def _clamp(value: float, limit: float) -> float:
    if limit <= 0:
        return 0.0
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class Setpoint:
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    failsafe: bool
    reason: str = ""


class MavlinkBridge:
    def __init__(self, config: BridgeConfig, cv_reader: Optional[CVCommandReader] = None):
        self.config = config
        self.cv_reader = cv_reader or CVCommandReader(
            name=config.shm_name, max_retries=config.shm_read_retries
        )
        self._master: Optional[mavutil.mavfile] = None
        self._t0 = time.monotonic()
        self._last_fc_heartbeat_mono = 0.0
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._link_lock = threading.Lock()

    # -- connection lifecycle -----------------------------------------------------------

    def connect(self, heartbeat_timeout_s: float = 30.0) -> None:
        """Open the serial link and block until the FC's own HEARTBEAT is seen (this is
        also how pymavlink learns ``target_system``/``target_component`` — see README)."""
        log.info(
            "opening serial link to FC on %s @ %d baud",
            self.config.serial_port, self.config.baud,
        )
        self._master = mavutil.mavlink_connection(
            self.config.serial_port,
            baud=self.config.baud,
            source_system=self.config.mavlink_source_system,
            source_component=self.config.mavlink_source_component,
        )
        msg = self._master.wait_heartbeat(timeout=heartbeat_timeout_s)
        if msg is None:
            raise TimeoutError(
                f"no HEARTBEAT from flight controller within {heartbeat_timeout_s}s "
                f"on {self.config.serial_port} — check wiring/baud (see README)"
            )
        self._last_fc_heartbeat_mono = time.monotonic()
        log.info(
            "FC HEARTBEAT received: system=%d component=%d type=%d autopilot=%d",
            self._master.target_system, self._master.target_component,
            msg.type, msg.autopilot,
        )
        if self.config.auto_request_guided_mode:
            self._request_guided_mode()

    def _request_guided_mode(self) -> None:
        # Off by default (see config.auto_request_guided_mode docstring): mode changes are
        # left to the human on the ELRS transmitter unless explicitly opted in.
        assert self._master is not None
        log.warning("auto_request_guided_mode=True — requesting GUIDED mode")
        mode_id = self._master.mode_mapping().get("GUIDED")
        if mode_id is None:
            log.error("flight stack does not expose a GUIDED mode mapping; skipping")
            return
        self._master.mav.set_mode_send(
            self._master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    # -- run loop -------------------------------------------------------------------------

    def run(self) -> None:
        """Blocks, running the heartbeat, receive, and setpoint-streaming loops until
        ``stop()`` is called (e.g. from a SIGINT/SIGTERM handler in ``__main__``)."""
        assert self._master is not None, "call connect() first"
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._heartbeat_loop, name="hb-tx", daemon=True),
            threading.Thread(target=self._rx_loop, name="rx", daemon=True),
        ]
        for t in self._threads:
            t.start()
        try:
            self._setpoint_loop()
        finally:
            self._shutdown_settle()

    def stop(self) -> None:
        self._stop.set()

    def _shutdown_settle(self) -> None:
        """Best-effort: send a short burst of zero-velocity setpoints before exiting, so a
        service restart (or a crash we're recovering from) doesn't leave the FC's last
        setpoint as the last thing it heard before the stream goes silent for 3s."""
        if self._master is None:
            return
        log.info("sending hover setpoints before shutdown")
        for _ in range(5):
            try:
                self._send_setpoint(Setpoint(0.0, 0.0, 0.0, 0.0, failsafe=True, reason="shutdown"))
            except Exception:  # noqa: BLE001 — best-effort, we're already exiting
                break
            time.sleep(0.05)

    # -- heartbeat / rx -------------------------------------------------------------------

    def _heartbeat_loop(self) -> None:
        assert self._master is not None
        period = self.config.heartbeat_period_s
        while not self._stop.is_set():
            try:
                self._master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
            except Exception:
                log.exception("failed to send companion heartbeat")
            self._stop.wait(period)

    def _rx_loop(self) -> None:
        assert self._master is not None
        while not self._stop.is_set():
            try:
                msg = self._master.recv_match(blocking=True, timeout=0.5)
            except Exception:
                log.exception("serial read error")
                self._stop.wait(0.5)
                continue
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "HEARTBEAT" and msg.get_srcSystem() == self._master.target_system:
                with self._link_lock:
                    self._last_fc_heartbeat_mono = time.monotonic()
            elif mtype == "STATUSTEXT":
                log.info("FC STATUSTEXT: %s", msg.text)
            elif mtype == "BAD_DATA":
                log.debug("bad MAVLink data on link (parity/baud mismatch?): %r", msg)

    def fc_link_alive(self) -> bool:
        with self._link_lock:
            last = self._last_fc_heartbeat_mono
        return (time.monotonic() - last) < self.config.fc_link_timeout_s

    # -- setpoint computation / streaming --------------------------------------------------

    def _compute_setpoint(self) -> Setpoint:
        cfg = self.config
        cmd = self.cv_reader.read()

        if cmd is None:
            return Setpoint(0.0, 0.0, 0.0, 0.0, failsafe=True, reason="no CV command (node down?)")
        if not cmd.valid:
            return Setpoint(0.0, 0.0, 0.0, 0.0, failsafe=True, reason="CV command marked invalid")
        age = cmd.age_s()
        if age > cfg.cv_stale_timeout_s:
            return Setpoint(
                0.0, 0.0, 0.0, 0.0, failsafe=True,
                reason=f"CV command stale ({age * 1000:.0f} ms > {cfg.cv_stale_timeout_s * 1000:.0f} ms)",
            )
        if not self.fc_link_alive():
            return Setpoint(0.0, 0.0, 0.0, 0.0, failsafe=True, reason="FC link timeout")

        return Setpoint(
            vx=_clamp(cmd.vx, cfg.max_horizontal_speed_mps),
            vy=_clamp(cmd.vy, cfg.max_horizontal_speed_mps),
            vz=_clamp(cmd.vz, cfg.max_vertical_speed_mps),
            yaw_rate=_clamp(cmd.yaw_rate, cfg.max_yaw_rate_rps),
            failsafe=False,
        )

    def _send_setpoint(self, sp: Setpoint) -> None:
        assert self._master is not None
        time_boot_ms = int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF
        self._master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self._master.target_system,
            self._master.target_component,
            self.config.coordinate_frame,
            _POSITION_TARGET_TYPEMASK_VELOCITY_AND_YAW_RATE,
            0.0, 0.0, 0.0,          # x, y, z — ignored by type_mask
            sp.vx, sp.vy, sp.vz,
            0.0, 0.0, 0.0,          # afx, afy, afz — ignored by type_mask
            0.0,                    # yaw — ignored by type_mask
            sp.yaw_rate,
        )

    def _setpoint_loop(self) -> None:
        period = self.config.setpoint_period_s
        last_failsafe_reason = None
        next_tick = time.monotonic()
        overrun_warned_at = 0.0
        while not self._stop.is_set():
            sp = self._compute_setpoint()
            try:
                self._send_setpoint(sp)
            except Exception:
                log.exception("failed to send setpoint — attempting reconnect")
                self._reconnect()

            if sp.failsafe and sp.reason != last_failsafe_reason:
                log.warning("failsafe hover engaged: %s", sp.reason)
            elif not sp.failsafe and last_failsafe_reason is not None:
                log.info("failsafe cleared, resuming CV-commanded velocity")
            last_failsafe_reason = sp.reason if sp.failsafe else None

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                self._stop.wait(sleep_for)
            else:
                # We're falling behind the setpoint rate — resync instead of spiraling, and
                # log it (rate-limited) since sustained overrun means the loop is too slow
                # for this Pi, which is exactly the kind of thing to catch on the bench.
                now = time.monotonic()
                if now - overrun_warned_at > 1.0:
                    log.warning("setpoint loop overrun by %.1f ms", -sleep_for * 1000)
                    overrun_warned_at = now
                next_tick = now

    def _reconnect(self) -> None:
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
        backoff = 0.5
        while not self._stop.is_set():
            try:
                self.connect()
                log.info("reconnected to flight controller")
                return
            except Exception:
                log.error("reconnect failed, retrying in %.1fs", backoff)
                self._stop.wait(backoff)
                backoff = min(backoff * 2, 5.0)
