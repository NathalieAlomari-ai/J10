"""Runtime configuration for the MAVLink bridge.

Every field can be overridden by an environment variable prefixed ``J10_BRIDGE_`` (see
``from_env``), which is how the systemd unit in ``systemd/j10-mavlink-bridge.service``
configures it. Kept as a plain dataclass — no YAML/JSON config library — because the only
consumer is a single process on a Pi Zero 2W and every extra dependency is something that
has to be cross-built or vendored for armv7/aarch64.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, fields
from typing import Any


def _bool(v: str) -> bool:
    return v.strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class BridgeConfig:
    # --- Serial link to the CUAV V7 Nano -----------------------------------------------
    # /dev/serial0 is the Pi's symlink to the primary UART (see README "Wiring" section
    # for the raspi-config / config.txt steps needed to free it from the login console).
    serial_port: str = "/dev/serial0"
    # 921600 is the commonly recommended companion-computer baud for ArduPilot telemetry
    # ports (matches SERIALx_BAUD=921 on the FC side). Verify against the CUAV V7 Nano
    # manual for the physical port used — see README references.
    baud: int = 921_600
    mavlink_source_system: int = 1
    # MAV_COMP_ID_ONBOARD_COMPUTER (191) — identifies this process as a companion computer
    # rather than the autopilot itself. https://mavlink.io/en/messages/common.html#MAV_COMPONENT
    mavlink_source_component: int = 191

    # --- Setpoint streaming --------------------------------------------------------------
    # ArduPilot's guided-mode setpoint watchdog trips after ~3s of silence (see README).
    # 20 Hz gives a wide safety margin against that while staying cheap on a Pi Zero 2W.
    setpoint_rate_hz: float = 20.0
    heartbeat_rate_hz: float = 1.0
    # MAV_FRAME_BODY_NED — body-relative velocity, consistent with the coordinate_frame
    # already chosen for the PC-side bridge in docs/ARCHITECTURE.md §4/§5. Confirm this
    # value (8) is still current for your ArduPilot version — see README references; some
    # ArduPilot releases prefer MAV_FRAME_BODY_OFFSET_NED (9) for velocity-only commands.
    coordinate_frame: int = 8  # MAV_FRAME_BODY_NED

    # --- Shared-memory adapter to the CV node --------------------------------------------
    shm_name: str = "j10_cv_cmd"
    # A command older than this is not trusted, even if the CV node is still technically
    # alive — this is what bounds "how stale can a velocity command be before we stop
    # trusting the world model behind it".
    cv_stale_timeout_s: float = 0.25
    # How many times to retry a shared-memory read that raced a torn write before giving up
    # for this cycle and treating the command as missing (fails safe, not blocking).
    shm_read_retries: int = 5

    # --- Failsafe ---------------------------------------------------------------------
    # No HEARTBEAT from the FC within this window means the serial link itself is suspect;
    # the bridge stops trusting target_system/target_component and tries to reconnect.
    fc_link_timeout_s: float = 3.0

    # --- Indoor PT1 velocity envelope (independent of whatever the CV node asks for) -----
    # Conservative bench/tethered-testing limits per docs/ARCHITECTURE.md Phase 6/7. Raise
    # only after each envelope has flown clean.
    max_horizontal_speed_mps: float = 0.6
    max_vertical_speed_mps: float = 0.4
    max_yaw_rate_rps: float = 0.6

    # --- Safety posture -----------------------------------------------------------------
    # This bridge NEVER arms the vehicle and, by default, never changes flight mode either
    # — matching the project's standing rule that a human on the transmitter (the ELRS
    # link in the BOM) holds arming/mode authority. See README "Safety posture".
    auto_request_guided_mode: bool = False

    log_level: str = "INFO"

    @classmethod
    def from_env(cls, prefix: str = "J10_BRIDGE_") -> "BridgeConfig":
        kwargs: dict[str, Any] = {}
        for f in fields(cls):
            env_name = prefix + f.name.upper()
            raw = os.environ.get(env_name)
            if raw is None:
                continue
            if f.type in (int, "int"):
                kwargs[f.name] = int(raw)
            elif f.type in (float, "float"):
                kwargs[f.name] = float(raw)
            elif f.type in (bool, "bool"):
                kwargs[f.name] = _bool(raw)
            else:
                kwargs[f.name] = raw
        return cls(**kwargs)

    @property
    def setpoint_period_s(self) -> float:
        return 1.0 / self.setpoint_rate_hz

    @property
    def heartbeat_period_s(self) -> float:
        return 1.0 / self.heartbeat_rate_hz
