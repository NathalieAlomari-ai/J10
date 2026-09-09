"""Entrypoint: ``python -m mavlink_bridge``.

Reads configuration from the ``J10_BRIDGE_*`` environment variables (see
``config.BridgeConfig.from_env``), connects to the flight controller, and runs the
setpoint-streaming loop until SIGINT/SIGTERM. Intended to be launched by
``systemd/j10-mavlink-bridge.service``, but runs the same way from a plain shell for bench
testing.
"""

from __future__ import annotations

import logging
import signal
import sys

from .bridge import MavlinkBridge
from .config import BridgeConfig


def main() -> int:
    config = BridgeConfig.from_env()
    logging.basicConfig(
        level=getattr(logging, config.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    )
    log = logging.getLogger("j10.mavlink_bridge")
    log.info("J10 MAVLink bridge starting: %s", config)

    bridge = MavlinkBridge(config)

    def _handle_signal(signum, _frame):
        log.info("received signal %d, shutting down", signum)
        bridge.stop()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        bridge.connect()
    except TimeoutError as exc:
        log.error("%s", exc)
        return 1

    bridge.run()
    log.info("bridge stopped cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
