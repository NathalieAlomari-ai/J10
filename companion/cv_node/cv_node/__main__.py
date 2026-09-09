"""Entrypoint: the ``j10-cv-node`` console script (equivalently ``python -m cv_node``, run
from a directory other than ``companion/`` — see ``../README.md`` "Running it" for why).

Reads configuration from ``J10_CV_*`` environment variables, opens the configured camera
backend, and runs the vision loop until SIGINT/SIGTERM.
"""

from __future__ import annotations

import logging
import signal
import sys

from .config import CVNodeConfig
from .node import CVNode


def main() -> int:
    config = CVNodeConfig.from_env()
    logging.basicConfig(
        level=getattr(logging, config.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    )
    log = logging.getLogger("j10.cv_node")
    log.info("J10 CV node starting: %s", config)

    try:
        node = CVNode(config)
    except Exception:
        log.exception("failed to initialize CV node (camera backend %r)", config.camera_backend)
        return 1

    def _handle_signal(signum, _frame):
        log.info("received signal %d, shutting down", signum)
        node.stop()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        node.run()
    finally:
        node.close()
    log.info("CV node stopped cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
