"""Bench-test stand-in for the CV node.

The real CV node is a separate piece of software (not part of this package) that decides
Vx/Vy/Vz/yaw_rate from perception and calls :class:`CVCommandWriter.write` at whatever rate
it can sustain. This stub exists so the bridge can be exercised — including its failsafe
path — without that pipeline running.

Examples::

    # constant forward crawl at 0.2 m/s for 10s, then stop writing (simulates a CV crash
    # and should make the bridge log "failsafe hover engaged: CV command stale ...")
    python -m mavlink_bridge.cv_stub --vx 0.2 --duration 10

    # mark every command invalid, to exercise the "valid=False" failsafe path instead
    python -m mavlink_bridge.cv_stub --vx 0.2 --invalid
"""

from __future__ import annotations

import argparse
import logging
import signal
import time

from .shm_protocol import CVCommandWriter

log = logging.getLogger("j10.mavlink_bridge.cv_stub")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shm-name", default="j10_cv_cmd")
    parser.add_argument("--rate-hz", type=float, default=30.0, help="write rate, Hz")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--vz", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--invalid", action="store_true", help="write valid=False every cycle")
    parser.add_argument(
        "--duration", type=float, default=None,
        help="seconds to write for, then stop (leaves the segment stale so you can watch "
             "the bridge's failsafe engage). Default: run until Ctrl-C.",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-8s %(message)s")

    stop = False

    def _sigint(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sigint)

    writer = CVCommandWriter(name=args.shm_name)
    log.info(
        "writing vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f valid=%s @ %.1f Hz to shm '%s'",
        args.vx, args.vy, args.vz, args.yaw_rate, not args.invalid, args.rate_hz, args.shm_name,
    )
    period = 1.0 / args.rate_hz
    start = time.monotonic()
    try:
        while not stop:
            if args.duration is not None and (time.monotonic() - start) > args.duration:
                log.info("duration elapsed, stopped writing (segment will go stale)")
                break
            writer.write(args.vx, args.vy, args.vz, args.yaw_rate, valid=not args.invalid)
            time.sleep(period)
        if stop:
            log.info("Ctrl-C, stopped writing")
    finally:
        writer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
