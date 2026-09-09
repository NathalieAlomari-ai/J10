"""j10.mavlink_bridge — standalone PC/FC-... no, Pi/FC communication bridge.

A lightweight microservice that runs on the Raspberry Pi Zero 2W companion computer for
J10 PT1. It owns the Serial/UART link to the CUAV V7 Nano flight controller and streams
MAVLink velocity setpoints derived from whatever the CV node last wrote to shared memory.

This package has exactly one job: physical drive commands. It knows nothing about vision,
object detection, or planning — that lives entirely on the other side of the shared-memory
adapter, in the CV node. The adapter itself is the standalone ``j10_shm_protocol`` package
(see ``../j10_shm_protocol/``), not part of this one — see ``README.md`` in this directory
for the wire protocol, the MAVLink references, and how the two processes are meant to be
run together.
"""

from j10_shm_protocol import CVCommand, CVCommandReader, CVCommandWriter

from .config import BridgeConfig

__all__ = ["BridgeConfig", "CVCommand", "CVCommandReader", "CVCommandWriter"]

__version__ = "0.1.0"
