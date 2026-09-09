"""j10_shm_protocol — the shared-memory wire contract between the CV node and the MAVLink
bridge, standalone so neither side depends on the other's package.

This is the whole point of the "microservice" split: the CV node and the bridge are two
independent OS processes (possibly restarted independently, possibly written in different
languages later) that must hand off a tiny, high-frequency payload — (vx, vy, vz, yaw_rate)
— with the lowest latency the Pi Zero 2W can give us. POSIX shared memory
(``/dev/shm``, via :mod:`multiprocessing.shared_memory`) gets us a memcpy-speed handoff with
no serialization, no socket syscalls, and no broker process. Living in its own package means
the contract itself — not one service's internals — is the thing both `mavlink_bridge` and
`cv_node` import; either can be rewritten (even in another language) without the other
noticing, as long as this wire format doesn't move.

Wire format (36 bytes, little-endian, fixed layout — see ``_STRUCT``)
-----------------------------------------------------------------------
    offset  size  field          notes
    0       4     magic          b"J10C" as uint32, sanity check only
    4       4     seq            seqlock counter — even = stable, odd = write in progress
    8       8     timestamp_ns   time.monotonic_ns() when the CV node computed this command
    16      4     vx             body-frame forward velocity, m/s
    20      4     vy             body-frame right velocity, m/s
    24      4     vz             body-frame down velocity, m/s (NED: positive = descending)
    28      4     yaw_rate       rad/s, positive = clockwise viewed from above
    32      1     valid          1 = CV node asserts this command is safe to act on
    33      3     (padding)

Concurrency: seqlock, not a mutex
----------------------------------
A named POSIX mutex/semaphore shared across independently-started processes needs an extra
C-extension dependency (``posix_ipc`` or similar) that has no reason to exist on a Pi Zero
2W for a 36-byte payload. Instead we use the same lock-free pattern the Linux kernel and
DDS shared-memory transports (Cyclone DDS, iceoryx — see docs/ARCHITECTURE.md §4) use for
exactly this shape of problem:

  * writer sets seq to odd *before* touching the payload, writes the payload, then sets
    seq to the next even number:
  * reader reads seq, reads the payload, re-reads seq; if seq is odd, or the two reads of
    seq disagree, the read raced a write and is discarded (retried, bounded).

This never blocks either side, which matters more than perfect fairness here: a CV node
that stalls on a lock is worse than a bridge that occasionally discards one torn read and
tries again next cycle. Caveat: CPython gives us an atomic-enough contiguous memcpy per
``memoryview`` slice assignment, but there is no explicit memory fence — on paper, a
weakly-ordered ARM core could reorder the seq write relative to the payload write. In
practice this is single-writer/single-reader on the same SoC at ≤50 Hz with a payload this
small, which is the same risk profile these lock-free transports accept in production; if
that ever needs to be bulletproof, swap ``CVCommandWriter``/``CVCommandReader`` for a
version backed by ``posix_ipc.Semaphore`` without touching any caller.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from multiprocessing import shared_memory
from typing import Optional

__all__ = ["CVCommand", "CVCommandReader", "CVCommandWriter", "SHM_SIZE"]
__version__ = "0.1.0"

_MAGIC = 0x4A313043  # b"J10C" read as a little-endian uint32
_STRUCT = struct.Struct("<IIQ4fB3x")  # magic, seq, timestamp_ns, vx,vy,vz,yaw_rate, valid
SHM_SIZE = _STRUCT.size  # 36 bytes
assert SHM_SIZE == 36

_SEQ_OFFSET = 4
_SEQ_STRUCT = struct.Struct("<I")


@dataclass(frozen=True)
class CVCommand:
    """One velocity/yaw-rate command as produced by the CV node."""

    timestamp_ns: int
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    valid: bool

    def age_s(self, now_ns: Optional[int] = None) -> float:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        return max(0.0, (now_ns - self.timestamp_ns) / 1e9)


class CVCommandWriter:
    """Used by the CV node. Kept in the same module as the reader because they're two
    halves of one contract and must never drift apart — this package's only job is that
    contract, not whichever service happens to call it."""

    def __init__(self, name: str = "j10_cv_cmd"):
        try:
            self._shm = shared_memory.SharedMemory(name=name, create=True, size=SHM_SIZE)
        except FileExistsError:
            # A previous, uncleanly-terminated process left the segment behind. Attach to
            # it rather than fail — this is a companion-computer service that gets
            # restarted by systemd, and /dev/shm survives a process crash.
            self._shm = shared_memory.SharedMemory(name=name, create=False)
        self._buf = self._shm.buf
        self._seq = 0
        _STRUCT.pack_into(self._buf, 0, _MAGIC, 0, 0, 0.0, 0.0, 0.0, 0.0, 0)

    def write(self, vx: float, vy: float, vz: float, yaw_rate: float, valid: bool = True) -> None:
        self._seq += 1  # odd: writer in progress
        _SEQ_STRUCT.pack_into(self._buf, _SEQ_OFFSET, self._seq)
        _STRUCT.pack_into(
            self._buf, 0,
            _MAGIC, self._seq, time.monotonic_ns(),
            float(vx), float(vy), float(vz), float(yaw_rate),
            1 if valid else 0,
        )
        self._seq += 1  # even: stable
        _SEQ_STRUCT.pack_into(self._buf, _SEQ_OFFSET, self._seq)

    def close(self) -> None:
        self._shm.close()

    def unlink(self) -> None:
        """Remove the segment from /dev/shm. Call on clean shutdown of the CV node."""
        self._shm.unlink()


class CVCommandReader:
    """Used by the bridge. Attaches lazily and tolerates the CV node not being up yet —
    that state is indistinguishable from "CV node crashed" and both must fail safe."""

    def __init__(self, name: str = "j10_cv_cmd", max_retries: int = 5):
        self._name = name
        self._max_retries = max_retries
        self._shm: Optional[shared_memory.SharedMemory] = None

    def _ensure_attached(self) -> bool:
        if self._shm is not None:
            return True
        try:
            self._shm = shared_memory.SharedMemory(name=self._name, create=False)
        except FileNotFoundError:
            return False
        if self._shm.size < SHM_SIZE:
            # Stale/incompatible segment from a different protocol version — never trust it.
            self._shm.close()
            self._shm = None
            return False
        return True

    def read(self) -> Optional[CVCommand]:
        """Returns the latest command, or ``None`` if the CV node isn't up, the segment is
        unreadable, or every retry raced a torn write. Callers must treat ``None`` exactly
        like a stale command — this is the failure mode the bridge's failsafe covers."""
        if not self._ensure_attached():
            return None
        buf = self._shm.buf
        for _ in range(self._max_retries):
            seq1 = _SEQ_STRUCT.unpack_from(buf, _SEQ_OFFSET)[0]
            if seq1 % 2 == 1:
                continue  # writer mid-update
            magic, _seq, ts_ns, vx, vy, vz, yaw_rate, valid = _STRUCT.unpack_from(buf, 0)
            seq2 = _SEQ_STRUCT.unpack_from(buf, _SEQ_OFFSET)[0]
            if seq1 != seq2:
                continue  # writer started a new update mid-read
            if magic != _MAGIC:
                return None
            return CVCommand(ts_ns, vx, vy, vz, yaw_rate, bool(valid))
        return None

    def close(self) -> None:
        if self._shm is not None:
            self._shm.close()
            self._shm = None
