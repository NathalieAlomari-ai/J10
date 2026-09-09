"""Frame sources. One tiny interface, three implementations — real Pi Camera Module 3,
a USB/dev webcam for bench-testing the vision logic on a laptop, and a synthetic source
for tests that needs neither a camera nor a camera library installed.
"""

from __future__ import annotations

from typing import Optional, Protocol, Sequence

import cv2
import numpy as np

from .config import CVNodeConfig


class FrameSource(Protocol):
    def read(self) -> Optional["np.ndarray"]:
        """Returns the next grayscale frame, or ``None`` on a capture failure. ``None`` is
        not an exception: the node treats it as "skip this tick" and moves on — a camera
        hiccup should not crash the process, and not writing is exactly what lets the
        bridge's own staleness failsafe take over if it keeps happening."""
        ...

    def close(self) -> None: ...


class PiCamera2Source:
    """The real thing: Raspberry Pi Camera Module 3 via ``picamera2``/``libcamera``.

    ``picamera2`` is imported lazily, inside ``__init__``, not at module load time — it's
    a system package on Raspberry Pi OS (``apt install python3-picamera2``, not pip; see
    ``../README.md``) and simply isn't importable on a dev machine or in CI. Keeping the
    import local means this module — and everything that imports it — stays importable
    everywhere else; only actually *constructing* this class requires the real hardware.
    """

    def __init__(self, width: int, height: int):
        from picamera2 import Picamera2  # noqa: PLC0415 — see class docstring

        self._picam2 = Picamera2()
        video_config = self._picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self._picam2.configure(video_config)
        self._picam2.start()

    def read(self) -> Optional["np.ndarray"]:
        frame = self._picam2.capture_array()  # RGB888, HxWx3
        return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    def close(self) -> None:
        self._picam2.stop()


class OpenCVCameraSource:
    """A USB or `/dev/videoN` webcam via ``cv2.VideoCapture``. Not what flies on the
    drone — this exists so the obstacle-avoidance logic can be exercised against a real,
    moving scene on a laptop before it ever touches the Pi Camera Module 3."""

    def __init__(self, index: int, width: int, height: int):
        self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise RuntimeError(f"could not open camera device index {index}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self) -> Optional["np.ndarray"]:
        ok, frame = self._cap.read()
        if not ok:
            return None
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def close(self) -> None:
        self._cap.release()


class SyntheticFrameSource:
    """Hands back a fixed, deterministic sequence of frames. Used by tests, and useful for
    dry-running the node's timing/logging against a known scene with no camera at all."""

    def __init__(self, frames: Sequence["np.ndarray"], loop: bool = True):
        self._frames = list(frames)
        self._loop = loop
        self._i = 0

    def read(self) -> Optional["np.ndarray"]:
        if not self._frames:
            return None
        if self._i >= len(self._frames):
            if not self._loop:
                return None
            self._i = 0
        frame = self._frames[self._i]
        self._i += 1
        return frame

    def close(self) -> None:
        pass


def create_frame_source(config: CVNodeConfig) -> FrameSource:
    if config.camera_backend == "picamera2":
        return PiCamera2Source(config.frame_width, config.frame_height)
    if config.camera_backend == "opencv":
        return OpenCVCameraSource(config.opencv_device_index, config.frame_width, config.frame_height)
    if config.camera_backend == "synthetic":
        raise ValueError(
            "camera_backend='synthetic' has no default frames to hand back — construct "
            "SyntheticFrameSource directly and pass it to CVNode(frame_source=...) "
            "(tests/dry-runs only, not selectable via config alone)"
        )
    raise ValueError(f"unknown camera_backend {config.camera_backend!r}")
