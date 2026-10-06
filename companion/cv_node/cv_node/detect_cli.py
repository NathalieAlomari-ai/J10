"""``j10-detect``: run the detector by itself — no navigation, no shared memory, no bridge.

For answering two questions before detection goes anywhere near a flight: "does it see a
person?" (point it at an image, a video, or the camera) and "how fast and how hot does it
run on this board?" (the summary line at the end). Uses the same ``J10_CV_DETECT_*``
configuration as the real node, so what's measured here is what the node will do.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Iterator, Optional

import cv2
import numpy as np

from .camera import PiCamera2Source
from .config import CVNodeConfig
from .detection import create_detector
from .detection_output import annotate
from .detection_worker import DetectionWorker

_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def _frames(source: str, config: CVNodeConfig) -> Iterator["np.ndarray"]:
    if source == "picamera2":
        camera = PiCamera2Source(config.frame_width, config.frame_height)
        try:
            while True:
                yield camera.read()
        finally:
            camera.close()
    elif Path(source).suffix.lower() in _IMAGE_SUFFIXES:
        image = cv2.imread(source)
        if image is None:
            raise SystemExit(f"could not read image {source}")
        yield image
    else:
        # A video file path, or a bare number for a webcam index.
        capture = cv2.VideoCapture(int(source) if source.isdigit() else source)
        if not capture.isOpened():
            raise SystemExit(f"could not open {source}")
        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    return
                yield frame
        finally:
            capture.release()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("source", help="image file, video file, webcam index (e.g. 0), or 'picamera2'")
    parser.add_argument("--max-frames", type=int, default=50, help="default: %(default)s")
    parser.add_argument("--every", type=int, default=1, help="only run on every Nth frame")
    parser.add_argument("--save-dir", type=Path, help="write annotated frames here")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)-8s %(name)s: %(message)s")
    config = CVNodeConfig.from_env()
    worker = DetectionWorker(create_detector(config), config)
    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)

    timings: list[float] = []
    frames_with_hits = 0
    last_temp: Optional[float] = None
    for i, frame in enumerate(_frames(args.source, config)):
        if i % args.every:
            continue
        result = worker.process(frame)
        last_temp = result.cpu_temp_c
        if result.thermal_paused:
            print(f"frame {i}: skipped, CPU at {result.cpu_temp_c:.1f} C")
        else:
            timings.append(result.inference_ms)
            frames_with_hits += bool(result.detections)
            found = ", ".join(f"{d.label} {d.score:.2f}" for d in result.detections) or "-"
            print(f"frame {i}: {result.inference_ms:6.0f} ms  {found}")
            if args.save_dir:
                cv2.imwrite(str(args.save_dir / f"frame-{i:05d}.jpg"), annotate(frame, result.detections))
        if len(timings) >= args.max_frames:
            break

    if not timings:
        print("no frames processed")
        return 1
    # The first inference pays for one-off warm-up, so the median is the honest number.
    median = sorted(timings)[len(timings) // 2]
    temp = f", CPU {last_temp:.1f} C" if last_temp is not None else ""
    print(f"\n{len(timings)} frames, {frames_with_hits} with a detection, "
          f"median {median:.0f} ms ({1000.0 / median:.1f} fps max){temp}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
