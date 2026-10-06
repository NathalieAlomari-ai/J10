"""Object detection: one small interface, one real implementation (TFLite SSD).

Same split as ``obstacle_avoidance.py``: the part that turns raw model output into
``Detection`` objects (``parse_ssd_outputs``) is a pure function, testable against plain
numpy arrays, and the part that owns a model file and an interpreter (``TFLiteDetector``)
takes that interpreter as an injectable dependency so tests need neither.

Why TFLite SSD-MobileNet and not something newer
------------------------------------------------
A Pi Zero 2W is four Cortex-A53 cores and 512 MB of RAM with no usable GPU/NPU. A
quantized (uint8) SSD-MobileNet-v1 is ~4 MB on disk, runs on the CPU through XNNPACK, and
ends in the standard ``TFLite_Detection_PostProcess`` op, so box decoding and NMS happen
inside the model rather than in Python. Anything in the YOLO family at a useful input size
is several times the compute for accuracy this airframe can't use at 2 Hz anyway.
"""

from __future__ import annotations

import importlib
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Protocol, Sequence

import cv2
import numpy as np

from .config import CVNodeConfig

log = logging.getLogger("j10.cv_node.detection")

_PROJECT_ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class Detection:
    """One detected object. Box is normalized (0..1) image coordinates, origin top-left,
    so it means the same thing whatever resolution the frame was captured or inferred at."""

    label: str
    score: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "label": self.label,
            "score": round(self.score, 3),
            "box": [round(v, 4) for v in (self.x_min, self.y_min, self.x_max, self.y_max)],
        }


class Detector(Protocol):
    def detect(self, frame: "np.ndarray") -> list[Detection]:
        """``frame`` is BGR (HxWx3) or grayscale (HxW), any resolution."""
        ...

    def close(self) -> None: ...


def load_labels(path: Path) -> list[str]:
    """Reads a TFLite ``labelmap.txt`` (one class name per line, index = class id).

    The COCO labelmap shipped with the SSD-MobileNet models starts with a ``???``
    placeholder line that the model's class ids do *not* count — class 0 is ``person``,
    the second line — so that first line is dropped, same as TensorFlow's own examples do.
    Without a labelmap at all, only class 0 gets a name.
    """
    if not path.is_file():
        log.warning("labelmap %s not found; only class 0 ('person') will be named", path)
        return ["person"]
    labels = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()]
    if labels and labels[0] == "???":
        del labels[0]
    return labels


def _split_classes_and_scores(a: "np.ndarray", b: "np.ndarray") -> tuple["np.ndarray", "np.ndarray"]:
    """The post-process op emits two [1, N] tensors — class ids and scores — and which
    comes first depends on how the model was exported (TF1 exports: classes then scores;
    TF2 exports: the other way round). Class ids are whole numbers and scores are not, so
    tell them apart by that; fall back to the TF1 order when both look integral (e.g. no
    detections at all, where it doesn't matter)."""
    a_integral = bool(np.all(a == np.round(a)))
    b_integral = bool(np.all(b == np.round(b)))
    if b_integral and not a_integral:
        return b, a
    return a, b


def parse_ssd_outputs(
    outputs: Sequence["np.ndarray"],
    labels: Sequence[str],
    score_threshold: float,
    wanted_labels: frozenset[str] = frozenset(),
) -> list[Detection]:
    """Turns the four ``TFLite_Detection_PostProcess`` output tensors into ``Detection``s.

    Tensors are identified by shape, not position: boxes are [1, N, 4] as
    (ymin, xmin, ymax, xmax), classes and scores are [1, N], count is [1]. An empty
    ``wanted_labels`` keeps every class.
    """
    boxes = next((o for o in outputs if o.ndim == 3 and o.shape[-1] == 4), None)
    flat = [o for o in outputs if o.ndim == 2]
    count = next((o for o in outputs if o.ndim == 1), None)
    if boxes is None or len(flat) != 2:
        raise ValueError(
            f"not an SSD post-process output: tensor shapes {[tuple(o.shape) for o in outputs]}"
        )
    classes, scores = _split_classes_and_scores(flat[0], flat[1])

    n = boxes.shape[1] if count is None else min(int(count[0]), boxes.shape[1])
    detections: list[Detection] = []
    for i in range(n):
        score = float(scores[0][i])
        if score < score_threshold:
            continue
        class_id = int(classes[0][i])
        label = labels[class_id] if 0 <= class_id < len(labels) else f"class_{class_id}"
        if wanted_labels and label not in wanted_labels:
            continue
        y_min, x_min, y_max, x_max = (float(v) for v in np.clip(boxes[0][i], 0.0, 1.0))
        detections.append(Detection(label, score, x_min, y_min, x_max, y_max))
    return detections


def _load_interpreter_class():
    """Whichever TFLite runtime is installed, lightest first. ``ai-edge-litert`` is the
    current name of the standalone runtime, ``tflite-runtime`` its predecessor (the one
    with wheels for older Raspberry Pi OS Pythons); full TensorFlow is the dev-machine
    fallback and far too heavy to install on the Pi."""
    for module_name, attr_path in (
        ("ai_edge_litert.interpreter", "Interpreter"),
        ("tflite_runtime.interpreter", "Interpreter"),
        ("tensorflow", "lite.Interpreter"),
    ):
        try:
            obj: Any = importlib.import_module(module_name)
        except ImportError:
            continue
        for attr in attr_path.split("."):
            obj = getattr(obj, attr)
        return obj
    raise RuntimeError(
        "no TFLite runtime installed: `pip install ai-edge-litert` (or `tflite-runtime` "
        "on older Raspberry Pi OS) — see README 'Human detection'"
    )


class TFLiteDetector:
    def __init__(
        self,
        model_path: Path,
        labels: Sequence[str],
        score_threshold: float,
        wanted_labels: frozenset[str] = frozenset(),
        num_threads: int = 2,
        interpreter: Optional[Any] = None,
    ):
        if interpreter is None:
            if not model_path.is_file():
                raise FileNotFoundError(
                    f"detection model {model_path} not found — run `j10-fetch-model` first"
                )
            interpreter = _load_interpreter_class()(model_path=str(model_path), num_threads=num_threads)
        self._interpreter = interpreter
        self._labels = list(labels)
        self._score_threshold = score_threshold
        self._wanted_labels = wanted_labels

        self._interpreter.allocate_tensors()
        input_details = self._interpreter.get_input_details()[0]
        self._input_index = input_details["index"]
        _, self._input_h, self._input_w, _ = (int(v) for v in input_details["shape"])
        self._input_is_float = np.issubdtype(input_details["dtype"], np.floating)
        self._output_indices = [d["index"] for d in self._interpreter.get_output_details()]

    def detect(self, frame: "np.ndarray") -> list[Detection]:
        # Resize before the color conversion: the model input is smaller than any frame
        # worth capturing, so this is the cheaper order. Aspect ratio is not preserved —
        # boxes come back normalized, so the stretch cancels out on the way back.
        resized = cv2.resize(frame, (self._input_w, self._input_h), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_GRAY2RGB if resized.ndim == 2 else cv2.COLOR_BGR2RGB)
        if self._input_is_float:
            tensor = (rgb.astype(np.float32) - 127.5) / 127.5
        else:
            tensor = rgb
        self._interpreter.set_tensor(self._input_index, tensor[np.newaxis, ...])
        self._interpreter.invoke()
        outputs = [self._interpreter.get_tensor(i) for i in self._output_indices]
        return parse_ssd_outputs(outputs, self._labels, self._score_threshold, self._wanted_labels)

    def close(self) -> None:
        pass


def resolve_path(raw: str) -> Path:
    path = Path(raw).expanduser()
    if path.is_absolute() or path.exists():
        return path
    return _PROJECT_ROOT / path


def create_detector(config: CVNodeConfig) -> Detector:
    return TFLiteDetector(
        model_path=resolve_path(config.detect_model_path),
        labels=load_labels(resolve_path(config.detect_labels_path)),
        score_threshold=config.detect_score_threshold,
        wanted_labels=config.detect_wanted_labels,
        num_threads=config.detect_threads,
    )
