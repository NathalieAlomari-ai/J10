"""Detection without a model: `parse_ssd_outputs` against hand-built tensors, and
`TFLiteDetector` against a fake interpreter that records what it was fed. No TFLite
runtime and no model file required — same hardware-free rule as the rest of this suite."""

import numpy as np
import pytest

from cv_node.detection import TFLiteDetector, load_labels, parse_ssd_outputs

LABELS = ["person", "bicycle", "car"]


def ssd_outputs(rows, tf2_order=False):
    """rows: (class_id, score, (ymin, xmin, ymax, xmax)). Padded to 10 slots like the
    real post-process op, which always emits a fixed-size tensor plus a count."""
    boxes = np.zeros((1, 10, 4), dtype=np.float32)
    classes = np.zeros((1, 10), dtype=np.float32)
    scores = np.zeros((1, 10), dtype=np.float32)
    for i, (class_id, score, box) in enumerate(rows):
        boxes[0, i], classes[0, i], scores[0, i] = box, class_id, score
    count = np.array([len(rows)], dtype=np.float32)
    return [scores, boxes, count, classes] if tf2_order else [boxes, classes, scores, count]


def test_keeps_detections_at_or_above_threshold():
    outputs = ssd_outputs([(0, 0.9, (0.1, 0.2, 0.8, 0.6)), (0, 0.3, (0, 0, 1, 1))])
    detections = parse_ssd_outputs(outputs, LABELS, score_threshold=0.5)
    assert len(detections) == 1
    d = detections[0]
    assert d.label == "person"
    assert d.score == pytest.approx(0.9)
    # Model order is (ymin, xmin, ymax, xmax); Detection exposes x/y by name.
    assert (d.x_min, d.y_min, d.x_max, d.y_max) == pytest.approx((0.2, 0.1, 0.6, 0.8))


def test_wanted_labels_drops_other_classes():
    outputs = ssd_outputs([(2, 0.95, (0, 0, 1, 1)), (0, 0.7, (0, 0, 1, 1))])
    detections = parse_ssd_outputs(outputs, LABELS, 0.5, wanted_labels=frozenset({"person"}))
    assert [d.label for d in detections] == ["person"]


def test_empty_wanted_labels_keeps_every_class():
    outputs = ssd_outputs([(2, 0.95, (0, 0, 1, 1)), (0, 0.7, (0, 0, 1, 1))])
    assert [d.label for d in parse_ssd_outputs(outputs, LABELS, 0.5)] == ["car", "person"]


def test_tf2_export_tensor_order_is_handled():
    rows = [(1, 0.82, (0.1, 0.1, 0.5, 0.5))]
    tf1 = parse_ssd_outputs(ssd_outputs(rows), LABELS, 0.5)
    tf2 = parse_ssd_outputs(ssd_outputs(rows, tf2_order=True), LABELS, 0.5)
    assert tf1 == tf2
    assert tf2[0].label == "bicycle"


def test_slots_beyond_count_are_ignored():
    outputs = ssd_outputs([(0, 0.9, (0, 0, 1, 1))])
    outputs[2][0, 5] = 0.99  # stale garbage past `count`, as the real op can leave behind
    assert len(parse_ssd_outputs(outputs, LABELS, 0.5)) == 1


def test_boxes_are_clipped_and_unknown_classes_named():
    outputs = ssd_outputs([(42, 0.9, (-0.1, -0.2, 1.3, 1.1))])
    d = parse_ssd_outputs(outputs, LABELS, 0.5)[0]
    assert d.label == "class_42"
    assert (d.x_min, d.y_min, d.x_max, d.y_max) == (0.0, 0.0, 1.0, 1.0)


def test_non_ssd_output_is_rejected():
    with pytest.raises(ValueError):
        parse_ssd_outputs([np.zeros((1, 1001), dtype=np.float32)], LABELS, 0.5)


def test_load_labels_drops_the_placeholder_first_line(tmp_path):
    path = tmp_path / "labelmap.txt"
    path.write_text("???\nperson\nbicycle\n???\ncar\n", encoding="utf-8")
    labels = load_labels(path)
    assert labels[0] == "person"
    assert labels[2] == "???"  # only the leading placeholder goes; later gaps keep ids aligned


def test_load_labels_without_a_file_still_names_person(tmp_path):
    assert load_labels(tmp_path / "missing.txt") == ["person"]


class FakeInterpreter:
    def __init__(self, outputs, dtype=np.uint8, size=300):
        self._outputs = outputs
        self._dtype = dtype
        self._size = size
        self.fed = None
        self.invocations = 0

    def allocate_tensors(self):
        pass

    def get_input_details(self):
        return [{"index": 7, "shape": np.array([1, self._size, self._size, 3]), "dtype": self._dtype}]

    def get_output_details(self):
        return [{"index": i} for i in range(len(self._outputs))]

    def set_tensor(self, index, value):
        assert index == 7
        self.fed = value

    def invoke(self):
        self.invocations += 1

    def get_tensor(self, index):
        return self._outputs[index]


def make_detector(interpreter):
    return TFLiteDetector(None, LABELS, 0.5, frozenset({"person"}), interpreter=interpreter)


def test_detector_resizes_and_converts_bgr_to_rgb():
    interpreter = FakeInterpreter(ssd_outputs([(0, 0.9, (0, 0, 1, 1))]))
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[..., 0] = 255  # pure blue in BGR

    detections = make_detector(interpreter).detect(frame)

    assert interpreter.invocations == 1
    assert interpreter.fed.shape == (1, 300, 300, 3)
    assert interpreter.fed.dtype == np.uint8
    assert tuple(interpreter.fed[0, 0, 0]) == (0, 0, 255)  # blue is last in RGB
    assert [d.label for d in detections] == ["person"]


def test_detector_accepts_grayscale_frames():
    interpreter = FakeInterpreter(ssd_outputs([]))
    make_detector(interpreter).detect(np.full((120, 160), 128, dtype=np.uint8))
    assert interpreter.fed.shape == (1, 300, 300, 3)


def test_float_input_models_get_normalized_input():
    interpreter = FakeInterpreter(ssd_outputs([]), dtype=np.float32, size=320)
    make_detector(interpreter).detect(np.full((240, 320, 3), 255, dtype=np.uint8))
    assert interpreter.fed.dtype == np.float32
    assert interpreter.fed.shape == (1, 320, 320, 3)
    assert interpreter.fed.max() == pytest.approx(1.0)
