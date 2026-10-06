"""DetectionWorker and DetectionPublisher with a fake detector and a fake thermometer —
the thermal gate, latest-frame-wins handoff, and output files, with no model involved."""

import json
import time

import numpy as np

from cv_node.config import CVNodeConfig
from cv_node.detection import Detection
from cv_node.detection_output import DetectionPublisher, annotate
from cv_node.detection_worker import DetectionWorker

PERSON = Detection("person", 0.9, 0.25, 0.25, 0.75, 0.75)


class FakeDetector:
    def __init__(self, detections=(PERSON,), fail=False):
        self.detections = list(detections)
        self.fail = fail
        self.frames = []
        self.closed = False

    def detect(self, frame):
        self.frames.append(frame)
        if self.fail:
            raise RuntimeError("boom")
        return list(self.detections)

    def close(self):
        self.closed = True


class RecordingSink:
    def __init__(self):
        self.results = []

    def publish(self, result, frame):
        self.results.append(result)


def frame(value=0):
    return np.full((24, 32, 3), value, dtype=np.uint8)


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def test_process_returns_and_publishes_the_detections():
    sink = RecordingSink()
    worker = DetectionWorker(FakeDetector(), CVNodeConfig(), sink=sink, read_temp_c=lambda: 50.0)
    result = worker.process(frame())
    assert result.detections == (PERSON,)
    assert result.thermal_paused is False
    assert result.cpu_temp_c == 50.0
    assert worker.latest() is result
    assert sink.results == [result]


def test_thermal_gate_pauses_then_resumes_with_hysteresis():
    temps = iter([60.0, 76.0, 72.0, 67.0])
    detector = FakeDetector()
    config = CVNodeConfig(detect_temp_pause_c=75.0, detect_temp_resume_c=68.0)
    worker = DetectionWorker(detector, config, read_temp_c=lambda: next(temps))

    paused = [worker.process(frame()).thermal_paused for _ in range(4)]

    # 72 C is below the pause point but above the resume point: still paused.
    assert paused == [False, True, True, False]
    assert len(detector.frames) == 2  # no inference at all while paused


def test_no_thermal_sensor_never_pauses():
    worker = DetectionWorker(FakeDetector(), CVNodeConfig(), read_temp_c=lambda: None)
    result = worker.process(frame())
    assert result.thermal_paused is False
    assert result.cpu_temp_c is None


def test_a_failing_sink_does_not_lose_the_result():
    class BrokenSink:
        def publish(self, result, frame):
            raise OSError("disk full")

    worker = DetectionWorker(FakeDetector(), CVNodeConfig(), sink=BrokenSink(), read_temp_c=lambda: None)
    result = worker.process(frame())
    assert worker.latest() is result


def test_thread_processes_submitted_frames_and_stops_cleanly():
    detector = FakeDetector()
    worker = DetectionWorker(detector, CVNodeConfig(detect_max_rate_hz=1000.0), read_temp_c=lambda: None)
    worker.start()
    worker.submit(frame(7))
    assert wait_for(lambda: worker.latest() is not None)
    worker.stop()
    assert detector.frames[0][0, 0, 0] == 7
    assert detector.closed is True


def test_thread_survives_a_detector_exception():
    detector = FakeDetector(fail=True)
    worker = DetectionWorker(detector, CVNodeConfig(detect_max_rate_hz=1000.0), read_temp_c=lambda: None)
    worker.start()
    worker.submit(frame())
    assert wait_for(lambda: len(detector.frames) == 1)
    detector.fail = False
    worker.submit(frame())
    assert wait_for(lambda: worker.latest() is not None)
    worker.stop()


def test_rate_ceiling_drops_intermediate_frames_and_keeps_the_newest():
    detector = FakeDetector()
    worker = DetectionWorker(detector, CVNodeConfig(detect_max_rate_hz=4.0), read_temp_c=lambda: None)
    worker.start()
    worker.submit(frame(1))
    assert wait_for(lambda: len(detector.frames) == 1)
    for value in (2, 3, 4):  # all arrive inside the 250 ms the worker is idling
        worker.submit(frame(value))
    assert wait_for(lambda: len(detector.frames) == 2)
    worker.stop()
    assert [f[0, 0, 0] for f in detector.frames] == [1, 4]


def make_result(worker_detections=(PERSON,)):
    worker = DetectionWorker(FakeDetector(worker_detections), CVNodeConfig(), read_temp_c=lambda: 55.5)
    return worker.process(frame())


def test_publisher_writes_status_json(tmp_path):
    status = tmp_path / "detections.json"
    publisher = DetectionPublisher(CVNodeConfig(detect_status_path=str(status)))
    publisher.publish(make_result(), frame())

    data = json.loads(status.read_text(encoding="utf-8"))
    assert data["detections"] == [{"label": "person", "score": 0.9, "box": [0.25, 0.25, 0.75, 0.75]}]
    assert data["cpu_temp_c"] == 55.5
    assert data["thermal_paused"] is False
    assert not status.with_suffix(".tmp").exists()


def test_publisher_tolerates_a_missing_status_directory(tmp_path):
    config = CVNodeConfig(detect_status_path=str(tmp_path / "nope" / "detections.json"))
    DetectionPublisher(config).publish(make_result(), frame())  # e.g. /dev/shm on Windows


def test_snapshots_are_rate_limited_and_only_taken_on_detections(tmp_path):
    config = CVNodeConfig(detect_status_path="", detect_snapshot_dir=str(tmp_path),
                          detect_snapshot_min_interval_s=60.0)
    publisher = DetectionPublisher(config)
    publisher.publish(make_result(()), frame())
    assert list(tmp_path.glob("*.jpg")) == []
    publisher.publish(make_result(), frame())
    publisher.publish(make_result(), frame())
    assert len(list(tmp_path.glob("*.jpg"))) == 1


def test_snapshot_directory_is_capped(tmp_path):
    for i in range(5):
        (tmp_path / f"detect-20000101-00000{i}-000.jpg").write_bytes(b"old")
    config = CVNodeConfig(detect_status_path="", detect_snapshot_dir=str(tmp_path),
                          detect_snapshot_min_interval_s=0.0, detect_snapshot_max_files=3)
    DetectionPublisher(config).publish(make_result(), frame())

    remaining = sorted(p.name for p in tmp_path.glob("*.jpg"))
    assert len(remaining) == 3
    assert remaining[0] == "detect-20000101-000003-000.jpg"  # the oldest three went


def test_publisher_logs_only_when_the_picture_changes(tmp_path, caplog):
    publisher = DetectionPublisher(CVNodeConfig(detect_status_path=""))
    with caplog.at_level("INFO", logger="j10.cv_node.detection"):
        for detections in ((PERSON,), (PERSON,), (PERSON,), (), ()):
            publisher.publish(make_result(detections), frame())
    messages = [r.getMessage() for r in caplog.records]
    assert len(messages) == 2
    assert messages[0].startswith("detected: 1 person")
    assert messages[1] == "nothing detected"


def test_annotate_draws_on_a_copy_and_accepts_grayscale():
    original = frame()
    drawn = annotate(original, [PERSON])
    assert not original.any()
    assert drawn.any()
    assert annotate(np.zeros((24, 32), dtype=np.uint8), [PERSON]).shape == (24, 32, 3)
