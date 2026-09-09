import time
import uuid

import pytest

from j10_shm_protocol import CVCommandReader, CVCommandWriter


@pytest.fixture
def shm_name():
    # unique per test so parallel test runs don't collide in /dev/shm
    return f"j10_test_{uuid.uuid4().hex[:12]}"


def test_reader_returns_none_before_writer_exists(shm_name):
    reader = CVCommandReader(name=shm_name)
    assert reader.read() is None
    reader.close()


def test_round_trip_write_then_read(shm_name):
    writer = CVCommandWriter(name=shm_name)
    reader = CVCommandReader(name=shm_name)
    try:
        writer.write(vx=0.3, vy=-0.1, vz=0.05, yaw_rate=0.2, valid=True)
        cmd = reader.read()
        assert cmd is not None
        assert cmd.vx == pytest.approx(0.3)
        assert cmd.vy == pytest.approx(-0.1)
        assert cmd.vz == pytest.approx(0.05)
        assert cmd.yaw_rate == pytest.approx(0.2)
        assert cmd.valid is True
        assert cmd.age_s() < 1.0
    finally:
        reader.close()
        writer.close()
        writer.unlink()


def test_latest_write_wins(shm_name):
    writer = CVCommandWriter(name=shm_name)
    reader = CVCommandReader(name=shm_name)
    try:
        writer.write(0.1, 0.0, 0.0, 0.0)
        writer.write(0.9, 0.0, 0.0, 0.0)
        cmd = reader.read()
        assert cmd.vx == pytest.approx(0.9)
    finally:
        reader.close()
        writer.close()
        writer.unlink()


def test_invalid_flag_round_trips(shm_name):
    writer = CVCommandWriter(name=shm_name)
    reader = CVCommandReader(name=shm_name)
    try:
        writer.write(0.1, 0.0, 0.0, 0.0, valid=False)
        cmd = reader.read()
        assert cmd.valid is False
    finally:
        reader.close()
        writer.close()
        writer.unlink()


def test_age_s_reflects_write_time(shm_name):
    writer = CVCommandWriter(name=shm_name)
    reader = CVCommandReader(name=shm_name)
    try:
        writer.write(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.05)
        cmd = reader.read()
        assert cmd.age_s() >= 0.05
    finally:
        reader.close()
        writer.close()
        writer.unlink()


def test_writer_survives_stale_segment_left_by_a_crashed_writer(shm_name):
    first = CVCommandWriter(name=shm_name)
    first.write(0.5, 0.0, 0.0, 0.0)
    # simulate a crash: close() without unlink(), like a killed systemd unit
    first.close()

    second = CVCommandWriter(name=shm_name)  # must not raise FileExistsError
    reader = CVCommandReader(name=shm_name)
    try:
        second.write(0.7, 0.0, 0.0, 0.0)
        cmd = reader.read()
        assert cmd.vx == pytest.approx(0.7)
    finally:
        reader.close()
        second.close()
        second.unlink()
