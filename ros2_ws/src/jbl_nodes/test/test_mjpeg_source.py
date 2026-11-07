"""The ESP32-CAM stream reader against a fake of the firmware (pure Python, no ROS needed)."""

import socket
import time

import pytest

from fake_esp32cam import FakeEsp32Cam
from jbl_nodes.mjpeg_source import MjpegHttpSource, parse_setup_paths

QVGA = "/resolution?size=QVGA"
FPS_10 = "/fps?val=10"


@pytest.fixture
def camera():
    fake = FakeEsp32Cam()
    try:
        yield fake
    finally:
        fake.close()


@pytest.fixture
def source(camera):
    reader = MjpegHttpSource(camera.url(), [QVGA, FPS_10], timeout_s=1.0)
    try:
        yield reader
    finally:
        reader.close()


def test_parse_setup_paths():
    assert parse_setup_paths("") == []
    assert parse_setup_paths(" , ") == []
    assert parse_setup_paths("/resolution?size=QVGA, /fps?val=10,") == [QVGA, FPS_10]


@pytest.mark.parametrize("url", ["https://camera/stream", "http://", "/dev/video0", "rtsp://camera/stream"])
def test_rejects_what_is_not_an_http_url(url):
    with pytest.raises(ValueError, match="http://"):
        MjpegHttpSource(url)


def test_rejects_a_timeout_that_cannot_work(camera):
    with pytest.raises(ValueError, match="timeout"):
        MjpegHttpSource(camera.url(), timeout_s=0.0)


def test_frames_arrive_whole_at_the_size_the_setup_asked_for(source):
    assert not source.is_open()
    assert source.open(), source.last_error
    assert source.is_open() and source.last_error == ""
    for _ in range(3):
        assert source.grab(), source.last_error
        assert source.retrieve().shape == (240, 320, 3)
    source.close()
    assert not source.is_open() and not source.grab()


def test_a_camera_with_no_setup_keeps_its_own_size(camera):
    reader = MjpegHttpSource(camera.url())
    assert reader.open() and reader.grab()
    assert reader.retrieve().shape == (480, 640, 3)
    reader.close()
    assert camera.requests == ["/stream"]


def test_setup_goes_first_on_every_connection_because_a_reboot_forgets_it(camera, source):
    camera.reboot_after = 2
    assert source.open()
    assert source.grab() and source.grab()
    assert not source.grab(), "the camera rebooted: the connection is gone"
    assert not source.is_open() and source.last_error
    assert camera.size == "VGA"

    assert source.open() and source.grab()
    assert source.retrieve().shape == (240, 320, 3)
    assert camera.requests == [QVGA, FPS_10, "/stream", QVGA, FPS_10, "/stream"]


def test_a_frame_that_stalls_is_aged_from_its_boundary(camera, source):
    camera.stall_inside[2] = 0.5
    assert source.open()
    ages = []
    for _ in range(4):
        assert source.grab()
        ages.append(source.frame_age())
        assert source.retrieve() is not None
    assert ages[2] >= 0.45, ages
    assert ages[0] < 0.25 and ages[1] < 0.25 and ages[3] < 0.25, ages


def test_a_silent_stream_times_out_and_the_next_open_recovers(camera):
    camera.silent_after = 1
    reader = MjpegHttpSource(camera.url(), [QVGA], timeout_s=0.3)
    assert reader.open() and reader.grab()
    t0 = time.monotonic()
    assert not reader.grab()
    assert 0.25 <= time.monotonic() - t0 < 2.0
    assert "timed out" in reader.last_error and not reader.is_open()
    assert reader.open(), reader.last_error
    assert reader.grab()
    reader.close()


def test_a_camera_that_is_off_does_not_open():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    reader = MjpegHttpSource("http://127.0.0.1:%d/stream" % port, [QVGA], timeout_s=0.5)
    assert not reader.open()
    assert reader.last_error and not reader.is_open() and not reader.grab()


def test_a_refused_setup_does_not_open_the_stream(camera, source):
    camera.setup_reply = b'{"status":"error","msg":"Cannot change resolution while recording"}'
    assert not source.open()
    assert "refused" in source.last_error and "recording" in source.last_error
    assert "/stream" not in camera.requests


@pytest.mark.parametrize("path", ["/missing", QVGA], ids=["404", "not-multipart"])
def test_a_url_that_is_no_mjpeg_stream_does_not_open(camera, path):
    reader = MjpegHttpSource(camera.url(path))
    assert not reader.open()
    assert "not an MJPEG stream" in reader.last_error


def test_a_broken_jpeg_loses_one_frame_not_the_stream(camera, source):
    camera.bad_jpeg_at = 1
    assert source.open()
    results = []
    for _ in range(3):
        assert source.grab()
        results.append(source.retrieve())
    assert results[0] is not None and results[1] is None and results[2] is not None
