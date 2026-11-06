"""MJPEG over HTTP, read with the standard library (no ROS, so it runs and tests on its own).

camera_node uses this for an http:// device: the robot's ESP32-CAM (AI-Thinker, OV2640, firmware
github.com/jakhon37/myESP32CAM-BLACKBOX). cv2.VideoCapture(url) reads that stream too, but this
stream needs three things it cannot do:

Stamps
    The stream carries no timestamps. The firmware sends a part's boundary right after it takes
    the frame from the driver, so the boundary's arrival is the best estimate of the capture
    time. The camera's long pauses mostly fall inside a frame (32 of 45 at 320x240 and 44 of 45
    at 640x480 on 2026-11-07), so a frame stamped when its last byte arrives can look fresh when
    it is seconds old. The estimate is still late by up to one sensor frame.
Setup on every connection
    The firmware forgets its frame size when it reboots and serves one request at a time, so
    nothing reaches it while a client holds /stream: setup has to go first, every time.
A short timeout
    The firmware can leave a connection open and send nothing (its handler returns when the
    driver has no frame). OpenCV's FFmpeg backend waits 30 s by default; this reconnects sooner.
"""

import http.client
import re
import time
from typing import List, Optional, Sequence
from urllib.parse import urlsplit

import cv2
import numpy as np

# socket.timeout and ConnectionError are OSErrors; ValueError is a malformed reply or part.
_STREAM_ERRORS = (OSError, http.client.HTTPException, ValueError)
_BOUNDARY = re.compile(r'boundary="?([^";\s]+)', re.IGNORECASE)
_LINE_LIMIT = 256
_MAX_LINES_BEFORE_BOUNDARY = 4
_MAX_PART_HEADERS = 16
_MAX_SETUP_REPLY = 1024


def parse_setup_paths(text: str) -> List[str]:
    """"/resolution?size=QVGA, /fps?val=10" -> the two paths."""
    return [path.strip() for path in text.split(",") if path.strip()]


class MjpegHttpSource(object):
    """multipart/x-mixed-replace JPEG parts with a Content-Length, on the capture-source interface."""

    def __init__(self, url: str, setup_paths: Sequence[str] = (), timeout_s: float = 2.0) -> None:
        parts = urlsplit(url)
        if parts.scheme != "http" or not parts.hostname:
            raise ValueError("expected an http:// URL, got %r" % (url,))
        if timeout_s <= 0:
            raise ValueError("the stream timeout must be > 0, got %s" % timeout_s)
        self._url = url
        self._host, self._port = parts.hostname, parts.port or 80
        self._path = (parts.path or "/") + ("?" + parts.query if parts.query else "")
        self._setup_paths = list(setup_paths)
        self._timeout = timeout_s
        self._connection: Optional[http.client.HTTPConnection] = None
        self._response: Optional[http.client.HTTPResponse] = None
        self._delimiter = b""
        self._jpeg = b""
        self._boundary_t = 0.0
        self.last_error = ""

    def describe(self) -> str:
        return "MJPEG stream %s" % self._url

    def is_open(self) -> bool:
        return self._connection is not None

    def open(self) -> bool:
        self.close()
        connection = None
        try:
            for path in self._setup_paths:
                self._send_setup(path)
            connection = self._connect()
            connection.request("GET", self._path)
            response = connection.getresponse()
            content_type = response.getheader("Content-Type", "")
            boundary = _BOUNDARY.search(content_type)
            if response.status != 200 or boundary is None:
                raise ValueError("not an MJPEG stream: HTTP %d, %r" % (response.status, content_type))
        except _STREAM_ERRORS as exc:
            if connection is not None:
                connection.close()
            self.last_error = str(exc) or type(exc).__name__
            return False
        self._connection, self._response = connection, response
        self._delimiter = b"--" + boundary.group(1).encode()
        self.last_error = ""
        return True

    def grab(self) -> bool:
        """Read one part, however long it takes; a dead stream closes the source."""
        if self._response is None:
            return False
        try:
            self._jpeg = self._read_part(self._response)
        except _STREAM_ERRORS as exc:
            self.last_error = str(exc) or type(exc).__name__
            self.close()
            return False
        return True

    def frame_age(self) -> float:
        """Seconds since the boundary of the frame last grabbed arrived."""
        return time.monotonic() - self._boundary_t

    def retrieve(self) -> Optional[np.ndarray]:
        """The frame last grabbed as BGR, or None if its JPEG does not decode."""
        return cv2.imdecode(np.frombuffer(self._jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)

    def close(self) -> None:
        if self._connection is not None:
            self._response.close()
            self._connection.close()
            self._connection = self._response = None

    def _connect(self) -> http.client.HTTPConnection:
        return http.client.HTTPConnection(self._host, self._port, timeout=self._timeout)

    def _send_setup(self, path: str) -> None:
        connection = self._connect()
        try:
            connection.request("GET", path)
            response = connection.getresponse()
            reply = response.read(_MAX_SETUP_REPLY)
        finally:
            connection.close()
        # While it records to SD, the firmware refuses a new size with a 200 and this in the body.
        if response.status != 200 or b'"status":"error"' in reply:
            raise ValueError("setup %s refused: HTTP %d, %r" % (path, response.status, reply[:80]))

    def _read_part(self, response: http.client.HTTPResponse) -> bytes:
        for _ in range(_MAX_LINES_BEFORE_BOUNDARY):
            line = response.readline(_LINE_LIMIT)
            if not line:
                raise ValueError("the stream ended")
            if line.strip() == self._delimiter:
                break
        else:
            raise ValueError("no part boundary where one was expected")
        self._boundary_t = time.monotonic()

        length = None
        for _ in range(_MAX_PART_HEADERS):
            line = response.readline(_LINE_LIMIT).strip()
            if not line:
                break
            name, _, value = line.partition(b":")
            if name.strip().lower() == b"content-length":
                length = int(value)
        if length is None or length <= 0:
            raise ValueError("a part without a usable Content-Length")
        jpeg = response.read(length)
        if len(jpeg) != length:
            raise ValueError("the stream ended inside a frame")
        return jpeg
