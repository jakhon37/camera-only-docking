"""The ESP32-CAM firmware's HTTP behaviour for tests: one request at a time, a chunked multipart
/stream, sizes and rates it forgets when it reboots, and the failures measured on the real one."""

import http.server
import socket
import threading
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np

BOUNDARY = "123456789000000000000987654321"
SIZES = {"QVGA": (320, 240), "VGA": (640, 480)}


def _jpeg(size):
    width, height = size
    frame = np.dstack(
        [
            np.linspace(0, 255, width, dtype=np.uint8)[None, :].repeat(height, 0),
            np.linspace(0, 255, height, dtype=np.uint8)[:, None].repeat(width, 1),
            np.full((height, width), 96, dtype=np.uint8),
        ]
    )
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    assert ok
    # A comment holding the part delimiter: only a reader that trusts Content-Length gets past it.
    trap = ("\r\n--%s\r\n" % BOUNDARY).encode()
    comment = b"\xff\xfe" + (len(trap) + 2).to_bytes(2, "big") + trap
    data = encoded.tobytes()
    return data[:2] + comment + data[2:]


class FakeEsp32Cam(object):
    """Set the attributes below, before or during a test, to script the camera."""

    def __init__(self, fps=30.0):
        self.period = 1.0 / fps
        self.requests = []
        self.size = "VGA"
        self.index = 0
        self.stall_inside = {}  # frame index -> seconds of silence inside that frame
        self.silent_after = None  # after this many frames on a connection: keep it open, send nothing
        self.reboot_after = None  # after this many frames on a connection: drop it and forget the size
        self.bad_jpeg_at = None  # frame index that carries garbage instead of a JPEG
        self.setup_reply = b'{"status":"ok"}'
        self._stopping = threading.Event()
        self._jpegs = {name: _jpeg(size) for name, size in SIZES.items()}
        self._server = http.server.HTTPServer(("127.0.0.1", 0), _Handler)
        self._server.camera = self
        self._thread = threading.Thread(target=self._server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        self._thread.start()

    def url(self, path="/stream"):
        return "http://127.0.0.1:%d%s" % (self._server.server_address[1], path)

    def close(self):
        self._stopping.set()
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)


class _Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *_args):
        pass

    def do_GET(self):
        camera = self.server.camera
        camera.requests.append(self.path)
        url = urlsplit(self.path)
        self.close_connection = True
        if url.path == "/stream":
            self._stream(camera)
        elif url.path == "/resolution":
            if b'"error"' not in camera.setup_reply:
                camera.size = parse_qs(url.query)["size"][0]
            self._reply(camera.setup_reply)
        elif url.path == "/fps":
            self._reply(b'{"status":"ok"}')
        else:
            self.send_error(404)

    def _reply(self, body):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _chunk(self, data):
        self.wfile.write(b"%x\r\n%s\r\n" % (len(data), data))

    def _stream(self, camera):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace;boundary=" + BOUNDARY)
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()
        sent = 0
        try:
            while not camera._stopping.is_set():
                if camera.reboot_after == sent:
                    camera.reboot_after = None
                    camera.size = "VGA"
                    return
                if camera.silent_after == sent:
                    camera.silent_after = None
                    self._wait_until_the_client_leaves(camera)
                    return
                jpeg = camera._jpegs[camera.size]
                if camera.index == camera.bad_jpeg_at:
                    jpeg = b"this is not a JPEG"
                self._chunk(("\r\n--%s\r\n" % BOUNDARY).encode())
                self._chunk(("Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg)).encode())
                self._chunk(jpeg[: len(jpeg) // 2])
                camera._stopping.wait(camera.stall_inside.pop(camera.index, 0.0))
                self._chunk(jpeg[len(jpeg) // 2 :])
                camera.index += 1
                sent += 1
                camera._stopping.wait(camera.period)
        except OSError:
            pass  # the client left: the firmware's next send fails the same way

    def _wait_until_the_client_leaves(self, camera):
        self.connection.settimeout(0.02)
        while not camera._stopping.is_set():
            try:
                if not self.connection.recv(1):
                    return
            except socket.timeout:
                continue
            except OSError:
                return
