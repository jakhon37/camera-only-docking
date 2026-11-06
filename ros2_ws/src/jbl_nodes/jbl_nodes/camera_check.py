"""Is the camera steady enough to record or run on?

    ros2 run jbl_nodes camera_check [--seconds S] [--params-file robot.yaml] [--domain-id N]

Starts camera_node and a constant-output policy_runner from the params file on a private ROS
domain, so nothing here can reach safety_gate or the motors, and measures them from outside:
frames per second, the longest gap between frames, and the policy ticks that fall back to zero
because the newest frame is too old (dock_judge counts those as "still", so they can fake a
stop). PASS means all three meet the limits below and the view is not nearly black, because a
measurement of a dark room or a covered lens says little about the real scene (docs/PROTOCOL.md
section 4).

The ESP32-CAM serves one client at a time: stop anything else that reads it first, including
robot_record.launch.py's camera_node and any browser open on its stream. ``--test-pattern``
replaces the camera with synthetic frames (a bench test of this tool).

Exit status: 0 pass, 1 fail, 2 bad arguments or no frame at all.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from typing import List, Optional

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import cv2
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

MIN_FPS = 9.0
MAX_GAP_S = 0.5
MAX_FALLBACK_PCT = 5.0
MIN_BRIGHTNESS = 25.0  # mean grey level, 0-255; a covered lens or a dark room measured about 9
BRIGHTNESS_EVERY = 10  # frames
DEFAULT_SECONDS = 180.0
DEFAULT_DOMAIN_ID = 88
FIRST_FRAME_TIMEOUT_S = 25.0
SETTLE_S = 2.0  # what arrives while the nodes are still matching is not counted
STOP_TIMEOUT_S = 5.0


def failures(fps: float, max_gap_s: float, fallback_pct: float, brightness: float) -> List[str]:
    """What misses the pass mark, in words; empty means PASS."""
    missed = []
    if fps < MIN_FPS:
        missed.append("%.2f fps is under %.1f" % (fps, MIN_FPS))
    if max_gap_s > MAX_GAP_S:
        missed.append("the longest gap, %.2f s, is over %.1f s" % (max_gap_s, MAX_GAP_S))
    if fallback_pct >= MAX_FALLBACK_PCT:
        missed.append("%.1f%% fallback zeros is not under %.1f%%" % (fallback_pct, MAX_FALLBACK_PCT))
    if brightness < MIN_BRIGHTNESS:
        missed.append(
            "the view is nearly black (mean brightness %.0f of 255), so this says little about the real "
            "scene: light it or uncover the lens" % brightness
        )
    return missed


def _percentile(values: List[float], q: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


class _Probe(Node):
    def __init__(self) -> None:
        super().__init__("camera_check")
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.frames: List[float] = []
        self.jpeg_bytes: List[int] = []
        self.brightness: List[float] = []
        self.ticks = 0
        self.ages_ms: List[float] = []
        self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self._on_frame, qos)
        self.create_subscription(Twist, "/cmd_vel_in", self._on_tick, 10)
        self.create_subscription(String, "/policy/latency", self._on_latency, 10)

    def _on_frame(self, msg: CompressedImage) -> None:
        self.frames.append(time.monotonic())
        self.jpeg_bytes.append(len(msg.data))
        if len(self.frames) % BRIGHTNESS_EVERY == 1:
            gray = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            if gray is not None:
                self.brightness.append(float(gray.mean()))

    def _on_tick(self, _msg: Twist) -> None:
        self.ticks += 1

    def _on_latency(self, msg: String) -> None:
        self.ages_ms.append(json.loads(msg.data)["image_age_ms"])

    def reset(self) -> None:
        self.frames, self.jpeg_bytes, self.brightness, self.ticks, self.ages_ms = [], [], [], 0, []


def _spin_until(node: Node, deadline: float, done=lambda: False) -> None:
    while time.monotonic() < deadline and not done():
        rclpy.spin_once(node, timeout_sec=0.05)


def _start_node(name: str, ros_args: List[str], env: dict, quiet: bool = False) -> subprocess.Popen:
    exe = os.path.join(get_package_prefix("jbl_nodes"), "lib", "jbl_nodes", name)
    output = subprocess.DEVNULL if quiet else None
    return subprocess.Popen([exe, "--ros-args"] + ros_args, env=env, stdout=output, stderr=output)


def _stop_node(process: subprocess.Popen) -> None:
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=STOP_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def _report(probe: _Probe, span_s: float) -> int:
    gaps = [b - a for a, b in zip(probe.frames, probe.frames[1:])]
    if not gaps or not probe.brightness or probe.ticks == 0:
        print("FAIL: too little to measure (%d frames, %d policy ticks)" % (len(probe.frames), probe.ticks))
        return 1
    fps = len(probe.frames) / span_s
    fallback_pct = 100.0 * max(0, probe.ticks - len(probe.ages_ms)) / probe.ticks
    brightness = _percentile(probe.brightness, 0.5)
    print(
        "camera_check: %.0f s at %.2f fps, longest gap %.3f s, %.1f%% fallback zeros"
        % (span_s, fps, max(gaps), fallback_pct)
    )
    print(
        "  gaps p50/p95 %.0f/%.0f ms, %d over %.1f s; JPEG median %.1f KB; mean brightness %.0f of 255"
        % (
            _percentile(gaps, 0.5) * 1e3,
            _percentile(gaps, 0.95) * 1e3,
            sum(gap > MAX_GAP_S for gap in gaps),
            MAX_GAP_S,
            _percentile(probe.jpeg_bytes, 0.5) / 1024.0,
            brightness,
        )
    )
    if probe.ages_ms:
        print(
            "  image age at a policy tick p50/p95 %.0f/%.0f ms"
            % (_percentile(probe.ages_ms, 0.5), _percentile(probe.ages_ms, 0.95))
        )
    missed = failures(fps, max(gaps), fallback_pct, brightness)
    print("FAIL: " + "; ".join(missed) if missed else "PASS")
    return 1 if missed else 0


def main(argv: Optional[List[str]] = None) -> int:
    argv = list(sys.argv if argv is None else argv)
    parser = argparse.ArgumentParser(prog="camera_check", description="Measure the camera against the pass mark.")
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS, metavar="S", help="(default %(default)s)")
    parser.add_argument(
        "--params-file",
        default=os.path.join(get_package_share_directory("jbl_nodes"), "config", "robot.yaml"),
        help="the camera's configuration (default: this package's config/robot.yaml)",
    )
    parser.add_argument(
        "--domain-id", type=int, default=DEFAULT_DOMAIN_ID, metavar="N", help="private ROS domain (default %(default)s)"
    )
    parser.add_argument("--test-pattern", action="store_true", help="synthetic frames instead of the camera")
    opts = parser.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])
    if opts.seconds <= 0:
        parser.error("--seconds must be positive")
    # This must never share a domain with the robot: policy_runner publishes /cmd_vel_in.
    if str(opts.domain_id) == os.environ.get("ROS_DOMAIN_ID", "0"):
        parser.error("--domain-id %d is this shell's own ROS domain; pick another" % opts.domain_id)

    os.environ["ROS_DOMAIN_ID"] = str(opts.domain_id)
    env = dict(os.environ)
    params = ["--params-file", opts.params_file]
    # With the node's name: a plain -p loses to that node's own section in the params file.
    camera_args = params + (["-p", "camera_node:source:=test_pattern"] if opts.test_pattern else [])
    runner_args = params + ["-p", "policy_runner:backend:=constant", "-p", "policy_runner:enabled_on_start:=true"]
    nodes = [_start_node("camera_node", camera_args, env), _start_node("policy_runner", runner_args, env, quiet=True)]
    rclpy.init(args=argv)
    probe = _Probe()
    status = 2
    try:
        print("camera_check: waiting for the first frame", flush=True)
        started = time.monotonic()
        _spin_until(probe, started + FIRST_FRAME_TIMEOUT_S, lambda: bool(probe.frames))
        if not probe.frames:
            print("FAIL: no frame in %.0f s" % FIRST_FRAME_TIMEOUT_S)
        else:
            print(
                "camera_check: first frame after %.1f s; measuring for %.0f s"
                % (time.monotonic() - started, opts.seconds),
                flush=True,
            )
            _spin_until(probe, time.monotonic() + SETTLE_S)
            probe.reset()
            started = time.monotonic()
            _spin_until(probe, started + opts.seconds)
            status = _report(probe, time.monotonic() - started)
    except KeyboardInterrupt:
        print("camera_check: interrupted")
    finally:
        for process in reversed(nodes):
            _stop_node(process)
        probe.destroy_node()
        rclpy.try_shutdown()
    return status


if __name__ == "__main__":
    sys.exit(main())
