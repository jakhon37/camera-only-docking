"""Draw the figures of README.md and docs/ from results/ and config/.

    python tools/make_figures.py                        # docs/figures/<name>.svg and <name>-dark.svg
    python tools/make_figures.py --png /tmp/preview     # also PNGs, to look at them
    python tools/make_figures.py --only results,poses   # some of them
    python tools/make_figures.py --bag sim_work/bags/humble/servo
        # first refresh results/sim_showcase/ and docs/media/docking.gif from a bag of
        # scripts/sim_e2e.sh's servo stage (the bag is not in git; the files made from it are)

Every number in a chart is read from results/, every pose from config/, and the camera views are
rendered by jbl_nodes.sim_world, the simulator the results came from. The two diagrams use the
node and topic names of docs/INTERFACES.md. Each figure is written for a light and a dark page;
the Markdown picks one with <picture>. Colours follow the meaning: the servo baseline is always
blue and behaviour cloning orange (aqua for the model trained through the 62.2 deg lens).
Host tools environment (requirements-tools.txt); --bag also needs rosbags and Pillow.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from collections.abc import Callable
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import patheffects  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "jbl_nodes"))
from jbl_nodes import sim_world as sw  # noqa: E402
from jbl_nodes.aruco_utils import MarkerDetector  # noqa: E402

FIGURES = ROOT / "docs" / "figures"
MEDIA = ROOT / "docs" / "media"
CONFIG = ROOT / "ros2_ws" / "src" / "jbl_nodes" / "config"
V2, V3, V4 = (ROOT / "results" / d for d in ("sim_eval_v2", "sim_eval_v3_bcfix", "sim_eval_v4_cleanstop"))
SHOWCASE = ROOT / "results" / "sim_showcase"
GIF_POSE = 9  # the marker starts out of view: a search turn, then the approach (18 s)
BAND = (0.18, 0.32)  # dock_judge's band on the lidar's front range (docs/INTERFACES.md)
STILL_V = {"v2-v3": 0.02, "v4": 0.005}  # dock_judge request_still_v before and from v4

THEMES = {
    "light": {"surface": "#fcfcfb", "plane": "#f3f2ef", "ink": "#0b0b0b", "ink2": "#52514e", "muted": "#898781",
              "grid": "#e1e0d9", "axis": "#c3c2b7", "critical": "#d03b3b", "tint": 0.13,
              "blue": "#2a78d6", "orange": "#eb6834", "aqua": "#1baf7a", "yellow": "#eda100", "magenta": "#e87ba4",
              "violet": "#6250d6", "green": "#1baf7a"},
    "dark": {"surface": "#1a1a19", "plane": "#242422", "ink": "#f0efec", "ink2": "#c3c2b7", "muted": "#898781",
             "grid": "#2c2c2a", "axis": "#383835", "critical": "#d03b3b", "tint": 0.22,
             "blue": "#3987e5", "orange": "#d95926", "aqua": "#199e70", "yellow": "#c98500", "magenta": "#d55181",
             "violet": "#9085e9", "green": "#199e70"},
}
METHODS = {"servo": ("servo baseline", "blue"), "bc_v4": ("BC, trained at 80°", "orange"),
           "bc_v4_62": ("BC, trained through the 62° lens", "aqua")}


def blend(fg: str, bg: str, a: float) -> str:
    """fg over bg at opacity a, as an opaque colour: SVG viewers then agree on it."""
    f, b = (tuple(int(c[i:i + 2], 16) for i in (1, 3, 5)) for c in (fg, bg))
    return "#" + "".join(f"{round(a * x + (1 - a) * y):02x}" for x, y in zip(f, b))


def style() -> None:
    plt.rcParams.update({"font.family": ["Noto Sans", "DejaVu Sans"], "font.size": 10, "svg.fonttype": "path",
                         "svg.hashsalt": "camera-only-docking", "axes.linewidth": 1.0,
                         "axes.unicode_minus": True})


# ---------------------------------------------------------------------------------------------- data


def read_csv(path: Path) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def scored(path: Path) -> dict[str, list[dict]]:
    """trials.csv rows by method, one per pose x round slot: the first success or fail row (an abort
    never counts, a success with an intervention is a fail), as tools/analyze_trials.py scores them."""
    slots: dict[tuple, dict] = {}
    for row in read_csv(path):
        if row["outcome"] in ("success", "fail"):
            slots.setdefault((row["method"], row["pose_id"], row["round"]), row)
    out: dict[str, list[dict]] = {}
    for (method, _, _), row in slots.items():
        out.setdefault(method, []).append(row)
    return out


def won(row: dict) -> bool:
    return row["outcome"] == "success" and int(row.get("interventions") or 0) == 0


def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def last_second_request(row: dict) -> float:
    """Smallest |v| a trial's controller requested in its last second. v4's files hold the
    controller's own requests (own_v_...), without policy_runner's fallback zeros; earlier files
    have every request only (results/sim_eval_v4_cleanstop/SUMMARY.md)."""
    value = row.get("own_v_min_last1s") or row["cmd_v_min_last1s"]
    return abs(float(value))


# ---------------------------------------------------------------------------------------------- layout


def chart(t: dict, width: float, plot_height: float, title: str, subtitle: str, left: float,
          right: float = 0.35, top: float = 1.15, bottom: float = 0.55):
    """A figure laid out in inches: title, subtitle, a legend row, then the plot."""
    height = top + plot_height + bottom
    fig = plt.figure(figsize=(width, height), facecolor=t["surface"])
    ax = fig.add_axes((left / width, bottom / height, (width - left - right) / width, plot_height / height),
                      facecolor=t["surface"])
    fig.text(0.2 / width, 1 - 0.22 / height, title, color=t["ink"], fontsize=12.5, fontweight="bold", va="top")
    fig.text(0.2 / width, 1 - 0.5 / height, subtitle, color=t["ink2"], fontsize=9.5, va="top")
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(t["axis"])
    ax.tick_params(colors=t["muted"], labelcolor=t["ink2"], length=0, labelsize=9.5)
    ax.grid(axis="x", color=t["grid"], linewidth=0.8)
    ax.set_axisbelow(True)
    fig.legend_y = 1 - 0.86 / height
    return fig, ax


def legend(fig, t: dict, items, x_in: float, marker: str = "o", size: float = 9.5) -> None:
    """A row of (label, colour) swatches at the legend line, starting x_in inches from the left."""
    width = fig.get_figwidth()
    x = x_in / width
    for label, colour in items:
        fig.text(x, fig.legend_y, {"o": "●", "s": "■", "-": "—"}[marker], color=colour,
                 fontsize=size + 1, va="center")
        txt = fig.text(x + 0.17 / width, fig.legend_y, label, color=t["ink2"], fontsize=size, va="center")
        fig.canvas.draw()
        x = txt.get_window_extent().x1 / fig.dpi / width + 0.3 / width


def canvas(t: dict, w: float, h: float):
    """A diagram in units of 0.1 inch: x in [0, 10 w], y in [0, 10 h]."""
    fig = plt.figure(figsize=(w, h), facecolor=t["surface"])
    ax = fig.add_axes((0, 0, 1, 1), facecolor=t["surface"])
    ax.set_xlim(0, 10 * w)
    ax.set_ylim(0, 10 * h)
    ax.axis("off")
    return fig, ax


def node(ax, t: dict, x, y, w, h, title, body=(), hue=None, title_size=10.2, body_size=8.4, mono=False):
    """A rounded box: bold title over lighter body lines, centred; hue tints the fill and edge."""
    edge = t[hue] if hue else t["axis"]
    fill = blend(t[hue], t["surface"], t["tint"]) if hue else t["plane"]
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.9", facecolor=fill,
                                edgecolor=edge, linewidth=1.2, zorder=2))
    lines = [(title, title_size, "bold", t["ink"])] + [(b, body_size, "normal", t["ink2"]) for b in body]
    step = 2.05 if body else 0
    top = y + h / 2 + step * (len(lines) - 1) / 2
    for i, (text, size, weight, colour) in enumerate(lines):
        family = "DejaVu Sans Mono" if (mono and i == 0) else None
        ax.text(x + w / 2, top - i * step, text, fontsize=size, fontweight=weight, color=colour, ha="center",
                va="center", zorder=3, **({"family": family} if family else {}))


def arrow(ax, t: dict, a, b, label=None, colour=None, rad=0.0, dash=False, label_at=0.5, label_dx=0.0,
          label_dy=0.9, label_ha="center", lw=1.3):
    colour = colour or t["muted"]
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=10, color=colour, linewidth=lw,
                                 connectionstyle=f"arc3,rad={rad}", linestyle=(0, (3, 2)) if dash else "-",
                                 shrinkA=0, shrinkB=0, zorder=1))
    if label:
        x = a[0] + (b[0] - a[0]) * label_at + label_dx
        y = a[1] + (b[1] - a[1]) * label_at + label_dy
        ax.text(x, y, label, fontsize=7.6, color=t["ink2"], ha=label_ha, va="center", zorder=3,
                family="DejaVu Sans Mono")


def wire(ax, t: dict, points, label=None, seg=0, side="above", dash=False, rotate=False, dot=None):
    """A right-angled wire through points, with an arrowhead on its last segment. The label sits on
    segment ``seg``, on ``side`` (above, below, left or right) of it."""
    colour, style_ = t["muted"], ((0, (3, 2)) if dash else "-")
    if len(points) > 2:
        ax.plot([p[0] for p in points[:-1]], [p[1] for p in points[:-1]], color=colour, linewidth=1.3,
                linestyle=style_, solid_joinstyle="miter", zorder=1)
    ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle="-|>", mutation_scale=10, color=colour,
                                 linewidth=1.3, linestyle=style_, shrinkA=0, shrinkB=0, zorder=1))
    if dot:
        ax.plot([dot[0]], [dot[1]], "o", color=colour, markersize=3.6, zorder=2)
    if label:
        (x1, y1), (x2, y2) = points[seg], points[seg + 1]
        x, y = (x1 + x2) / 2, (y1 + y2) / 2
        dx, dy, ha = {"above": (0, 1.0, "center"), "below": (0, -1.0, "center"), "left": (-0.8, 0, "right"),
                      "right": (0.8, 0, "left")}[side]
        ax.text(x + dx, y + dy, label, fontsize=7.6, color=t["ink2"], ha="center" if rotate else ha, va="center",
                rotation=90 if rotate else 0, zorder=3, family="DejaVu Sans Mono")


def header(ax, t: dict, x, y, number, text, hue):
    ax.add_patch(FancyBboxPatch((x, y - 1.3), 2.6, 2.6, boxstyle="round,pad=0,rounding_size=1.3",
                                facecolor=t[hue], edgecolor="none", zorder=3))
    ax.text(x + 1.3, y, number, fontsize=10, fontweight="bold", color="#ffffff", ha="center", va="center",
            zorder=4)
    ax.text(x + 3.6, y, text, fontsize=11.2, fontweight="bold", color=t["ink"], va="center")


# ---------------------------------------------------------------------------------------------- diagrams


def overview(t: dict):
    fig, ax = canvas(t, 10, 5.3)
    ax.text(2, 50.5, "Demonstrate, learn, then compare on the same start poses", fontsize=13, fontweight="bold",
            color=t["ink"], va="center")
    ax.text(2, 47.4, "Every trial is decided by the same referee node, and only the safety gate can drive the "
            "motors, whichever controller runs", fontsize=9.5, color=t["ink2"], va="center")
    cols = [(2, "1", "Demonstrate", "violet"), (35, "2", "Learn, off the robot", "orange"),
            (68, "3", "Evaluate, paired", "blue")]
    for x, number, text, hue in cols:
        ax.add_patch(FancyBboxPatch((x, 8.5), 30, 34.5, boxstyle="round,pad=0,rounding_size=1.2",
                                    facecolor=t["plane"], edgecolor="none", zorder=0))
        header(ax, t, x + 1.6, 40.2, number, text, hue)
    w = 27
    node(ax, t, 3.5, 29, w, 7.6, "Operator with teleop", ["web pad, keyboard or joystick", "requests (v, w) on /cmd_vel_in"],
         hue="violet")
    node(ax, t, 3.5, 19.6, w, 7.6, "safety_gate  (C++, 50 Hz)", ["the only path to the motors:",
                                                                 "clamps, lidar stop, timeouts"])
    node(ax, t, 3.5, 10.2, w, 7.6, "ros2 bag", ["camera JPEG, requests, odometry,", "episode start and end"])
    node(ax, t, 36.5, 29, w, 7.6, "bag_to_lerobot", ["LeRobot v3 dataset, 10 fps:", "240×320 RGB → requested (v, w)"],
         hue="orange")
    node(ax, t, 36.5, 19.6, w, 7.6, "Behaviour cloning · ACT", ["ResNet18 trained on successful", "demos, start spots held out"])
    node(ax, t, 36.5, 10.2, w, 7.6, "ONNX + policy_meta.json", ["output checked against", "PyTorch to 1e-4"])
    node(ax, t, 69.5, 31.4, w, 5.4, "10 start poses × 3 rounds", ["counterbalanced, same slots for all"], hue="blue",
         body_size=8.2)
    node(ax, t, 69.5, 23.9, w, 5.4, "servo baseline   vs   policy", ["ArUco bearing + lidar  |  image → (v, w)"],
         body_size=8.2)
    node(ax, t, 69.5, 16.4, w, 5.4, "dock_judge decides", ["in band, marker ≤ 10°, still for 1 s"], body_size=8.2)
    node(ax, t, 69.5, 8.9 + 0.0, w, 5.4, "Wilson 95% CI · McNemar", ["paired by pose and round"], body_size=8.2)
    for x in (3.5 + w / 2, 36.5 + w / 2):
        arrow(ax, t, (x, 29), (x, 27.2))
        arrow(ax, t, (x, 19.6), (x, 17.8))
    for y in (31.4, 23.9, 16.4):
        arrow(ax, t, (69.5 + w / 2, y), (69.5 + w / 2, y - 2.1))
    arrow(ax, t, (30.5, 14.0), (36.5, 32.8), rad=-0.25)
    arrow(ax, t, (63.5, 14.0), (69.5, 26.6), rad=-0.25)
    ax.add_patch(FancyBboxPatch((2, 1.2), 96, 5.3, boxstyle="round,pad=0,rounding_size=1.0",
                                facecolor=blend(t["aqua"], t["surface"], t["tint"]), edgecolor=t["aqua"],
                                linewidth=1.0, zorder=0))
    ax.text(50, 3.85, "Robot: Jetson Xavier NX, ESP32-CAM, RPLidar A1, ESP32-S3 motor board   ·   "
            "Simulation: fake_robot publishes the same topics, so every stage runs without hardware",
            fontsize=8.8, color=t["ink"], ha="center", va="center")
    return fig


def architecture(t: dict):
    fig, ax = canvas(t, 10, 6.4)
    ax.text(2, 61.5, "System structure on the robot", fontsize=13, fontweight="bold", color=t["ink"], va="center")
    ax.text(2, 58.4, "ROS 2 Humble in Docker on the Jetson, with the topic names of docs/INTERFACES.md. "
            "Training and analysis run on a PC.", fontsize=9.5, color=t["ink2"], va="center")
    for x, w, title in ((1.5, 18.5, "Hardware"), (23, 56.5, "Jetson Xavier NX · Docker auto_ros:humble"),
                        (82.5, 16, "PC")):
        ax.add_patch(FancyBboxPatch((x, 2), w, 52.5, boxstyle="round,pad=0,rounding_size=1.2",
                                    facecolor=t["plane"], edgecolor="none", zorder=0))
        ax.text(x + w / 2, 52.0, title, fontsize=9.8, fontweight="bold", color=t["ink2"], ha="center",
                va="center")
    r1, r2, r3, r4, h = 41, 29.5, 17, 4.5, 7
    hx, hw, lx, lw, rx, rw, px_, pw = 3, 15.5, 26, 19, 54, 22, 84, 13
    node(ax, t, hx, r1, hw, h, "ESP32-CAM", ["OV2640 · MJPEG", "over Wi-Fi"], hue="violet")
    node(ax, t, hx, r2, hw, h, "RPLidar A1", ["360° · about 7 Hz", "USB"], hue="violet")
    node(ax, t, hx, r3, hw, h, "ESP32-S3", ["motors, encoders, IMU", "USB serial"], hue="violet")
    node(ax, t, lx, r1, lw, h, "camera_node", ["stamps each frame", "at its first byte"], mono=True)
    node(ax, t, lx, r2, lw, h, "rplidar_ros", ["lidar driver"], mono=True)
    node(ax, t, lx, r3, lw, h, "diffdrive_node", ["wheel commands", "and /odom"], mono=True)
    node(ax, t, lx, r4, lw, h, "episode", ["start / end / intervention"], mono=True)
    node(ax, t, rx, r1, rw, h, "dock_judge", ["referee: success or fail", "from camera, scan, odom"], hue="aqua",
         mono=True)
    node(ax, t, rx, r2, rw, h, "controller", ["teleop · servo_baseline", "· policy_runner"], hue="blue", mono=True)
    node(ax, t, rx, r3, rw, h, "safety_gate", ["C++ · 50 Hz · stops on /scan", "the only /cmd_vel publisher"],
         hue="critical", mono=True)
    node(ax, t, rx, r4, rw, h, "trial_logger · bag", ["trials.csv · rosbag"], mono=True)
    node(ax, t, px_, r1, pw, h, "bag_to_lerobot", ["→ LeRobot", "dataset"], mono=True, title_size=8.4)
    node(ax, t, px_, r2, pw, h, "train_bc", ["also train_act"], mono=True, title_size=8.4)
    node(ax, t, px_, r3, pw, h, "export_onnx", ["policy.onnx", "→ policy_runner"], mono=True, title_size=8.4)
    node(ax, t, px_, r4, pw, h, "analyze_trials", ["Wilson CIs,", "McNemar"], mono=True, title_size=8.4)
    mid = h / 2
    wire(ax, t, [(hx + hw, r1 + mid), (lx, r1 + mid)], "HTTP")
    wire(ax, t, [(hx + hw, r2 + mid), (lx, r2 + mid)], "USB")
    wire(ax, t, [(lx, r3 + 4.9), (hx + hw, r3 + 4.9)], "m v_l v_r")
    wire(ax, t, [(hx + hw, r3 + 2.1), (lx, r3 + 2.1)], "e · i", side="below")
    jx = (lx + lw + rx) / 2
    wire(ax, t, [(lx + lw, r1 + mid), (rx, r1 + mid)], dot=(jx, r1 + mid))
    ax.text(jx, r1 + h + 1.4, "/camera/image_raw/compressed", fontsize=7.6, color=t["ink2"], ha="center",
            va="center", family="DejaVu Sans Mono")
    wire(ax, t, [(jx, r1 + mid), (jx, r2 + 4.9), (rx, r2 + 4.9)])
    wire(ax, t, [(lx + lw, r2 + 2.1), (rx, r2 + 2.1)], "/scan", side="below")
    wire(ax, t, [(rx + rw / 2, r2), (rx + rw / 2, r3 + h)], "/cmd_vel_in", side="right")
    wire(ax, t, [(rx, r3 + mid), (lx + lw, r3 + mid)], "/cmd_vel")
    wire(ax, t, [(lx + lw, r4 + mid), (rx, r4 + mid)], "start / end")
    ex = rx + rw + 2.2
    wire(ax, t, [(rx + rw, r1 + mid), (ex, r1 + mid), (ex, r4 + 5.0), (rx + rw, r4 + 5.0)], "/episode/event",
         seg=1, side="right", rotate=True)
    tx = 81.0
    wire(ax, t, [(rx + rw, r4 + 1.6), (tx, r4 + 1.6), (tx, r1 + mid), (px_, r1 + mid)], dash=True)
    wire(ax, t, [(tx, r4 + 3.4), (px_, r4 + 3.4)], dash=True, dot=(tx, r4 + 3.4))
    ax.text(tx - 0.6, r2 + mid, "rsync", fontsize=7.6, color=t["ink2"], ha="center", va="center", rotation=90,
            family="DejaVu Sans Mono")
    for y in (r1, r2):
        arrow(ax, t, (px_ + pw / 2, y), (px_ + pw / 2, y - (r1 - r2 - h)))
    return fig


# ---------------------------------------------------------------------------------------------- charts


def results(t: dict):
    rows = []  # (group, method, k, n)
    groups = [("clean_ref", "clean start"), ("noise", "noise"), ("wide", "unseen start poses"),
              ("hfov62", "62° lens")]
    for cond, _ in groups:
        by_method = scored(V4 / cond / "trials.csv")
        for method in ("servo", "bc_v4", "bc_v4_62"):
            if method in by_method:
                trials = by_method[method]
                rows.append((cond, method, sum(map(won, trials)), len(trials)))
    fig, ax = chart(t, 8.4, 3.55, "Docking success in simulation (v4, pre-registered)",
                    "10 start poses × 3 rounds per condition (clean start: 1 round). Success as dock_judge "
                    "decides it, with Wilson 95% CIs", left=1.7, right=0.95)
    legend(fig, t, [(METHODS[m][0], t[METHODS[m][1]]) for m in ("servo", "bc_v4", "bc_v4_62")], 0.2)
    y, ticks, labels = 0.0, [], []
    for cond, title in groups:
        members = [r for r in rows if r[0] == cond]
        top = y
        for _, method, k, n in members:
            lo, hi = wilson(k, n)
            colour = t[METHODS[method][1]]
            ax.plot([100 * lo, 100 * hi], [y, y], color=colour, linewidth=2.2, solid_capstyle="round", zorder=2,
                    clip_on=False)
            ax.plot([100 * k / n], [y], "o", color=colour, markersize=7.5, markeredgecolor=t["surface"],
                    markeredgewidth=1.2, zorder=3, clip_on=False)
            ax.text(104, y, f"{k}/{n}", color=t["ink"], fontsize=9.5, va="center", fontweight="bold")
            y -= 0.62
        ticks.append((top + y + 0.62) / 2)
        labels.append(title)
        y -= 0.55
    ax.set_yticks(ticks, labels, color=t["ink"], fontsize=10)
    ax.set_ylim(y + 0.3, 0.55)
    ax.set_xlim(0, 100)
    ax.set_xticks(range(0, 101, 20), [f"{v}%" for v in range(0, 101, 20)])
    ax.tick_params(axis="y", pad=6)
    return fig


def stopping(t: dict):
    sets = [("v2", V2 / "noise", "bc", "BC"), ("v3", V3 / "noise", "bc_v2", "BC, re-recorded demos"),
            ("v4", V4 / "noise", "bc_v4", "BC, clean-stop demos"), ("v4", V4 / "noise", "servo", "servo baseline")]
    fig, ax = chart(t, 8.4, 3.1, "What BC still asked for at the end: success alone hid it",
                    "noise condition, 30 trials per row. Each dot: the smallest speed the controller requested "
                    "in the trial's last second", left=2.75, right=0.95)
    legend(fig, t, [("success", t["ink2"])], 0.2)
    fig.text(1.35 / 8.4, fig.legend_y, "○", color=t["ink2"], fontsize=10.5, va="center")
    fig.text(1.52 / 8.4, fig.legend_y, "fail", color=t["ink2"], fontsize=9.5, va="center")
    rng = random.Random(7)
    labels = []
    for i, (version, folder, method, label) in enumerate(sets):
        rows = [r for r in read_csv(folder / "episodes_gt.csv") if r["method"] == method]
        colour = t["blue"] if method == "servo" else t["orange"]
        y = -i
        for row in rows:
            v = last_second_request(row)
            jitter = rng.uniform(-0.22, 0.22)
            success = row["outcome"] == "success"
            ax.plot([v], [y + jitter], "o", markersize=5.2, zorder=3,
                    color=colour if success else t["surface"], markeredgecolor=colour, markeredgewidth=1.1,
                    alpha=0.9)
        k = sum(r["outcome"] == "success" for r in rows)
        ax.text(0.0362, y, f"{k}/{len(rows)}", color=t["ink"], fontsize=9.5, va="center", fontweight="bold")
        labels.append(f"{version} · {label}")
    for limit, text, dy in ((STILL_V["v2-v3"], "counted as still in v2–v3", 0.0),
                            (STILL_V["v4"], "from v4", 0.0)):
        ax.axvline(limit, color=t["critical"], linewidth=1.2, linestyle=(0, (4, 2)), zorder=1)
        ax.text(limit + 0.0004, 0.62 + dy, f"≤ {limit:g} m/s {text}", color=t["critical"], fontsize=8.6,
                va="center")
    ax.set_yticks([-i for i in range(len(sets))], labels, color=t["ink"], fontsize=9.8)
    ax.set_ylim(-len(sets) + 0.5, 0.9)
    ax.set_xlim(-0.001, 0.035)
    ax.set_xticks([0, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03], ["0", "0.005", "0.01", "0.015", "0.02", "0.025",
                                                                "0.03 m/s"])
    return fig


# ---------------------------------------------------------------------------------------------- top views


def poses_world(name: str) -> list:
    return [(p.pose_id, sw.start_pose_to_world(p)) for p in sw.load_start_poses(str(CONFIG / name))]


def floor(ax, t: dict, xlim=(-0.75, 1.9), ylim=(-1.05, 1.05)):
    """Top view of the simulated room around the box: x towards the box, y to the robot's left."""
    world = sw.WorldConfig()
    ax.set_facecolor(t["surface"])
    ax.set_aspect("equal")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.axis("off")
    ax.add_patch(Rectangle((xlim[0], ylim[0]), xlim[1] - xlim[0], ylim[1] - ylim[0], facecolor=t["plane"],
                           edgecolor="none", zorder=0))
    face = world.box_x
    ax.add_patch(Rectangle((face - BAND[1], -0.36), BAND[1] - BAND[0], 0.72,
                           facecolor=blend(t["aqua"], t["plane"], 0.28), edgecolor="none", zorder=1))
    ax.text(face - (BAND[0] + BAND[1]) / 2, -0.41, "docking band (lidar)", color=t["ink2"], fontsize=8.2,
            ha="center", va="top", rotation=90)
    ax.add_patch(Polygon(sw.box_corners(world), closed=True, facecolor=blend(t["yellow"], t["plane"], 0.55),
                         edgecolor=t["ink2"], linewidth=1.0, zorder=2))
    ax.plot([face, face], [-world.marker_side / 2, world.marker_side / 2], color=t["ink"], linewidth=4.5,
            solid_capstyle="butt", zorder=3)
    ax.text(face + world.box_depth / 2, -world.box_width / 2 - 0.05, "box with\nmarker 7", color=t["ink"],
            fontsize=8.6, ha="center", va="top", zorder=4)


def pose_arrow(ax, pose, colour, length=0.15, width=0.025, zorder=4, alpha=1.0):
    dx, dy = length * math.cos(pose.yaw), length * math.sin(pose.yaw)
    ax.arrow(pose.x, pose.y, dx, dy, width=width * 0.3, head_width=width * 2.2, head_length=width * 2.2,
             length_includes_head=True, color=colour, zorder=zorder, alpha=alpha, linewidth=0)


def poses(t: dict):
    fig = plt.figure(figsize=(8.4, 6.0), facecolor=t["surface"])
    ax = fig.add_axes((0.02, 0.02, 0.96, 0.8))
    floor(ax, t, xlim=(-0.95, 1.9), ylim=(-1.12, 1.12))
    fig.text(0.2 / 8.4, 1 - 0.22 / 6.0, "Where trials start: test poses are held out of training",
             color=t["ink"], fontsize=12.5, fontweight="bold", va="top")
    fig.text(0.2 / 8.4, 1 - 0.5 / 6.0, "Top view of the simulated room (m). Arrows: where the robot starts "
             "and faces. Demonstrations use only the training spots", color=t["ink2"], fontsize=9.5, va="top")
    fig.legend_y = 1 - 0.86 / 6.0
    legend(fig, t, [("10 test poses", t["blue"]), ("20 training spots", t["muted"]),
                    ("10 unseen poses (wide)", t["orange"])], 0.2, marker="s")
    for pid, pose in poses_world("sim_demo_poses.yaml")[10:]:
        pose_arrow(ax, pose, t["muted"], alpha=0.85)
    for pid, pose in poses_world("sim_eval_wide_poses.yaml"):
        pose_arrow(ax, pose, t["orange"])
    for pid, pose in poses_world("start_poses.yaml"):
        pose_arrow(ax, pose, t["blue"], length=0.17, width=0.03, zorder=5)
        ax.text(pose.x - 0.07 * math.cos(pose.yaw), pose.y - 0.07 * math.sin(pose.yaw), str(pid),
                color=t["blue"], fontsize=9.5, fontweight="bold", ha="center", va="center", zorder=6,
                path_effects=[patheffects.withStroke(linewidth=3, foreground=t["plane"])])
    return fig


def trajectories(t: dict):
    rows = read_csv(SHOWCASE / "trajectories.csv")
    episodes = {(int(r["pose_id"]), r["outcome"], float(r["duration_s"])) for r in read_csv(SHOWCASE / "episodes.csv")}
    fig = plt.figure(figsize=(8.4, 5.6), facecolor=t["surface"])
    ax = fig.add_axes((0.02, 0.02, 0.96, 0.8))
    floor(ax, t, xlim=(-0.38, 1.9), ylim=(-0.72, 0.72))
    k = sum(o == "success" for _, o, _ in episodes)
    fig.text(0.2 / 8.4, 1 - 0.22 / 5.6, f"The servo baseline from the 10 test poses ({k}/{len(episodes)} docked)",
             color=t["ink"], fontsize=12.5, fontweight="bold", va="top")
    fig.text(0.2 / 8.4, 1 - 0.5 / 5.6, "Simulation, ground truth from fake_robot, one episode per pose: "
             "it turns to find the marker, then drives in and stops in the band", color=t["ink2"], fontsize=9.5,
             va="top")
    fig.legend_y = 1 - 0.86 / 5.6
    legend(fig, t, [("path", t["blue"]), ("start", t["ink2"]), ("end", t["aqua"])], 0.2, marker="s")
    starts = dict(poses_world("start_poses.yaml"))
    by_pose: dict[int, list] = {}
    for r in rows:
        by_pose.setdefault(int(r["pose_id"]), []).append((float(r["x"]), float(r["y"]), float(r["yaw"])))
    for pid, path in sorted(by_pose.items()):
        xs, ys = [p[0] for p in path], [p[1] for p in path]
        ax.plot(xs, ys, color=t["blue"], linewidth=1.6, alpha=0.85, zorder=4, solid_capstyle="round")
        start = starts[pid]
        pose_arrow(ax, start, t["ink2"], length=0.13, width=0.024, zorder=5)
        ax.text(start.x - 0.06 * math.cos(start.yaw), start.y - 0.06 * math.sin(start.yaw), str(pid),
                color=t["ink"], fontsize=9.2, fontweight="bold", ha="center", va="center", zorder=6)
        end = sw.Pose2D(*path[-1])
        pose_arrow(ax, end, t["aqua"], length=0.07, width=0.018, zorder=6)
    return fig


# ---------------------------------------------------------------------------------------------- rasters


def start_views(path: Path) -> None:
    """The robot camera's first frame at each test pose, rendered by the simulator (80 deg lens)."""
    sim = sw.SimWorld()
    tiles = []
    for pid, pose in poses_world("start_poses.yaml"):
        sim.reset(pose)
        frame = cv2.resize(sim.render(), (320, 240), interpolation=cv2.INTER_AREA)
        frame = _label(frame, f"pose {pid}")
        tiles.append(cv2.copyMakeBorder(frame, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=(251, 252, 252)))
    grid = np.vstack([np.hstack(tiles[:5]), np.hstack(tiles[5:])])
    cv2.imwrite(str(path), grid, [cv2.IMWRITE_JPEG_QUALITY, 88])


def _label(bgr: np.ndarray, text: str) -> np.ndarray:
    from PIL import Image, ImageDraw, ImageFont

    img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    font = ImageFont.truetype("NotoSans-Bold.ttf", 15) if _has_font() else ImageFont.load_default()
    w = int(draw.textlength(text, font=font))
    draw.rounded_rectangle((6, 6, 6 + w + 16, 30), radius=6, fill=(20, 20, 20))
    draw.text((14, 8), text, font=font, fill=(255, 255, 255))
    return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)


# ---------------------------------------------------------------------------------------------- the bag


def episodes_from(data: dict) -> list[dict]:
    """Start and end times of each episode, from /episode/event."""
    episodes, current = [], None
    for t_ns, text in data["events"]:
        words = text.split()
        if words[0] == "start":
            current = {"pose_id": int(words[2]), "round": int(words[3]), "start": t_ns}
        elif words[0] == "end" and current is not None:
            current.update(end=t_ns, outcome=words[1], reason=" ".join(words[2:]))
            episodes.append(current)
            current = None
    return episodes


def refresh_showcase(bag: Path) -> None:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    starts = dict(poses_world("start_poses.yaml"))
    events, gt, status = [], [], []
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        topics = {"/fake_robot/ground_truth": gt, "/dock_judge/status": status, "/episode/event": events}
        connections = [c for c in reader.connections if c.topic in topics]
        for conn, t_ns, raw in reader.messages(connections=connections):
            topics[conn.topic].append((t_ns, reader.deserialize(raw, conn.msgtype).data))
    episodes = episodes_from({"events": events})
    SHOWCASE.mkdir(parents=True, exist_ok=True)
    with open(SHOWCASE / "trajectories.csv", "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["pose_id", "t_s", "x", "y", "yaw"])
        for ep in episodes:
            start = starts[ep["pose_id"]]
            at_start = False
            for t_ns, text in gt:
                if not ep["start"] <= t_ns <= ep["end"]:
                    continue
                g = json.loads(text)
                # fake_robot teleports on the start event: skip samples from before it arrived
                at_start = at_start or math.hypot(g["x"] - start.x, g["y"] - start.y) < 0.05
                if at_start:
                    out.writerow([ep["pose_id"], f"{(t_ns - ep['start']) * 1e-9:.2f}", f"{g['x']:.4f}",
                                  f"{g['y']:.4f}", f"{g['yaw']:.4f}"])
    with open(SHOWCASE / "episodes.csv", "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["pose_id", "round", "outcome", "reason", "duration_s"])
        for ep in episodes:
            out.writerow([ep["pose_id"], ep["round"], ep["outcome"], ep["reason"],
                          f"{(ep['end'] - ep['start']) * 1e-9:.2f}"])
    make_gif(bag, episodes, gt, status)


def make_gif(bag: Path, episodes: list, gt: list, status: list, step: int = 2) -> None:
    """docs/media/docking.gif: the robot camera with what the judge sees, next to the top view."""
    from PIL import Image, ImageDraw, ImageFont
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    ep = next(e for e in episodes if e["pose_id"] == GIF_POSE)
    t0, t1 = ep["start"], ep["end"] + int(0.6e9)
    font = ImageFont.truetype("NotoSans-Regular.ttf", 15) if _has_font() else ImageFont.load_default()
    bold = ImageFont.truetype("NotoSans-Bold.ttf", 15) if _has_font() else font
    small = ImageFont.truetype("NotoSans-Regular.ttf", 12) if _has_font() else font
    detector = MarkerDetector()
    starts = dict(poses_world("start_poses.yaml"))
    world = sw.WorldConfig()
    gt_ep = [(t, json.loads(s)) for t, s in gt if t0 <= t <= t1]
    st_ep = [(t, json.loads(s)) for t, s in status if t0 <= t <= t1]
    frames = []
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conns = [c for c in reader.connections if c.topic == "/camera/image_raw/compressed"]
        for i, (conn, t_ns, raw) in enumerate(reader.messages(connections=conns, start=t0, stop=t1)):
            if i % step:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            frames.append((t_ns, bgr))
    W, H, CAM_W, CAM_H, MAP = 760, 352, 416, 312, 300
    images = []
    trail: list[tuple[float, float]] = []
    for t_ns, bgr in frames:
        g = _latest(gt_ep, t_ns)
        s = _latest(st_ep, t_ns) or {}
        if g:
            trail.append((g["x"], g["y"]))
        cam = cv2.resize(bgr, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)
        sx = CAM_W / bgr.shape[1]
        for det in detector.detect(bgr):
            if det.marker_id == world.marker_id:
                cv2.polylines(cam, [np.int32(det.corners * sx)], True, (54, 175, 27), 3, cv2.LINE_AA)
        canvas_img = Image.new("RGB", (W, H), (252, 252, 251))
        canvas_img.paste(Image.fromarray(cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)), (12, 28))
        draw = ImageDraw.Draw(canvas_img)
        elapsed = (t_ns - t0) * 1e-9
        draw.text((12, 4), f"Simulation · servo baseline · start pose {GIF_POSE} · real time", font=bold,
                  fill=(11, 11, 11))
        draw.text((W - 12, 4), f"t = {elapsed:4.1f} s", font=font, fill=(82, 81, 78), anchor="ra")
        _hud(draw, s, 20, 36, font, small)
        _top_view(draw, canvas_img, (12 + CAM_W + 12, 28), MAP, CAM_H, g, trail, starts, world, small)
        images.append(canvas_img)
    palette = images[len(images) // 2].quantize(colors=96, method=Image.Quantize.MEDIANCUT)
    quant = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in images]
    period_ms = int(round(1000 * step / 15.0))
    quant[0].save(MEDIA / "docking.gif", save_all=True, append_images=quant[1:], duration=period_ms, loop=0,
                  optimize=True)
    print(f"docs/media/docking.gif: {len(quant)} frames, {(MEDIA / 'docking.gif').stat().st_size / 1e6:.1f} MB")


def _has_font() -> bool:
    try:
        from PIL import ImageFont

        ImageFont.truetype("NotoSans-Regular.ttf", 10)
        return True
    except OSError:
        return False


def _latest(samples: list, t_ns: int):
    best = None
    for t, value in samples:
        if t > t_ns:
            break
        best = value
    return best


def _hud(draw, s: dict, x: int, y: int, font, small) -> None:
    """What dock_judge says about this moment: the marker, the range and the three conditions."""
    rng = s.get("front_range_m")
    brg = s.get("bearing_deg")
    lines = [f"lidar range  {rng:.2f} m" if rng is not None else "lidar range  —",
             f"marker bearing  {brg:+.1f}°" if brg is not None else "marker  not in view"]
    draw.rounded_rectangle((x - 6, y - 4, x + 188, y + 44), radius=6, fill=(20, 20, 20))
    for i, text in enumerate(lines):
        draw.text((x, y + i * 20), text, font=small, fill=(240, 239, 236))
    chips = [("in band", s.get("in_band")), ("aligned", s.get("aligned")), ("still", s.get("stopped"))]
    cx = x - 6
    for label, ok in chips:
        w = int(draw.textlength(label, font=small)) + 16
        colour = (27, 175, 122) if ok else (90, 90, 88)
        draw.rounded_rectangle((cx, y + 52, cx + w, y + 72), radius=9, fill=colour)
        draw.text((cx + 8, y + 54), label, font=small, fill=(255, 255, 255))
        cx += w + 6
    if s.get("success"):
        draw.rounded_rectangle((x - 6, y + 80, x + 98, y + 102), radius=9, fill=(42, 120, 214))
        draw.text((x + 4, y + 82), "success", font=small, fill=(255, 255, 255))


def _top_view(draw, img, origin, size, height, g, trail, starts, world, small) -> None:
    x0, y0 = origin
    xlim, ylim = (-0.35, 1.86), (-0.95, 0.95)
    scale = min(size / (xlim[1] - xlim[0]), height / (ylim[1] - ylim[0]))

    def px(x, y):  # world -> image; the box is to the right, the robot's left is up
        return (x0 + (x - xlim[0]) * scale, y0 + (ylim[1] - y) * scale)

    draw.rectangle((x0, y0, x0 + size, y0 + height), fill=(243, 242, 239))
    face = world.box_x
    draw.rectangle((*px(face - BAND[1], 0.36), *px(face - BAND[0], -0.36)), fill=(205, 233, 220))
    box = [px(*c) for c in sw.box_corners(world)]
    draw.polygon(box, fill=(240, 205, 120), outline=(82, 81, 78))
    draw.line((*px(face, -0.05), *px(face, 0.05)), fill=(11, 11, 11), width=5)
    for pose in starts.values():
        tip = (pose.x + 0.09 * math.cos(pose.yaw), pose.y + 0.09 * math.sin(pose.yaw))
        draw.line((*px(pose.x, pose.y), *px(*tip)), fill=(195, 194, 183), width=2)
    if len(trail) > 1:
        draw.line([px(x, y) for x, y in trail], fill=(42, 120, 214), width=3, joint="curve")
    if g:
        c, s_ = math.cos(g["yaw"]), math.sin(g["yaw"])
        hl, hw = 0.097, 0.089
        corners = [(g["x"] + c * a - s_ * b, g["y"] + s_ * a + c * b) for a, b in ((hl, hw), (hl, -hw), (-hl, -hw),
                                                                                  (-hl, hw))]
        half = math.radians(40)
        fov = [(g["x"], g["y"])] + [(g["x"] + 0.55 * math.cos(g["yaw"] + a), g["y"] + 0.55 * math.sin(g["yaw"] + a))
                                    for a in (half, -half)]
        draw.polygon([px(*p) for p in fov], fill=(222, 233, 247))
        draw.polygon([px(*p) for p in corners], fill=(42, 120, 214), outline=(11, 11, 11))
        draw.line((*px(g["x"], g["y"]), *px(g["x"] + 0.12 * c, g["y"] + 0.12 * s_)), fill=(255, 255, 255), width=2)
    draw.text((x0 + 8, y0 + height - 22), "top view, ground truth", font=small, fill=(82, 81, 78))


# ---------------------------------------------------------------------------------------------- main

FIGS: dict[str, Callable] = {"overview": overview, "architecture": architecture, "results": results,
                             "stopping": stopping, "poses": poses, "trajectories": trajectories}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--png", type=Path, help="also write PNG previews here")
    parser.add_argument("--only", help="comma-separated figure names (default: all)")
    parser.add_argument("--bag", type=Path, help="a sim_e2e.sh servo bag: refresh results/sim_showcase/ and the GIF")
    args = parser.parse_args()
    style()
    FIGURES.mkdir(parents=True, exist_ok=True)
    MEDIA.mkdir(parents=True, exist_ok=True)
    if args.bag:
        refresh_showcase(args.bag)
    names = args.only.split(",") if args.only else list(FIGS) + ["start_views"]
    for name in names:
        if name == "start_views":
            start_views(MEDIA / "start_views.jpg")
            print("docs/media/start_views.jpg")
            continue
        for theme, suffix in (("light", ""), ("dark", "-dark")):
            fig = FIGS[name](THEMES[theme])
            fig.savefig(FIGURES / f"{name}{suffix}.svg", facecolor=fig.get_facecolor(), metadata={"Date": None})
            if args.png:
                args.png.mkdir(parents=True, exist_ok=True)
                fig.savefig(args.png / f"{name}{suffix}.png", dpi=110, facecolor=fig.get_facecolor())
            plt.close(fig)
        print(f"docs/figures/{name}.svg (+ -dark)")


if __name__ == "__main__":
    main()
