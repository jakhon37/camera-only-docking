# Getting started

Everything ROS runs in Docker. Training and analysis need a Python 3.12 environment with
[requirements-tools.txt](../requirements-tools.txt) (LeRobot 0.6.1, PyTorch, ONNX Runtime, rosbags).

## Simulation

Needs Docker and, for training, a Python 3.12 environment with `requirements-tools.txt`.

```bash
bash scripts/sim_e2e.sh servo                                   # servo baseline, 10 poses, ~5 min
PYTHON=.venv/bin/python bash scripts/sim_e2e.sh                 # everything below, ~50 min on 4 CPU cores
PYTHON=.venv/bin/python bash scripts/sim_eval.sh                # harder v2 evaluation of the trained model, ~35 min per condition
PYTHON=.venv/bin/python bash scripts/sim_eval.sh --work sim_work/v2 --results results/sim_eval_v3_bcfix \
  --methods bc_v2 --pair-with results/sim_eval_v2 clean_ref noise wide hfov62   # one new arm, paired with earlier rows
```

`sim_e2e.sh` starts a clean `ros:jazzy-ros-base` container (`--distro foxy|humble` also work),
builds the workspace and runs, in order:

1. **servo**: the baseline on the 10 test poses, episodes started with the `episode` CLI and
   ended by `dock_judge`, rows written by `trial_logger`, everything recorded with `ros2 bag`;
2. **demos**: scripted demonstrations from 20 *training* spots (`config/sim_demo_poses.yaml`,
   at least 10 cm from every test pose) with the servo under the method name `servo_demo`,
   two gain settings, and fake_robot's wheel slip, start jitter and image noise switched on;
3. **train** (host): `bag_to_lerobot.py` → `train_bc.py` (ResNet18 **from scratch**, CPU,
   `servo_demo` successes only, validation on held-out spots) → `export_onnx.py`;
4. **bc**: `policy_runner` with ONNX Runtime (CPU) on the 10 test poses;
5. **report**: `analyze_trials.py` → `results/sim_e2e/<distro>/report.md` and `success.png`.

By hand, inside a ROS 2 container with the workspace built:

```bash
ros2 launch jbl_nodes sim_dock.launch.py trials_csv:=/tmp/trials.csv          # terminal 1
ros2 run jbl_nodes episode --timeout 5 start servo 3 1                         # terminal 2
tail -f /tmp/trials.csv                                                        # dock_judge ends it
```

## Run the tests

```bash
bash scripts/docker_test.sh foxy humble jazzy        # colcon build + test of both packages per distro
HF_HUB_OFFLINE=1 python -m pytest tests ros2_ws/src/jbl_nodes/test -q   # host tools + pure-Python node logic
IMAGE=auto_ros:humble bash scripts/docker_test.sh humble   # on the Jetson host: the robot's own image
```

The ROS tests include gtest for the gate logic, launch tests for the gate, nodes on a live graph
(camera, policy runner, judge, servo, logger, simulator), and the three launch files (a servo
episode end to end with a bag, a constant policy through the gate, the simulated policy's
`sim.yaml` settings, calibration overrides, argument errors). The host suite covers bag
conversion (synthetic Foxy-layout and Jazzy bags), BC training and ONNX export parity, the
statistics against statsmodels/scipy, and the analysis.

Last run (October 2026):

- **Containers.** 280 tests on each of Foxy, Humble and Jazzy (with the v4 stop rules), with 0
  failures and 8 skips. The skipped tests are the ONNX Runtime and TorchScript backend checks;
  the base images have neither package.
- **Host.** 385 passed and 5 skipped. The skipped tests need rclpy or `sensor_msgs`.

On the robot's Jetson Xavier NX (November 2026, arm64):

- **The robot's own image** (`auto_ros:humble`). 303 tests, 0 failures and the same 8 skips.
- **The official Jazzy and Foxy images.** The same. Foxy's first full run had 1 failure, when its
  `ros2 launch` took the SIGINT in the middle of a log flush. That test then passed 10 of 10, 5 on
  the code before these changes and 5 after, and a second full run passed.
- **On a PC, without ROS.** The node tests in `ros2_ws/src/jbl_nodes/test`: 202 passed and 13
  skipped. The rest of the host suite was not rerun; in `tools/`, only a comment changed.
- **With the real camera, by hand.** `camera_check`, and `record_robot.sh` through to a closed bag.
  A simulator bag from that image was read with `bag_to_lerobot.py`'s reader, but not converted
  to a dataset.

## On the real robot

[docs/PROTOCOL.md](PROTOCOL.md) is the runbook: marker and box, floor layout of the 10
start poses, lidar `forward_angle` and camera HFOV calibration, teleop demonstrations, freezing
models before trials, the counterbalanced schedule, interventions and aborts, video rules,
analysis and honest reporting.

Build inside the robot's Humble container (autoJetsonBot's `./robot.sh shell`; clone this repo
into the autoJetsonBot checkout so the container sees it under `/autonomous_ROS/`. This repo's
`COLCON_IGNORE` keeps `./robot.sh build` out of it). Clear the blockers first (PROTOCOL, "Before
anything drives"). Then:

```bash
cd /autonomous_ROS/camera-only-docking/ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 run jbl_nodes camera_check    # must print PASS before any session (PROTOCOL section 4)
bash ../scripts/record_robot.sh    # reads one frame, then camera, gate, judge, logger + bag in data/robot/
```

Recordings go to `data/robot/` in this checkout, which is on the `/autonomous_ROS` bind mount
(`/root` in the container is not persistent). Teleop must publish `/cmd_vel_in`: the web
dashboard with its Command Velocity topic set to `/cmd_vel_in` (it publishes continuously while
a key is held), a joystick, or `teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_in` with a
fast key repeat (PROTOCOL section 6).

Nothing but `safety_gate` may publish `/cmd_vel`: `diffdrive_node` has no command watchdog, so
any other publisher bypasses the gate. The gate publishes a zero twist on SIGINT, SIGTERM,
SIGHUP and SIGQUIT, and the launch files respawn it if it dies; a crash or SIGKILL of the gate
still leaves the motors on their last command until it is back (about 3 s on Foxy because of
DDS rediscovery). The ESP32 firmware's 500 ms `m` command timeout is what actually bounds
this, so verify it (PROTOCOL section 4) before anything drives on its own.

## Training

Copy the recordings off the robot first (the container writes them world-readable):

```bash
rsync -av <jetson>:<autoJetsonBot checkout>/camera-only-docking/data/robot/ data/robot/
```

```bash
python tools/bag_to_lerobot.py data/robot/bags --root data/jbl_dock --repo-id <you>/jbl_dock   # Foxy .db3 or Jazzy .mcap
python tools/train_bc.py --root data/jbl_dock --out runs/bc --pretrained --methods teleop --outcomes success --split-by pose_id
python tools/export_onnx.py runs/bc/model.pt --dataset-root data/jbl_dock                 # -> runs/bc/policy.onnx + policy_meta.json
EPISODES=$(python -c 'import json; print(json.load(open("runs/bc/split.json"))["train"])') \
  DATASET_ROOT=data/jbl_dock bash tools/train_act.sh                                      # ACT via lerobot-train, BC's training episodes
```

`train_act.sh` without `EPISODES` selects the same way (`METHODS=teleop`, `OUTCOMES=success`,
split by pose_id, validation fraction 0.2, seed 0); `EPISODES=all` trains on everything,
including failed demos and trial rollouts. `bag_to_lerobot.py` drops each episode's idle start
(the zero commands before the operator's first one; `--keep-idle-start` keeps them).
`train_bc.py --stop-weight W` weights full-stop frames (|v| < 0.005 m/s and |w| < 0.05 rad/s)
W times in the loss. Check how many stop examples a dataset has, and where they are, with
`scripts/stop_labels.py dataset` before training (results/sim_eval_v3_bcfix shows why).

The dataset is LeRobot v3 at 10 fps: `observation.images.front` (240×320 RGB from the JPEG
stream), `observation.state` (odometry v, w), `action` (the `/cmd_vel_in` command), plus
`meta/jbl_episodes.jsonl` (method, pose, round, outcome per episode). Training and the robot
share one preprocessing module (`jbl_nodes/preprocess.py`), and `export_onnx.py` refuses to
write a model whose ONNX Runtime output differs from PyTorch by more than 1e-4 on real
dataset frames. ACT trains with `tools/train_act.sh` but cannot be deployed yet: it outputs a
chunk of actions and also takes odometry, while the robot's model contract is image in, one
normalised (v, w) out (see [Limitations](EXPERIMENTS.md#limitations)).

## Deployment

The policy is an ONNX file with `policy_meta.json` beside it; `policy_runner` checks the meta
(input size, normalisation, action scale) before it loads the model and publishes
`/cmd_vel_in` at 10 Hz, or zero when the newest frame is older than 0.2 s.

**Off-board (laptop over Wi-Fi, ONNX Runtime).** The robot runs `robot_record.launch.py`
(camera, gate, judge, logger); the laptop, on the same `ROS_DOMAIN_ID`, runs

```bash
ros2 run jbl_nodes policy_runner --ros-args -p backend:=onnxruntime \
  -p model_path:=runs/bc/policy.onnx -p method_filter:=bc
```

Frames are aged by their capture stamp, so the two clocks must agree (chrony, offset < 5 ms)
or the runner sees stale frames and publishes zeros. Wi-Fi latency and dropped JPEGs count
against the policy; label such results "off-board". ONNX Runtime comes from PyPI, installed
next to the distro's own numpy, which python3-opencv is built against (`inner_onnxruntime` in
`scripts/sim_e2e.sh`). The simulation runs used:

- **Foxy:** Ubuntu's numpy 1.17 limits ONNX Runtime to 1.10.0, the last release that accepts
  numpy ≥ 1.16.6.
- **Jazzy:** ONNX Runtime 1.31.1.

**On-board (Jetson Xavier NX).** Not set up yet: no policy has run on the robot. JetPack 5.1.6
has TensorRT 8.5.2 and CUDA 11.4, but its Python bindings are for the host's Python 3.8, which
has no ROS, and the Humble container (Python 3.10) has neither TensorRT nor ONNX Runtime. The
choice is still open; ONNX Runtime inside the container is the first thing to try. A second
container for TensorRT needs `--ipc host` on both containers, because the running
`auto_ros_humble` has its own IPC namespace and Fast DDS uses shared memory on one host.
`tools/build_trt_nano.sh` was written for the Nano's TensorRT 8.0/8.2: on 8.5 it needs
`--memPoolSize` for `--workspace`. The TensorRT backend and that script have only run against
stand-ins.

## Redraw the figures

```bash
python tools/make_figures.py                               # docs/figures/*.svg from results/ and config/
python tools/make_figures.py --png /tmp/preview            # also PNGs, to look at them
python tools/make_figures.py --bag sim_work/bags/humble/servo   # also redo results/sim_showcase/ and docs/media/docking.gif
```

The bag comes from `bash scripts/sim_e2e.sh --distro humble servo`; it is not in git, but the
trajectories and episode times taken from it are, in [results/sim_showcase](../results/sim_showcase/).
