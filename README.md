# Sim2realhandnode

ROS 2 Jazzy nodes for Eureka sim2real: nodes that bring the real cube's state into the robot hand's control stack, so a policy trained in
Isaac Lab (`ard-isaaclab-tasks`) can run on the real MASA hand.

```
RealSense D435 ──► camera driver ──► /camera/camera/color/image_raw ──┬──► cube pose node ──► /robot_hand/object/states ──► policy_runner
(realsense2_camera, Intel's)                                          └──► VLM node =
```

| Part | Status |
|---|---|
| **Camera node**: Intel's `realsense2_camera` driver, started with shared settings | ✅ working (`camera.launch.py`) |
| **Cube pose node**: STag markers → cube pose in `hand_root` (twist NaN) | ✅ written, tested on saved frames (`perception.launch.py`) |

## Repository layout

```
cube_perception/                 ament_python package
├── package.xml
├── setup.py, setup.cfg
├── config/
│   ├── camera.yaml              shared camera settings (resolution, exposure, white balance, ...)
│   ├── camera_intrinsics.npz    camera calibration, valid at 1280x720 only
│   └── cube_faces.json          learned marker rotations on each cube face
├── launch/
│   └── camera.launch.py         starts the D435 with camera.yaml
└── cube_perception/             Python code (cube pose node goes here)
```

## Requirements

- Ubuntu 24.04
- ROS 2 **Jazzy**
- Intel RealSense **D435** on a **USB 3** port (serial `826212070191`)
- Python 3.12 (the system Python ROS uses), **not** a conda environment

## Setup (once)

### 1. Keep conda out of ROS terminals

ROS Jazzy is built for the system Python 3.12. Conda's Python (e.g. the Isaac Lab env, Python 3.11)
cannot import `rclpy` and makes `colcon` build with the wrong interpreter.

```bash
conda config --set auto_activate_base false
```

Activate `env_isaaclab` only in terminals where you train. Never in ROS terminals.

### 2. Install ROS 2 Jazzy and the RealSense driver

Follow <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html> (`ros-jazzy-desktop` + `ros-dev-tools`), then:

```bash
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-rqt-image-view
```

### 3. Python packages

numpy, scipy and OpenCV come from apt, so they match the versions ROS is built against (NumPy 1.x):

```bash
sudo apt install python3-numpy python3-scipy python3-opencv
```

`stag-python` is not in apt, so it goes in a venv that can also see the ROS packages:

```bash
python3 -m venv --system-site-packages ~/ros2_venv
```

```bash
~/ros2_venv/bin/pip install stag-python "numpy<2"
```

Keep `numpy<2`: ROS Jazzy's Python packages are built against NumPy 1.x.

### 4. Workspace

The package lives in this repo and is linked into a colcon workspace:

```bash
mkdir -p ~/ros2_ws/src
```

```bash
ln -s ~/Documents/GitHub/Sim2realhandnode/cube_perception ~/ros2_ws/src/cube_perception
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select cube_perception
```

With `--symlink-install`, edits to existing files (e.g. `camera.yaml`) take effect on the next launch without
rebuilding. Rebuild after **adding** files.

## Every new terminal

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && source ~/ros2_venv/bin/activate
```

(Add it to `~/.bashrc` to skip this.)

## Run the camera

```bash
ros2 launch cube_perception camera.launch.py
```

For a different D435:

```bash
ros2 launch cube_perception camera.launch.py serial_no:=123456789012
```

Publishes:

| Topic | Type | |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | 1280×720 RGB8, ~30 Hz |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | factory intrinsics (use `camera_intrinsics.npz` instead) |

Only one program can open the camera at a time. While the driver runs, `cube_tracker.py` and
`validate_stag.py` cannot open it. Stop the driver (Ctrl+C) to use them.

### Check it

Rate (should settle at about 29–30 Hz, with `max` around 0.033–0.045 s):

```bash
ros2 topic hz /camera/camera/color/image_raw
```

Settings actually applied:

```bash
ros2 param dump /camera/camera
```

View the images (pick `/camera/camera/color/image_raw` in the dropdown; press refresh if it is empty):

```bash
ros2 run rqt_image_view rqt_image_view
```

## Camera settings

All settings are in [`cube_perception/config/camera.yaml`](cube_perception/config/camera.yaml).
**The cube pose node and the VLM node share the same images. Agree any change with the teammate first.**

| Setting | Value | Why |
|---|---|---|
| Resolution / rate | 1280×720 @ 30 fps | The calibration is only valid at 1280×720; the D435 colour sensor tops out at 30 fps there |
| Streams | Colour only | Depth and IR are unused and cost USB bandwidth |
| Exposure | Auto, `auto_exposure_priority: false` | With priority on, the camera drops to ~25 Hz in dim light |
| White balance | Auto | STag uses greyscale; mostly matters to the VLM |
| Power line frequency | 50 Hz | Avoids flicker from room lights (NZ mains) |

**Open:** fixed short exposure (e.g. 5 ms) would reduce motion blur on a fast-moving cube, at the cost of a
darker or noisier image. Compare auto and fixed on recordings (`ros2 bag record /camera/camera/color/image_raw`)
before deciding, and agree it with the teammate.

**Policy rate:** the camera should be at least as fast as the policy. The MASA OpenAI tasks run at 20 Hz
(`dt` 1/60 × decimation 3), so 30 Hz is enough. The MASA state task runs at 60 Hz (`dt` 1/120 × decimation 2),
so a new cube pose would arrive only every other step.

### Why not `rs_launch.py`?

Intel's `rs_launch.py` only forwards settings on its own argument list and ignores the rest with a warning.
`rgb_camera.auto_exposure_priority`, for example, would be silently dropped. `camera.launch.py` starts
`realsense2_camera_node` directly, so every setting in `camera.yaml` is applied.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `bash: ros2: No such file or directory` | You typed `bash ros2 ...`. Drop the `bash`. |
| `ros2: command not found` | ROS is not sourced in this terminal. Run the source line above. |
| `The requested device with serial number ... is NOT found` | Wrong serial. The driver's log prints the serial it found. |
| `Node not found` from `ros2 param` | The driver isn't running. `ros2 param` only works on a running node. |
| ~25 Hz instead of 30 Hz | `auto_exposure_priority` is on, or the camera is on a USB 2 port. |
| `symbol lookup error: /snap/core20/...libpthread.so.0` when starting rqt / RViz | Started from the terminal of **snap** VS Code, which passes its library paths to everything it starts. Use Ubuntu's Terminal app (Ctrl+Alt+T), or run `env \| grep -i snap` and `unset` the variables that point into `/snap/`. Installing VS Code from Microsoft's `.deb` fixes it for good. |
| rqt_image_view shows a grey gradient | No topic selected. Press refresh and pick the image topic. |

## Next steps

See `ROS_NODES_PLAN.md` for the full plan:

1. Cube pose node (`tracker_core.py`, `pose_filter.py`, `cube_pose_node.py`) publishing
   `robot_hand_interfaces/LinkStates` on `/robot_hand/object/states` in the `hand_root` frame.
2. `perception.launch.py`: camera + cube pose node together.
3. Build `robot_hand_interfaces` (from `robot-hand-control-stack`) in the same workspace.
4. Record test bags and tune the filter.
