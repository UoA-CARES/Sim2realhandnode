# Introduction and Feature List

This is a computer vision node for ROS 2 Jazzy. It finds the position and orientation of chosen objects, at high speed.

- **Fast.** The target is 60 Hz, the limit of the camera. Measured: 55 to 58 Hz, about 34 ms from image to pose.
- **Fixed world frame (`hand_root`).** The centre of the forearm tag is (0, 0, 0). The upper surface of the dexterous hand stays horizontal. The fingertips point to the +y axis.
  - World frame: the axes that all poses are given in. Origin: forearm tag (ID 10). x: to the little finger. y: to the fingertips. z: out of the tag surface.
- **Tracking by tags.** Objects are tracked with printed markers. STag is the type supported now.
- **Many objects.** One object is a set of tags. See [How an object is described](#how-an-object-is-described).

# Structure

## File structure

```
cube_perception/                ROS package
├── launch/
│   ├── camera.launch.py        camera only
│   └── perception.launch.py    camera + cube pose node
├── config/
│   ├── camera.yaml             camera settings
│   ├── camera_intrinsics.npz   calibration of camera 826212070191, at 1280x720
│   ├── perception.yaml         node settings, and the list of objects
│   └── cube_faces.json         cube, 6 tags
├── cube_perception/
│   ├── image_utils.py          ROS image -> numpy image
│   ├── tracker_core.py         image -> tag corners -> pose of an object
│   ├── pose_filter.py          removes jumps, holds the pose while an object is hidden
│   └── cube_pose_node.py       ROS node: joins the files above
└── tools/                      save_frames, measure_cube, learn_faces, compare_with_cube_tracker
```

## How it works

```
Intel camera driver ──► tag detector ──► object pose ──► filter ──► ROS topics
                        └──────────── cube_pose_node ───────────┘
```

## How an object is described

Each object has one file. The file lists the tags on the object. Each tag has an ID and its pose on the object. All tags of one object have the same size.

```
config/cube_faces.json (one tag shown, shortened)
{
  "marker_mm": 40.0,
  "faces": {
    "6": {
      "R_cube_marker": [[0.000585, 1.0, 0.0], [-1.0, 0.000585, 0.0], [0.0, 0.0, 1.0]],
      "t_cube_marker_mm": [0.043, 0.145, 25.0]
    }
  }
}
```

`R_cube_marker` is the tag rotation (3x3 matrix). `t_cube_marker_mm` is the tag centre. Both are relative to the object centre.

To add an object:

1. Write `config/<name>_faces.json`. One tag ID can belong to one object only.
2. Add the name to `objects:` in `config/perception.yaml`.
3. Build again.

`tools/learn_faces.py` corrects small errors in how the tags were glued on. It works for the cube only.

## Input and output

**Input:** none from other nodes. The launch file starts the camera, and `cube_pose_node` reads `/camera/camera/color/image_raw`.

**Publishes:**

| Topic | Type | Content |
|---|---|---|
| `/object_tracker/poses` | `vision_msgs/Detection3DArray` | Object list. Frame `hand_root`, metres, sent for every image. |
| `/object_tracker/status` | `std_msgs/String` | `<object>: tracking`, `predicting`, `lost` or `no_hand_marker` |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Colour image (RGB), 960x540 |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Factory calibration. The tracker uses `camera_intrinsics.npz` instead. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth image (D), matched to the colour pixels |
| `/object_tracker/debug_image` | `sensor_msgs/Image` | Image with axes drawn. Off by default. |
| `/tf` | `tf2_msgs/TFMessage` | `camera_color_optical_frame -> hand_root -> <object>` |

How to read the object list:

| Field | Meaning |
|---|---|
| `detections[i].id` | Object name, for example `cube` |
| `detections[i].results[0].pose.pose` | Pose |
| `detections[i].results[0].hypothesis.score` | 1.0: seen in this image. 0.5: not seen now, last pose held for a short time. |

A lost object is not in the list. If the forearm tag was never seen, the list is empty.

# Installation

You need:

- Ubuntu 24.04 and [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- The system Python 3.12, **not** conda
- Intel RealSense D435 on a **USB 3** port

1. Keep conda out of ROS terminals. ROS cannot use the conda Python.

   ```bash
   conda config --set auto_activate_base false
   ```

2. Install the system packages. Also install `librealsense2-udev-rules` from the RealSense apt repository. It lets a normal user open the camera.

   ```bash
   sudo apt install ros-jazzy-realsense2-camera ros-jazzy-vision-msgs python3-numpy python3-scipy python3-opencv
   ```

3. Install the Python packages that apt does not have. Keep `numpy<2`, because ROS Jazzy is built with NumPy 1.

   ```bash
   /usr/bin/python3 -m venv --system-site-packages ~/ros2_venv
   ~/ros2_venv/bin/pip install stag-python "numpy<2"
   ```

4. Build. Start in the folder of this repo.

   ```bash
   mkdir -p ~/ros2_ws/src && ln -s "$PWD/cube_perception" ~/ros2_ws/src/
   source /opt/ros/jazzy/setup.bash
   cd ~/ros2_ws && colcon build --symlink-install --packages-select cube_perception
   ```

# How to use it

Run this in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && source ~/ros2_venv/bin/activate
```

| Task | Command |
|---|---|
| Start | `ros2 launch cube_perception perception.launch.py` |
| Start with the debug image | `ros2 launch cube_perception perception.launch.py debug_image:=true` |
| Use another camera | `ros2 launch cube_perception perception.launch.py serial_no:=123456789012` |
| Start the camera only | `ros2 launch cube_perception camera.launch.py` |
| Check the rate (55 to 60 Hz) | `ros2 topic hz /object_tracker/poses` |
| Measure accuracy | `python3 tools/measure_cube.py` |
| Correct the tag poses of the cube | `python3 tools/learn_faces.py` |
| Save test images | `python3 tools/save_frames.py --out ~/frames` |

The tools need the camera running. Run them in the folder `cube_perception`. Another camera needs its own calibration file: set `intrinsics_path` in `config/perception.yaml`.

## Camera mode

| Mode | Rate | Notes |
|---|---|---|
| 960x540 (default) | 60 Hz | The largest colour image the D435 gives at 60 Hz |
| 1280x720 | 30 Hz | Less shaking: rotation 0.23° instead of 0.53° in one test with a still cube |

Change `color_profile` in `config/camera.yaml`. Only 16:9 sizes work. The VLM node uses the same images. Agree with the team before you change the mode.

## Problems

| Problem | Fix |
|---|---|
| `ros2: command not found` | Run the `source` line above. |
| The rate is below 60 Hz | Check that the camera is on USB 3. Check the processing time in the log. |
| The object list is empty | Show the forearm tag to the camera. |
| `symbol lookup error: /snap/...` | Do not use the terminal of the snap version of VS Code. Use the Ubuntu Terminal. |

# Future TODO list

- Change the camera while running, and combine (fuse) the results from several camera angles.
- Support more marker types and tracking methods.