# Published topics

This page is for people who write a program that reads the object poses. It shows which node publishes what, the exact content of each message, and how to read it.

## Layout

```
┌────────────────────────────┐
│ /camera/camera             │  RealSense driver (settings: config/camera.yaml)
└────────────────────────────┘
   │
   ├──► /camera/camera/color/image_raw ───────────────────┐
   ├──► /camera/camera/color/camera_info                  │
   └──► /camera/camera/aligned_depth_to_color/image_raw   │
                                                          ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ /cube_pose_node            (settings: config/perception.yaml)            │
│                                                                          │
│  image ─► find tags ─► pose of each object ─► to hand_root ─► filter     │
└──────────────────────────────────────────────────────────────────────────┘
   │
   ├──► /object_tracker/poses         the poses. Most programs need only this one.
   ├──► /object_tracker/status        one line of text
   ├──► /object_tracker/debug_image   image with axes drawn (off by default)
   └──► /tf                           the same poses, for RViz and tf2
```

`cube_pose_node` reads only the colour image. It does not read depth or `camera_info`.

## Topics

| Topic | Type | When it is sent |
|---|---|---|
| `/object_tracker/poses` | `vision_msgs/msg/Detection3DArray` | Once for every processed image, also when no object is found |
| `/object_tracker/status` | `std_msgs/msg/String` | Once for every processed image |
| `/object_tracker/debug_image` | `sensor_msgs/msg/Image` | Up to 5 times per second, only with `debug_image:=true` |
| `/tf` | `tf2_msgs/msg/TFMessage` | Once for every processed image, after the forearm tag was seen once |

The measured rate is 55 to 58 Hz.

QoS (the delivery rules of a topic): `poses` and `status` are reliable, volatile, depth 10. `debug_image` is reliable, volatile, depth 1. A default subscriber works with all of them.

## `/object_tracker/poses`

### Content

```
Detection3DArray
├── header
│   ├── stamp                    time of the camera image
│   └── frame_id                 "hand_root"
└── detections[]                 one entry for each object that has a pose
    ├── header                   same as above
    ├── id                       object name, for example "cube"
    ├── results[0]               always exactly one
    │   ├── hypothesis
    │   │   ├── class_id         object name, same as id
    │   │   └── score            1.0 or 0.5, see below
    │   └── pose
    │       ├── pose
    │       │   ├── position     x, y, z in metres
    │       │   └── orientation  quaternion x, y, z, w
    │       └── covariance       not filled (all 0)
    └── bbox                     not filled (default values)
```

The pose is the object centre in `hand_root`. The object axes are the axes used in `config/<name>_faces.json`. There is no velocity.

### Score

The score is a flag. It is not a probability.

| Score | Meaning |
|---|---|
| 1.0 | Measured in this image |
| 0.5 | Not measured in this image. This is the last good pose, held for at most 0.2 s. |

### Example

The numbers are only an example. This is the form that `ros2 topic echo` prints. `covariance` and `bbox` are left out here.

```yaml
header:
  stamp: {sec: 1764030000, nanosec: 250000000}
  frame_id: hand_root
detections:
- header:
    stamp: {sec: 1764030000, nanosec: 250000000}
    frame_id: hand_root
  results:
  - hypothesis: {class_id: cube, score: 1.0}
    pose:
      pose:
        position: {x: 0.012, y: 0.095, z: 0.041}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  id: cube
```

The cube centre is 12 mm to the little finger, 95 mm to the fingertips, and 41 mm out of the tag surface. It is not turned relative to `hand_root`.

An empty list means that no object has a pose now:

```yaml
header:
  stamp: {sec: 1764030000, nanosec: 250000000}
  frame_id: hand_root
detections: []
```

## `/object_tracker/status`

One line for all objects: `<object>: <status>`, joined with a comma and a space.

```
cube: tracking
cube: tracking, box: lost
```

| Status | Meaning | In `poses`? | Score |
|---|---|---|---|
| `tracking` | Measured in this image | yes | 1.0 |
| `predicting` | Hidden, or the measurement was ignored as a jump. The last pose is held. | yes | 0.5 |
| `lost` | No good measurement for more than 0.2 s | no | |
| `no_hand_marker` | The forearm tag was never seen, so `hand_root` is not known | no | |

This message has no time stamp. To know if one pose is measured or held, use the score in `poses`.

## `/tf`

```
camera_color_optical_frame
└── hand_root
    └── cube           one frame for each object, with the object name
```

- The parent of `hand_root` is the frame of the camera image.
- An object frame is sent only while the object has a pose. Held poses (score 0.5) are also sent, and `/tf` has no score.
- tf2 keeps old transforms for some time. Check the time stamp, or use `poses`, if you must know that the object is visible now.
- Set `publish_tf: false` to switch this off.

## `/object_tracker/debug_image`

A 640x360 `bgr8` image. It shows the axes of `hand_root` and of each object, and one line of text: the status, the IDs of the visible tags, and the fit error in pixels.

```bash
ros2 launch cube_perception perception.launch.py debug_image:=true
ros2 run rqt_image_view rqt_image_view /object_tracker/debug_image
```

## Camera topics

These come from the RealSense driver.

| Topic | Type | Content |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | Colour image (RGB), 960x540 |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Factory calibration. The tracker uses `camera_intrinsics.npz` instead. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Depth image, matched to the colour pixels |

## How to read the poses

### In the terminal

```bash
ros2 topic echo /object_tracker/poses                       # every message
ros2 topic echo /object_tracker/poses --field detections    # only the objects
ros2 topic echo /object_tracker/status
ros2 topic hz /object_tracker/poses
ros2 run tf2_ros tf2_echo hand_root cube                    # the pose from /tf
```

### In Python

```python
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from vision_msgs.msg import Detection3DArray


class ReadPoses(Node):
    def __init__(self):
        super().__init__("read_poses")
        # Depth 1: keep only the newest message.
        self.create_subscription(Detection3DArray, "/object_tracker/poses", self.on_poses, 1)

    def on_poses(self, msg):
        objects = {d.id: d.results[0] for d in msg.detections}
        if "cube" not in objects:
            return   # lost, or the forearm tag was never seen

        cube = objects["cube"]
        measured_now = cube.hypothesis.score > 0.9   # False: held pose
        p = cube.pose.pose.position                  # metres, in hand_root
        q = cube.pose.pose.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()   # 3x3

        self.get_logger().info(
            f"position ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m, measured now: {measured_now}\n{rotation}")


def main():
    rclpy.init()
    rclpy.spin(ReadPoses())


if __name__ == "__main__":
    main()
```

Your package needs `vision_msgs` in its `package.xml`.

To read the status line:

```python
status = dict(part.split(": ") for part in msg.data.split(", "))
# {"cube": "tracking", "box": "lost"}
```

## Important rules

1. **Find an object by `id`, not by its place in the list.** Lost objects are left out, so the places change.
2. **Positions are in metres.** The files in `config/` use millimetres.
3. **The quaternion order is x, y, z, w.**
4. **Use `header.stamp` for time.** It is the time of the image, about 34 ms before the message is sent. If you compare it with the clock of another computer, the two clocks must be synchronised.
5. **A missing message is not the same as an empty list.** Empty list: the node works, but no object has a pose. No message: the node or the camera stopped.
6. **A sudden large move arrives late.** A move of more than 20 mm or 30° between two good measurements is first ignored. It is accepted when 3 such measurements in a row agree. Until then the old pose is sent with score 0.5.
7. **A hidden forearm tag gives no warning.** `hand_root` is an average over 60 images, and the last average stays in use with no time limit. If the camera moves while the tag is hidden, all poses are wrong until the tag is seen again in 10 images in a row that agree.

## Settings that change the output

All are in `config/perception.yaml`.

| Setting | Default | Effect |
|---|---|---|
| `states_topic` | `/object_tracker/poses` | Name of the pose topic |
| `status_topic` | `/object_tracker/status` | Name of the status topic |
| `objects` | `[cube]` | Objects to track. Each one needs `config/<name>_faces.json`. |
| `frame_id` | `hand_root` | Name of the world frame in `poses` and `/tf` |
| `publish_tf` | `true` | Send `/tf` |
| `debug_image` | `false` | Send the debug image. `perception.launch.py` always replaces this value with its own argument `debug_image:=` (default `false`). |
| `debug_rate_hz` | `5.0` | Maximum rate of the debug image |
| `filter.max_hold_s` | `0.2` | How long a pose is held |
| `filter.max_jump_mm`, `filter.max_jump_deg` | `20.0`, `30.0` | A larger change is first ignored |
| `filter.reset_after` | `3` | Number of agreeing measurements needed to accept it |
