"""Global view node: RGBD images -> coloured points in hand_root, with axes and a name at every object.

    ros2 launch cube_perception perception.launch.py     # terminal 1: camera + poses
    ros2 launch cube_perception global_view.launch.py    # terminal 2: this node + RViz

Every colour image, with the depth image, camera_info and poses that have the same stamp:
  1. depth pixel -> point in the camera frame (camera_info) -> point in hand_root (/tf);
  2. publish the points with their colour;
  3. publish axes (centre and orientation) and a name for every object in /object_tracker/poses.

Publishes
  /object_tracker/points    sensor_msgs/PointCloud2          frame hand_root, metres
  /object_tracker/markers   visualization_msgs/MarkerArray   axes: x red, y green, z blue
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from message_filters import Subscriber, TimeSynchronizer
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from cube_perception.image_utils import image_to_bgr

FIELDS = [PointField(name=name, offset=4 * i, datatype=datatype, count=1) for i, (name, datatype) in enumerate([
    ("x", PointField.FLOAT32), ("y", PointField.FLOAT32), ("z", PointField.FLOAT32), ("rgb", PointField.UINT32)])]

RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
BLUE = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def cloud_msg(header, color, depth, info, tf, stride, max_depth_m):
    """RGB image + depth image (mm, aligned to the colour pixels) -> PointCloud2 in the frame of header."""
    bgr = image_to_bgr(color)[::stride, ::stride].astype(np.uint32)
    rgb = bgr[..., 2] << 16 | bgr[..., 1] << 8 | bgr[..., 0]
    rows = np.frombuffer(depth.data, np.uint16).reshape(depth.height, depth.step // 2)
    z = rows[:depth.height:stride, :depth.width:stride].astype(np.float32) / 1000.0
    seen = (z > 0) & (z < max_depth_m)   # 0 = no depth
    v, u = np.nonzero(seen)
    z = z[seen]

    fx, cx, fy, cy = info.k[0], info.k[2], info.k[4], info.k[5]
    x, y = (u * stride - cx) / fx * z, (v * stride - cy) / fy * z
    q, t = tf.transform.rotation, tf.transform.translation
    rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    cloud = np.empty(len(z), dtype=point_cloud2.dtype_from_fields(FIELDS))
    # Not a matrix product: numpy starts many threads for it, and that slows down cube_pose_node.
    for axis, row, offset in zip("xyz", rot, [t.x, t.y, t.z]):
        cloud[axis] = row[0] * x + row[1] * y + row[2] * z + offset
    cloud["rgb"] = rgb[seen]
    return point_cloud2.create_cloud(header, FIELDS, cloud)


def object_markers(det):
    """One detection -> axes at its centre and its name with the position."""
    common = dict(header=det.header, ns=det.id,
                  lifetime=Duration(seconds=0.3).to_msg())   # a lost object disappears after this time
    pose = det.results[0].pose.pose
    axes = Marker(id=0, type=Marker.LINE_LIST, pose=pose, **common)
    axes.scale.x = 0.003   # line width
    axes.points = [Point(), Point(x=0.05), Point(), Point(y=0.05), Point(), Point(z=0.05)]   # 5 cm
    axes.colors = [RED, RED, GREEN, GREEN, BLUE, BLUE]

    text = Marker(id=1, type=Marker.TEXT_VIEW_FACING, color=WHITE, **common)
    p = pose.position
    # No spaces or line breaks: RViz then draws only one of the words.
    text.text = f"{det.id}:x={p.x * 1000:.0f},y={p.y * 1000:.0f},z={p.z * 1000:.0f}mm"
    text.scale.z = 0.012   # letter height
    text.pose.position = Point(x=p.x, y=p.y, z=p.z + 0.07)   # above the axes, however they are turned
    return [axes, text]


class GlobalViewNode(Node):
    def __init__(self):
        super().__init__("global_view_node")
        p = self.declare_parameters("", [
            ("color_topic", "/camera/camera/color/image_raw"),
            ("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"),
            ("info_topic", "/camera/camera/aligned_depth_to_color/camera_info"),
            ("poses_topic", "/object_tracker/poses"),
            ("stride", 2),            # use every 2nd pixel in each direction (1 = all pixels)
            ("max_depth_m", 1.5),     # leave out points further from the camera
        ])
        g = {param.name: param.value for param in p}
        self.stride = g["stride"]
        self.max_depth_m = g["max_depth_m"]

        self.points_pub = self.create_publisher(PointCloud2, "/object_tracker/points", 1)
        self.markers_pub = self.create_publisher(MarkerArray, "/object_tracker/markers", 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Exact stamps: the driver gives aligned depth the stamp of its colour image, and the poses keep it.
        self.sync = TimeSynchronizer([
            Subscriber(self, Image, g["color_topic"], qos_profile=qos_profile_sensor_data),
            Subscriber(self, Image, g["depth_topic"], qos_profile=qos_profile_sensor_data),
            Subscriber(self, CameraInfo, g["info_topic"], qos_profile=qos_profile_sensor_data),
            Subscriber(self, Detection3DArray, g["poses_topic"], qos_profile=10)], 10)
        self.sync.registerCallback(self.on_frame)
        self.frames = 0
        self.create_timer(5.0, self.check_frames)

    def check_frames(self):
        if self.frames == 0:
            self.get_logger().warn("no frame for 5 s: colour, depth, camera_info or poses are missing")
        self.frames = 0

    def on_frame(self, color, depth, info, poses):
        self.frames += 1
        try:
            # Newest transform: the one of this image may still be on its way, and the hand base doesn't move.
            tf = self.tf_buffer.lookup_transform(poses.header.frame_id, color.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f"no hand_root yet, show the forearm tag: {e}", throttle_duration_sec=5.0)
            return
        self.points_pub.publish(cloud_msg(poses.header, color, depth, info, tf, self.stride, self.max_depth_m))
        self.markers_pub.publish(MarkerArray(markers=[m for det in poses.detections for m in object_markers(det)]))


def main(args=None):
    rclpy.init(args=args)
    node = GlobalViewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise   # otherwise: a publish failed because Ctrl-C/SIGTERM shut ROS down mid-callback
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
