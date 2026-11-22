"""Cube pose node: camera images -> cube pose in hand_root, for policy_runner.

    ros2 run cube_perception cube_pose_node --ros-args --params-file config/cube_pose.yaml
    ros2 launch cube_perception perception.launch.py          # camera + this node

Every image:
  1. image -> numpy (image_utils, no cv_bridge), tracker_core finds the cube and hand marker 10;
  2. hand marker pose through HandRootAverage (the hand base doesn't move);
  3. cube pose in hand_root (m) through PoseFilter (outliers, hold while hidden, optional smoothing);
  4. publish LinkStates (name ["cube"], frame hand_root, image stamp, twist NaN) and the status.

Publishes
  /robot_hand/object/states        robot_hand_interfaces/LinkStates   pose NaN when there is none
  /robot_hand/object/status        std_msgs/String   tracking | predicting | lost | no_hand_marker
  /tf  (publish_tf)                <camera optical frame> -> hand_root -> cube
  /robot_hand/object/debug_image   (debug_image) image with the axes drawn, at debug_rate_hz
"""
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, TransformStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_hand_interfaces.msg import LinkStates
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from cube_perception.image_utils import image_to_bgr
from cube_perception.pose_filter import PoseFilter
from cube_perception.tracker_core import CubeTracker, HandRootAverage

NAN = float("nan")


def pose_msg(T):
    """4x4 (m) -> geometry_msgs/Pose; all NaN if T is None."""
    p = Pose()
    if T is None:
        p.position.x = p.position.y = p.position.z = NAN
        p.orientation.x = p.orientation.y = p.orientation.z = p.orientation.w = NAN
        return p
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    # scipy and ROS both use (x, y, z, w).
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
        float(v) for v in R.from_matrix(T[:3, :3]).as_quat())
    return p


def nan_twist():
    tw = Twist()
    tw.linear.x = tw.linear.y = tw.linear.z = NAN
    tw.angular.x = tw.angular.y = tw.angular.z = NAN
    return tw


def transform_msg(stamp, parent, child, T):
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child
    p = pose_msg(T)
    tf.transform.translation.x = p.position.x
    tf.transform.translation.y = p.position.y
    tf.transform.translation.z = p.position.z
    tf.transform.rotation = p.orientation
    return tf


def mm_to_m(T):
    T = T.copy()
    T[:3, 3] /= 1000.0
    return T


class CubePoseNode(Node):
    def __init__(self):
        super().__init__("cube_pose_node")
        config_dir = os.path.join(get_package_share_directory("cube_perception"), "config")
        p = self.declare_parameters("", [
            ("image_topic", "/camera/camera/color/image_raw"),
            ("states_topic", "/robot_hand/object/states"),
            ("status_topic", "/robot_hand/object/status"),
            ("intrinsics_path", ""),      # empty: config/camera_intrinsics.npz of this package
            ("cube_faces_path", ""),      # empty: config/cube_faces.json of this package
            ("cube_side_mm", 50.0),
            ("cube_marker_mm", 40.0),
            ("hand_marker_id", 10),
            ("hand_marker_mm", 40.0),
            ("hd_library", 11),
            ("max_fit_error_px", 3.0),
            ("hand_average.window", 60),
            ("hand_average.max_jump_mm", 5.0),
            ("hand_average.max_jump_deg", 3.0),
            ("hand_average.reset_after", 10),
            ("filter.max_hold_s", 0.2),
            ("filter.max_jump_mm", 20.0),
            ("filter.max_jump_deg", 30.0),
            ("filter.reset_after", 3),
            ("filter.smoothing", False),
            ("filter.alpha", 0.5),
            ("frame_id", "hand_root"),
            ("cube_name", "cube"),
            ("publish_tf", True),
            ("debug_image", False),
            ("debug_rate_hz", 5.0),
            ("log_period_s", 5.0),
        ])
        g = {param.name: param.value for param in p}

        self.tracker = CubeTracker(
            g["intrinsics_path"] or os.path.join(config_dir, "camera_intrinsics.npz"),
            g["cube_faces_path"] or os.path.join(config_dir, "cube_faces.json"),
            side_mm=g["cube_side_mm"], marker_mm=g["cube_marker_mm"],
            hand_marker_id=g["hand_marker_id"], hand_marker_mm=g["hand_marker_mm"],
            hd=g["hd_library"], max_fit_error_px=g["max_fit_error_px"])
        self.hand = HandRootAverage(
            window=g["hand_average.window"], max_jump_mm=g["hand_average.max_jump_mm"],
            max_jump_deg=g["hand_average.max_jump_deg"], reset_after=g["hand_average.reset_after"])
        self.filter = PoseFilter(
            max_hold_s=g["filter.max_hold_s"], max_jump_m=g["filter.max_jump_mm"] / 1000.0,
            max_jump_deg=g["filter.max_jump_deg"], reset_after=g["filter.reset_after"],
            smoothing=g["filter.smoothing"], alpha=g["filter.alpha"])
        self.frame_id = g["frame_id"]
        self.cube_name = g["cube_name"]
        self.publish_tf = g["publish_tf"]
        self.debug_period_s = 1.0 / g["debug_rate_hz"] if g["debug_image"] else None
        self.log_period_s = g["log_period_s"]

        self.states_pub = self.create_publisher(LinkStates, g["states_topic"], 10)
        self.status_pub = self.create_publisher(String, g["status_topic"], 10)
        self.debug_pub = (self.create_publisher(Image, "/robot_hand/object/debug_image", 1)
                          if self.debug_period_s else None)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.last_debug = 0.0
        # Depth 1, best effort: if processing falls behind, skip to the newest image instead of queueing.
        self.create_subscription(Image, g["image_topic"], self.on_image, qos_profile_sensor_data)

        self.stats = self._new_stats()
        self.last_log = time.monotonic()
        self.get_logger().info(
            f"cube_pose_node: {g['image_topic']} -> {g['states_topic']} (frame {self.frame_id})")

    @staticmethod
    def _new_stats():
        return {"frames": 0, "proc_ms": [], "latency_ms": [], "status": {}}

    def on_image(self, msg):
        t0 = time.monotonic()
        t_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        image = image_to_bgr(msg)
        est = self.tracker.estimate(image)

        T_cam_root = self.hand.update(est.T_cam_root)   # mm; held while the marker is hidden
        T_meas = None
        if est.cube_ok and T_cam_root is not None:
            T_meas = mm_to_m(np.linalg.inv(T_cam_root) @ est.T_cam_cube)
        T_out, status = self.filter.update(t_img, T_meas)
        if T_cam_root is None:
            status = "no_hand_marker"   # never seen marker 10: no hand_root to express the cube in
            T_out = None

        out = LinkStates()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.name = [self.cube_name]
        out.pose = [pose_msg(T_out)]
        out.twist = [nan_twist()]   # the OpenAI policy doesn't use cube velocity
        self.states_pub.publish(out)
        self.status_pub.publish(String(data=status))

        if self.tf_pub is not None and T_cam_root is not None:
            tfs = [transform_msg(msg.header.stamp, msg.header.frame_id, self.frame_id, mm_to_m(T_cam_root))]
            if T_out is not None:
                tfs.append(transform_msg(msg.header.stamp, self.frame_id, self.cube_name, T_out))
            self.tf_pub.sendTransform(tfs)

        if self.debug_pub is not None and t0 - self.last_debug >= self.debug_period_s:
            self.last_debug = t0
            self.publish_debug(msg, image, est, T_cam_root, status)

        self._record(t0, msg, status)

    def publish_debug(self, msg, image, est, T_cam_root, status):
        img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()
        K, dist = self.tracker.camera_matrix, self.tracker.dist_coeffs
        for T, length in ((T_cam_root, 40.0), (est.T_cam_cube, 30.0)):
            if T is not None:
                rvec, _ = cv2.Rodrigues(T[:3, :3])
                cv2.drawFrameAxes(img, K, dist, rvec, T[:3, 3], length)
        text = f"{status}  faces {est.face_ids}  fit {est.fit_error_px:.2f}px"
        if est.reason:
            text += f"  ({est.reason})"
        cv2.putText(img, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
        out = Image()
        out.header = msg.header
        out.height, out.width = img.shape[:2]
        out.encoding = "bgr8"
        out.step = img.shape[1] * 3
        out.data = img.tobytes()
        self.debug_pub.publish(out)

    def _record(self, t0, msg, status):
        s = self.stats
        s["frames"] += 1
        s["proc_ms"].append((time.monotonic() - t0) * 1000.0)
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        s["latency_ms"].append((self.get_clock().now().nanoseconds - stamp_ns) / 1e6)
        s["status"][status] = s["status"].get(status, 0) + 1
        now = time.monotonic()
        if now - self.last_log < self.log_period_s:
            return
        dt = now - self.last_log
        self.get_logger().info(
            f"{s['frames'] / dt:.1f} Hz | processing {np.mean(s['proc_ms']):.1f} ms "
            f"(max {np.max(s['proc_ms']):.1f}) | image stamp -> publish {np.mean(s['latency_ms']):.1f} ms "
            f"(max {np.max(s['latency_ms']):.1f}) | {s['status']} | outliers rejected {self.filter.rejected}")
        self.stats = self._new_stats()
        self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = CubePoseNode()
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
