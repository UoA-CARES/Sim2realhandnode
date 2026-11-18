"""Save frames from the camera topic as PNG (BGR, like cv2.imwrite expects), for offline tests.

Needs the camera driver running and ROS sourced:
    python3 tools/save_frames.py --out ~/cube_frames --count 10 --every 1.0
"""
import argparse
import os
import sys

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cube_perception.image_utils import image_to_bgr  # noqa: E402


class SaveFrames(Node):
    def __init__(self, args):
        super().__init__("save_frames")
        self.args = args
        self.saved = 0
        self.last = None
        os.makedirs(args.out, exist_ok=True)
        self.create_subscription(Image, args.topic, self.on_image, 1)
        self.get_logger().info(f"saving {args.count} frames from {args.topic} to {args.out}")

    def on_image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last is not None and stamp - self.last < self.args.every:
            return
        self.last = stamp
        path = os.path.join(self.args.out, f"frame_{self.saved:03d}.png")
        cv2.imwrite(path, image_to_bgr(msg))
        self.saved += 1
        self.get_logger().info(f"{path} ({msg.width}x{msg.height} {msg.encoding})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--out", default=os.path.expanduser("~/cube_frames"))
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--every", type=float, default=1.0, help="seconds between saved frames")
    args = parser.parse_args()

    rclpy.init()
    node = SaveFrames(args)
    try:
        while rclpy.ok() and node.saved < args.count:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
