"""sensor_msgs/Image -> numpy, without cv_bridge (avoids its NumPy-version coupling)."""
import cv2
import numpy as np


def image_to_bgr(msg):
    """sensor_msgs/Image (rgb8 / bgr8 / mono8) -> numpy image cv2 can use. Handles row padding (step)."""
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}[msg.encoding]
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.squeeze()
