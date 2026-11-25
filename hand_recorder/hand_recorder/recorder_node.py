import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
# from cv_bridge import CvBridge
import cv2

class RecorderNode(Node):
    def __init__(self):
        super().__init__('hand_recorder')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('log_every_n', 30)

        self.image_topic = self.get_parameter('image_topic').value
        self.log_every_n = self.get_parameter('log_every_n').value

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.counter = 0
        self.get_logger().info('Recorder node has been started.')

    def image_callback(self, msg):
        w = msg.width
        h = msg.height
        enc = msg.encoding
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.counter % self.log_every_n == 0:
            self.get_logger().info('Image: %dx%d, %s, %f' % (w, h, enc, time_stamp))
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                      # Ctrl+C or `ros2 launch` shutting down: not an error
                                    # Will need to change this later as using `ros2 launch` already calls rclpy.try_shutdown() and will corrupt the video file if it is not finalised properly. So we need to handle this case and finalise the video file before shutting down.
    finally:
        node.destroy_node()       # later: this is where an unfinished video gets finalised
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
