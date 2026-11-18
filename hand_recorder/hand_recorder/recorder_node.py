import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
import cv2

class RecorderNode(Node):
    qos = 10         # queue size equation: Queue Size >= f_pub(t_proc + t_jitter) where f_pub is the publisher frequency, t_proc is the max execution time of subscriber callback, and t_jitter is network jitter
                            # Placeholder 10 for now, but will be updated once we have a better idea of the processing time and network jitter
    def __init__(self):
        super().__init__('hand_recorder')
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            self.qos,
        )
        period = 1 # seconds
        self.counter = 0
        self.get_logger().info('Recorder node has been started.')

    def image_callback(self, msg):
        w = msg.width
        h = msg.height
        enc = msg.encoding
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.counter % 30 == 0:
            self.get_logger().info('Image: %dx%d, %s, %f' % (w, h, enc, time_stamp))
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                      # Ctrl+C or `ros2 launch` shutting down: not an error
    finally:
        node.destroy_node()       # later: this is where an unfinished video gets finalised
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
