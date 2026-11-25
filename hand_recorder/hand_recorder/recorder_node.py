import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import os

class RecorderNode(Node):
    def __init__(self):
        super().__init__('hand_recorder')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('log_every_n', 30)
        self.declare_parameter('output_path', os.path.expanduser('~/hand_recorder_test/test.mp4'))
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('duration_sec', 10.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.log_every_n = self.get_parameter('log_every_n').value
        self.output_path = os.path.expanduser(self.get_parameter('output_path').value)
        self.fps = self.get_parameter('fps').value
        self.duration_sec = self.get_parameter('duration_sec').value

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.bridge = CvBridge()

        # Initialise tracking variables for video writing
        self.video_writer = None
        self.first_stamp = None
        self.done = False
        self.frames_written = 0

        self.get_logger().info('Recorder node has been started.')

        # track max time
        self.max_time = 0.0

    def image_callback(self, msg):
        if self.done:                   # stops recording after the specified duration
            return
        enc = msg.encoding
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, *_ = frame.shape

        if self.video_writer is None:
            self._open_writer(w, h)  # Open the video writer if not already opened
            if self.done:  # Check if the video writer failed to open
                return


        # Check if the recording duration has been reached
        if time_stamp - self.first_stamp > self.duration_sec:
            self._close_writer()  # Close the video writer and stop recording
            self.get_logger().info(f'max frame save time: {self.max_time:.2f} ms')
            return
        else:

            # write the frame to the video file
            self.video_writer.write(frame)
            self.frames_written += 1
            # Print image information every log_every_n frames (1 sec by default)
            if self.frames_written % self.log_every_n == 0:
                self.get_logger().info('Image: %dx%d, %s, %s, %f, ' % (w, h, frame.dtype, enc, time_stamp))

    def _open_writer(self, w, h):
        # Output video dir
        output_dir = os.path.dirname(self.output_path)
        os.makedirs(output_dir, exist_ok=True)
        # Start video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))         
        if self.video_writer.isOpened():
            self.get_logger().info('Video writer opened successfully.')
        else:
            self.get_logger().error('Failed to open video writer.')
            self.done = True  # Stop recording if the video writer fails to open
            return

    def _close_writer(self):
        self.get_logger().info('Recording duration reached. Stopping recording.')
        self.get_logger().info('Total frames written: %d' % self.frames_written)
        self.done = True
        self.video_writer.release()
        return


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                      # Ctrl+C or `ros2 launch` shutting down: not an error
                                    # Will need to change this later as using `ros2 launch` already calls rclpy.try_shutdown() and a 2nd call will corrupt the video file if it is not finalised properly. So we need to handle this case and finalise the video file before shutting down.
    finally:
        node.destroy_node()       # later: this is where an unfinished video gets finalised
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
