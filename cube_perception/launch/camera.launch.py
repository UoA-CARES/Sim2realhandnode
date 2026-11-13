"""Start the RealSense D435 colour stream with the team's shared settings (config/camera.yaml).

    ros2 launch cube_perception camera.launch.py
    ros2 launch cube_perception camera.launch.py serial_no:=123456789012   # a different D435

Publishes /camera/camera/color/image_raw (1280x720, ~30 Hz) and /camera/camera/color/camera_info.

The driver node is started directly instead of through realsense2_camera's rs_launch.py,
because rs_launch.py silently drops any setting not in its own argument list
(e.g. rgb_camera.auto_exposure_priority).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_yaml = os.path.join(get_package_share_directory('cube_perception'), 'config', 'camera.yaml')

    serial_no = DeclareLaunchArgument(
        'serial_no', default_value='826212070191', description='Serial number of the D435 to open')

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        parameters=[
            camera_yaml,
            # str type, or a numeric serial would be read as an int and rejected.
            {'serial_no': ParameterValue(LaunchConfiguration('serial_no'), value_type=str)},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([serial_no, camera])
