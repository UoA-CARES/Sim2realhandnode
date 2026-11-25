"""Launch file for the hand recorder node.

    ros2 launch hand_recorder recorder.launch.py

Subscribes to image publisher and merges into video.
Saves the video to pc in ARD runs folder output per iteration.
Also gets camera info.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    recorder_yaml = os.path.join(get_package_share_directory('hand_recorder'), 'config', 'recorder.yaml')
    return LaunchDescription([
        Node(
            package='hand_recorder',
            executable='recorder_node',
            name='hand_recorder',
            parameters=[recorder_yaml],
        )
    ])