"""Show the RGBD stream and the object poses in hand_root (global_view_node + RViz).

    ros2 launch cube_perception global_view.launch.py

perception.launch.py must be running. Closing the RViz window stops this launch.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('cube_perception')

    view = Node(
        package='cube_perception',
        executable='global_view_node',
        name='global_view_node',
        output='screen',
        emulate_tty=True,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'config', 'global_view.rviz')],
        on_exit=Shutdown(),
    )

    return LaunchDescription([view, rviz])
