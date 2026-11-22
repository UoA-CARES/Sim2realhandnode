"""Start the camera (camera.launch.py) and the cube pose node together.

    ros2 launch cube_perception perception.launch.py
    ros2 launch cube_perception perception.launch.py debug_image:=true

Node parameters: config/cube_pose.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('cube_perception')

    serial_no = DeclareLaunchArgument(
        'serial_no', default_value='826212070191', description='Serial number of the D435 to open')
    debug_image = DeclareLaunchArgument(
        'debug_image', default_value='false', description='Publish /robot_hand/object/debug_image')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'camera.launch.py')),
        launch_arguments={'serial_no': LaunchConfiguration('serial_no')}.items(),
    )

    cube_pose = Node(
        package='cube_perception',
        executable='cube_pose_node',
        name='cube_pose_node',
        parameters=[
            os.path.join(share, 'config', 'cube_pose.yaml'),
            {'debug_image': ParameterValue(LaunchConfiguration('debug_image'), value_type=bool)},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([serial_no, debug_image, camera, cube_pose])
