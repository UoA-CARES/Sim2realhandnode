from glob import glob

from setuptools import find_packages, setup

package_name = 'cube_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsh Thorat',
    maintainer_email='harshthorat123@gmail.com',
    description='Camera launch and cube pose node for the robot hand (sim2real).',
    license='TODO',
    entry_points={
        'console_scripts': [
            'cube_pose_node = cube_perception.cube_pose_node:main',
            'global_view_node = cube_perception.global_view_node:main',
        ],
    },
)
