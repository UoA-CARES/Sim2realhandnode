from setuptools import find_packages, setup

package_name = 'hand_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrew',
    maintainer_email='andyjkoh@gmail.com',
    description='Collects compressed image frames and collates them into a video file, encodes and saves the video file to disk.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'recorder_node = hand_recorder.recorder_node:main'
        ],
    },
)
