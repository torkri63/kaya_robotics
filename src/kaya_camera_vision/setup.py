from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'kaya_camera_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), ['models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elisha',
    maintainer_email='elisha@todo.todo',
    description='Cube detection and tracking using YOLO, depth camera, and EKF in ROS2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'live_capture_node = kaya_camera_vision.live_capture_node:main',
            'cube_detection_node = kaya_camera_vision.cube_detection_node:main',
            'motion_planning_subscriber = kaya_camera_vision.motion_planning_subscriber:main',
        ],
    },
)
