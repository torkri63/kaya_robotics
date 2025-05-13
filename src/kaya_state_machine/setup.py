# src/kaya_state_machine/setup.py

import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'kaya_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
    ],
    zip_safe=True,
    maintainer='tmkristi',
    maintainer_email='maggikrisse@gmail.com',
    description='State machine nodes for the KAYA platform',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'main_node = kaya_state_machine.main_node:main',
        ],
    },
)
