import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kaya_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # now picks up bringup_launch.py, hardware.launch.py, software.launch.py, etc.
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    scripts=[
        'scripts/init_hardware.sh',
        'scripts/launch_robot.sh',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmkristi',
    maintainer_email='tmkristi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # e.g. 'my_node = kaya_bringup.my_module:main',
        ],
    },
)