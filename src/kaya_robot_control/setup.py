from setuptools import find_packages, setup

package_name = 'kaya_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='tmkristi',
    maintainer_email='tmkristi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel = kaya_robot_control.Basic_control:main',  
            'motionDynamixel = kaya_robot_control.create_motion:main',
            'Controller = kaya_robot_control.create_motion_controller:main',
            'wasdController = kaya_robot_control.teleop_wasd:main'
        ],
    },
)

