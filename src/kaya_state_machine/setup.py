from setuptools import find_packages, setup

package_name = 'kaya_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['kaya_state_machine', 'kaya_state_machine.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmkristi',
    maintainer_email='maggikrisse@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'main_node = kaya_state_machine.main_node:main',

        ],
    },
)
