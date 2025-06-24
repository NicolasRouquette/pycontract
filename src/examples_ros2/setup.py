from setuptools import setup

package_name = 'examples_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@email.com',
    description='ROS2 examples for pycontract',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_publisher = examples_ros2.scripts.command_publisher:main',
            'status_publisher = examples_ros2.scripts.status_publisher:main',
            'ros2_monitor = examples_ros2.scripts.ros2_monitor:main',
        ],
    },
)
