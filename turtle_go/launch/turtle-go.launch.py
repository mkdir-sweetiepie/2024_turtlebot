from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='set_imu',
            executable='set_imu',
            name='set_imu'
        ),
        Node(
            package='robit_vision',
            executable='robit_vision',
            name='robit_vision'
        ),
        Node(
            package='robit_master',
            executable='robit_master',
            name='robit_master'
        )
    ])
