import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sentry_control'), 'config', 'control.yaml')

    return LaunchDescription([
        Node(
            package='sentry_control',
            executable='control_node',
            name='sentry_control',
            output='screen',
            parameters=[config],
        ),
    ])
