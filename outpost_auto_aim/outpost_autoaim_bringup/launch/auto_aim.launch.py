import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory(
        'outpost_autoaim_bringup'), 'config/default.yaml')

    detector_node = Node(
        package='outpost_detector',
        executable='outpost_detector_node',
        emulate_tty=True,
        output='screen',
        parameters=[params_file]
    )

    tracker_node = Node(
        package='outpost_tracker',
        executable='outpost_tracker_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file]
    )

    return LaunchDescription([
        detector_node,
        tracker_node,
    ])
