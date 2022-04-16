import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('rm_pioneer_bringup'), 'rviz', 'vision.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
        Node(
            package='image_transport',
            executable='republish',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', '/debug/final_img/compressed'),
                        ('out', '/debug/repub')],
        )
    ])
