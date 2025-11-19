from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    launch_file_dir = os.path.dirname(__file__)

    params_file = os.path.join(launch_file_dir, '../config/line_follower_params.yaml')

    return LaunchDescription([
        Node(
            package='line_follower',
            executable='avoid_wall',
            name='avoid_wall',
            parameters=[params_file],
            output='screen'
        )
    ])
