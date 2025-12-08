import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_detector',
            executable='obstacle_visualizer_node',
            name='obstacle_visualizer',
            output='screen'
        )
    ])
