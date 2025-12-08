import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # package path
    pkg_name = 'obstacle_detector'
    pkg_share = get_package_share_directory(pkg_name)
    
    # config/params.yaml file path
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Declare launch argument
    # Allows changing the params file from the command line with 'ros2 launch ... params_file:=/path/to/custom.yaml'
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the ROS 2 parameters file to use'
    )

    obstacle_extractor_node = Node(
        package=pkg_name,
        executable='obstacle_extractor_node', 
        name='obstacle_extractor',            
        output='screen',                     
        parameters=[LaunchConfiguration('params_file')], 
        remappings=[
        ]
    )

    return LaunchDescription([
        params_file_arg,
        obstacle_extractor_node
    ])