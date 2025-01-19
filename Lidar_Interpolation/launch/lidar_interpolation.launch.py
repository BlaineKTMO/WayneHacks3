import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_interpolation_node',  # Your package name
            executable='simple_node',  # Your node executable
            name='simple_node',
            output='screen',
            parameters=[
                {'input_topic': '/velodyne_points'},
                {'output_topic': '/lidar_points'},
                {'interpolation_multiple': 2}  # Example: Change offset to 2
            ]
        )
    ])
