from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch controller_node from controllers package
        Node(
            package='controllers',
            executable='position_controller',
            name='position_controller',
            output='screen',
            parameters=[{'param_name': 'default_value'}]  # Default parameters (if needed)
        ),
        
        # Launch odometry_node from controllers package
        Node(
            package='controllers',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{'param_name': 'default_value'}]  # Default parameters (if needed)
        ),
        
        # Launch rviz_visualizer from simulations package
        Node(
            package='simulations',
            executable='rviz_visualizer',
            name='rviz_visualizer',
            output='screen'
        ),
        
        # Launch RViz with default settings
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
