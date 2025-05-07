from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Controllers package nodes
        Node(
            package='controllers',
            executable='odometry_node',
            name='odometry_node',
        ),
        Node(
            package='controllers',
            executable='position_controller',
            name='position_controller',
        ),
        Node(
            package='controllers',
            executable='speed_controller',
            name='speed_controller',
        ),

        # Image processing package nodes
        Node(
            package='image_processing',
            executable='image_processor',
            name='image_processor',
        ),
        Node(
            package='image_processing',
            executable='camera_sim',
            name='camera_sim',
        ),
        Node(
            package='image_processing',
            executable='traffic_signal',
            name='traffic_signal',
        ),
    ])
