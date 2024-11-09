from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livo_bringup',
            executable='odom_node',
            output='screen'),
        Node(
            package='livo_bringup',
            executable='driver_node',
            output='screen'),
    ])