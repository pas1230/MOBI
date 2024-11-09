from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odrive_ep_pkg',
            executable='odrive_ep_motor_double',
            output='screen'),
        Node(
            package='odrive_ep_pkg',
            executable='odrive_ep_joy',
            output='screen'),
    ])
