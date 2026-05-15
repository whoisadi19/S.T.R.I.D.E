import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_control_pkg',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen'
        ),
        Node(
            package='drone_control_pkg',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        )
    ])
