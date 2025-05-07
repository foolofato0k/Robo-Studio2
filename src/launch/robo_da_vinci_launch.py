from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robo_da_vinci',
            namespace='processing',
            executable='processing_node',
            name='processing_node'
        ),
        Node(
            package='gui',
            namespace='gui',
            executable='gui_node',
            name='gui_node'
        ),
    ])