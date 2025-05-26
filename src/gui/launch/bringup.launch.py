from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # your Python planning node
        Node(
            package='py_planning',
            executable='processing_node',
            name='processing_node',
            output='screen',
        ),

        # your Python GUI node
        Node(
            package='gui',
            executable='gui_node',
            name='gui_node',
            output='screen',
        ),

        # your C++ control node
        Node(
            package='control',
            executable='ur3_control_node',
            name='ur3_control_node',
            output='screen',
        ),

    ])
