# run_all.launch.py
# Launch UR simulation with MoveIt, plus GUI and Processing nodes
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # UR Simulation + MoveIt
    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_simulation_gazebo'),
                'launch',
                'ur_sim_moveit.launch.py'
            )
        )
    )

    # Processing node
    processing_node = Node(
        package='robo_da_vinci',
        executable='processing_node',
        name='processing_node',
        output='screen'
    )

    # GUI node
    gui_node = Node(
        package='robo_da_vinci',
        executable='gui_node',
        name='gui_node',
        output='screen'
    )

    return LaunchDescription([
        ur_sim,
        processing_node,
        gui_node,
    ])
