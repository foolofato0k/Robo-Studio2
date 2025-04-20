# UR3 Control Package

## TESTS
### Simulation Test
To run test nodes code in SIMULTATION, run in seperate terminals:
This demonstrates the ur3_control_node subscribing to geometry::PoseArray
and then completing the cartesian movements of the array.

ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
and 
ros2 run control ur3_control_node
ros2 run py_planning test_publish_node

<!-- ### Simplified Simulation Test (with workspace auto‑sourcing)

* **One‑time setup**: Add your workspace’s setup file to your shell startup (e.g. `~/.bashrc` or `~/.zshrc`), replacing the path with your own workspace:
    echo "source /path/to/your/ros2_ws/install/setup.bash" >> ~/.bashrc
 
Or manually source in each new terminal:
source /path/to/your/ros2_ws/install/setup.bash

Commands to run:
* **Terminal 1: Gazebo + MoveIt** ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py 
* **Terminal 2: UR3 control node** ros2 run control ur3_control_node 
* **Terminal 3: PoseArray publisher** ros2 run control pose_array_publisher -->


### Real Robot Test w/out nodes
To RUN actual robot, run in seperate terminals:
1. ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=true
2. ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
3. Robot control code e.g. ros2 run control mtc_test

### Real Robot Test w/ nodes
To RUN actual robot, run in seperate terminals:
1. ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=true
2. ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
3. ros2 run control ur3_control_node
4. ros2 run control pose_array_publisher


## TO-DO:
* Publish to a custom node when finished with the geometry/msgs.
* Create a function that adjusts the points based on the gripper


