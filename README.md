# Robo-Studio2
Robotics Studio 2

#test

# UR3 Control and Planning Packages 

## Control TESTS
### Simulation Test
To run test nodes code in SIMULTATION, run in seperate terminals:
This demonstrates the ur3_control_node subscribing to geometry::PoseArray
and then completing the cartesian movements of the array.

ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
and 
ros2 run control ur3_control_node
ros2 run py_planning test_publish_node


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

## Planning Test
To run general planning tests:
In root workspace
1. colcon build --symlink-install
2. colcon test --packages-select py_planning
3. colcon test-result --verbose

### Visualisation Tests
To run the visualisation tests:
Run the visual_testing/visual_tests.py file

## TO-DO:
### Control
* Publish to a custom node when finished with the geometry/msgs.
* Create a function that adjusts the points based on the gripper

### Planning
* Make the planning more resiliant (move to a good planning location at the start)