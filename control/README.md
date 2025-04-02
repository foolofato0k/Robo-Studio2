# UR3 Control Package

To test demo code, run in seperate terminals:

ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
and 
ros2 run control mtc_test

To RUN actual robot, run in seperate terminals:
1. ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=true
2. ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
3. Robot control code e.g. ros2 run control mtc_test

TO-DO:
- Make node that will subscribe to geometry/msgs to get the points required
- Publish geometry/msgs to this node
- Publish to a custom node when finished with the geometry/msgs.
- Run moveit code

OR 

Make a service call instead of the publish/subscribe method above

