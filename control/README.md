# UR3 Control Package

To test demo code, run in seperate terminals:

ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
and 
ros2 run control mtc_test

TO-DO:
- Make node that will subscribe to geometry/msgs to get the points required
- Publish geometry/msgs to this node
- Publish to a custom node when finished with the geometry/msgs.
- Run moveit code

OR 

Make a service call instead of the publish/subscribe method above

