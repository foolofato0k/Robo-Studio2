# Robo-Da-Vinci

## What it is
A UR3-powered “art-bot” that turns a live webcam portrait into pen-and-ink splines.  

## Why use it
- **Automated art:**  Turn any face into a “wanted” poster  
- **Research playground:**  Path-planning, ROS2 control, Gazebo sim  
- **Modular design:**  Pick and choose image-proc, planning, control or GUI  

## Quickstart Simulation

1. **Clone & build**

Copy this git to the src folder of your ROS2 workspace:  
```bash
git clone https://github.com/foolofato0k/Robo-Studio2.git
cd Robo-Da-Vinci
colcon build --symlink-install
```
2. **Begin Simulation**
In one terminal copy the following command:
```bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```
3. **Begin Control, Processing And GUI nodes**
```bash
ros2 launch gui bringup.launch.py
```
**This launch file automatically starts:**

* processing_node (for image-based path planning)
* gui_node (for user photo capture and confirmation)
* ur3_control_node (for connection with the robot)



## Useful links
### Full docs & tutorials: 

For full documentation, tutorials and tests check the following page to see a full outline of the project

Wiki Home: https://github.com/foolofato0k/Robo-Studio2/wiki
