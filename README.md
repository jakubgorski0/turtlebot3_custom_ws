# ROS 2 Packages for Robot Teleoperation and Localization with AMCL
This repository contains two ROS 2 packages developed for controlling the turtlebot3 and performing localization and navigation using the Nav2 navigation stack.

## 1. custom_teleop_pkg
The custom_teleop_pkg package provides a node for teleoperating a robot via a computer keyboard. This node allows users to control the robot's movement using keyboard inputs.
### Usage
Launch the teleoperation node:
```
ros2 launch custom_teleop_pkg teleop_keyboard_launch.py
```
Use the keyboard arrow keys to control the robot's movement.
<pre>
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        w 
   a    s    d
        x    

anything else : stop

q/z : increase/decrease only linear speed by 0.025
e/c : increase/decrease only angular speed by 0.025

CTRL-C to quit
</pre>
## 2. loc_nav_pkg
The loc_nav_pkg package contains launch files for configuring localization and navigation using the Nav2 navigation stack.

### Launch Files
`localization_robot.launch.py`: Launches the AMCL node for robot localization. <br/>
`navigation_robot.launch.py`: Launches the Nav2 navigation stack for robot navigation. <br/>
### Usage
1. Launch localization:
```
ros2 launch loc_nav_pkg localization_robot.launch.py
```
2. Launch navigation stack:
```
ros2 launch loc_nav_pkg navigation_robot.launch.py
```
