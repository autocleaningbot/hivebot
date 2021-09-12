## Overview
This repo consists of 3 ROS packages that define the following:
1. Hivebot Trolley
2. Mycobot_Pro manipulator (320 mm) [Note this has been adapted from the manufacturer's ROS package]
3. trolley_arm_description [This package dynamically attaches the manipulator to the trolley for use in Rviz and Gazebo]


## Set Up Instructions
1. If you havent yet done so, create a catkin workspace using the following:

    a. create directory ie. ~/catkin_ws/src
    b. run `catkin_make`

2. Create an `src` folder inside your catkin workspace.

2. Git clone this repositority inside the src folder and rebuild your catkin_workspace.

    a. You should have ie. ~/catkin_ws/src/toilet_trolley_arm
    
3. In your root directory of your catkin_ws (ie. ~/catkin_ws) run `catkin_make`

## Startup 
1. To view the urdf in Rviz, run
    -  `roslaunch trolley_arm_description trolley_arm_description.launch`

2. To view the simulation in Gazebo, run
    - `roslaunch trolley_arm_description gazebo.launch`
    
## Potential Issues
1. Missing Dependencies
    - If there are dependency packages that are not yet installed
      - Run `rosdep install ${package name}`
    - effort_controllers/JointTrajectoryController does not exist
      - Run `sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers`
## Credits
1. Models used in this project are from https://github.com/elephantrobotics/mycobot_ros
