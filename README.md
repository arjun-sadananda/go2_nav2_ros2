# Unitree Go2 go 2 goal

ROS2 Packages for interfacing Unitree Go2 with Nav2.

This repo includes the follwoing ROS2 packages
- go2 robot description
  - /xacro: contains the robot description
  - /meshes: contains the mesh files
  - /config: contains the ros2_control parameters for joint level control and sensing interface
  - rest of the repo is not important and was used for debugging.
- champ
  - This is borrowed from [ROS Packages for CHAMP Quadruped Controller](https://github.com/chvmp/champ.git). Few files have been modified to make it suitable for go2.
  - Most important changes are in gait.yaml and state_estimation node. (Experimental changes in the trajectory_planner to change stance and swing time didn't show great results and threfore have been commented out)
  - Rest of the changes are primarily in launch files to change the robot, the world and the map used.


## How to run
- add the packages to your workspace and build.
- `ros2 launch champ_config gazebo.launch.py`
- `ros2 launch champ_config navigate.launch.py`

## Working:
gazebo.launch.py
  - launches champ_bringup bringup.launch.py
    - runs robot_state_publisher with the go2_description
    - runs quadruped_controller_node from champ_base that subscribes \cmd_vel (and body_pose) commands and publishes joint trajectory msgs, (and optionally foot_contact states)
  - robot_localisation to fuse estimated raw odometry with imu

  - 
