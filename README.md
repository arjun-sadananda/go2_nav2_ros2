# Unitree Go2 goes 2 goal

ROS2 Packages for simulating Unitree Go2 in Gazebo ROS2 and navigating the robot to a desired goal using Nav2.

Tested on ROS2 Humble.

This repo includes the following packages:
- **`go2_description`**
  - This is borrowed from https://github.com/unitreerobotics/unitree_ros.git and modified 
	  - to work in ROS2, (mainly in the mesh tags)
	  - to use ros2_control and 
	  - to mount a 2D lidar.
  - `/xacro`contains the robot description, `/meshes` contains the mesh files used.
  - Additionally `/config` is added to contain the ros2 control parameters for joint level control and state feedback interface
  - rest of the repo is not important used but was used during debugging.
- **`champ`**
  - This is borrowed from [ROS Packages for CHAMP Quadruped Controller](https://github.com/chvmp/champ.git). Few files have been modified to make it suitable for go2.
  - Most important changes are in `gait.yaml` and `state_estimation` node. (Experimental changes in the trajectory_planner to change stance and swing time didn't show great results and threfore have been commented out).
  - Rest of the changes are primarily in launch files to change the robot, the world and the map used.
- **`aws-robomaker-small-warehouse-world`**
	- This is borrowed from https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
	- It contains the `no_roof_small_warehouse.world`, the models and the map used for navigation.

## To Run: Go2 navigates 2 goal (in amazon small warehouse)
- Clone the packages to your workspace, build and source the repo (if not already).
- Launch Gazebo and spawn the robot along with its controllers and state estimators. 
```
ros2 launch champ_config gazebo.launch.py
```
- Launch Nav2 and RViz interface. 
```
ros2 launch champ_config navigate.launch.py
```
- Provide 2D Nav Goal on RViz.

![navigation](https://github.com/arjun-sadananda/go2_nav2_ros2/blob/main/go2nav2.gif)

## Behind the Scenes
**`champ_config`** `gazebo.launch.py`
  - launches **`champ_bringup`** `bringup.launch.py`
    - runs `robot_state_publisher` with the `robot_description` from **`go2_description`** `xacro/go2_robot.xacro`
    - runs **`champ_base`**`quadruped_controller_node` . This node subscribes to `\cmd_vel` (and `body_pose`) commands, generates gaits to achieve the body velocity and publishes joint trajectory msgs on `joint_group_effort_controller/joint_trajectory`(joint_control_topic param).
	    - this nodes takes gait related params from the `gait.yaml` file. The `nominal_height` parameter is reduced to better suit go2. This had a big positive impact in the performance (stability) of the go2 controller performance. 
	- runs **`champ_base`**  `state_estimation_node`. This node subscribes to `joint_states` and `foot_contacts`, estimates base_link to base_footprint transform & odometry  and publishes `base_to_footprint_pose` &`odom/raw`.
		- An issue was noticed later in navigation in localisation of the go2 robot. It was found that the major issue was in odometry (mostly since it takes CHAMP robot dimensions). The estimated pose using purely odom was consistently (roughly) half of the actual translation made. Therefore a temporary fix for this was made by modifying the `state_estimation_node` to simply double the linear velocity published in `odom/raw`.
	- runs two `ekf_node`(s)  from the `robot_localisation` package to fuse estimated raw odometry with imu. These nodes also publish the `tf`(s)  `base_footprint` -> `base_link` and `odom`->`base_footprint`
 - launches **`champ_gazebo`** `gazebo.launch.py`
	  - launches `gzserver.launch.py` and `gzclient.launch.py` from `gazebo_ros` package
	  - spawns the robot using `spawn_entity.py` from `gazebo_ros` package
 using the robot description from `/robot_description` topic.
	  - runs `contact_sensor` node from the `champ_gazebo` package. this node subscribes to `/physics/contacts` and publishes to `foot_contacts` (used by `state_estimation_node`).
	  - loads the two controllers `joint_group_position_controller` and `joint_group_effort_controller` in the ro2_control controller_manager.
  
**`champ_config`** `navigate.launch.py`
- launches **`champ_navigation`**  `navigate.launch.py` and provides `map.yaml` and `navigation.yaml` as launch arguments.
	- launches **`nav2_bringup`** `bringup_launch.py`with the map and params provided.
	- runs `rviz2` with the `navigation2.rviz` configuration.
	- the navigation.yaml has many important params including defining the frames, base_frame_id: "base_footprint", odom_frame_id: "odom", global_frame_id: "map", robot_model_type: "nav2_amcl::DifferentialMotionModel" and all the navigation related parameter. All params have been left as in CHAMP's implementation. Future Work: an experiment yet to be conducted is to try holonomic drive (instead of the differential drive) model, since the the low/mid- level controller implementation (and the robot) is capable of controlling all 3 DOF on the plane.

The packages not mentioned so far ar `champ` and `champ_msgs`. `champ`  contains the library files (from https://github.com/chvmp/libchamp.git) needed for the quadruped_controller_node and state_estimation_node. `champ_msgs` contains the custom msg types used in the aforementioned nodes.

That wraps the tour of this repo.

---
