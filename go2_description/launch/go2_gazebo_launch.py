#!/usr/bin/python3
# -*- coding: utf-8 -*-

#Archit Jain

import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='go2_description').find('go2_description')

    go2_xacro_file = os.path.join(pkg_share, 'xacro/', 'robot.xacro')
    # assert os.path.exists(go2_xacro_file), "The robot.xacro doesnt exist in "+str(go2_xacro_file)
    doc = xacro.parse(open(go2_xacro_file))
    xacro.process_doc(doc)
    # go2_description_config = xacro.process_file(go2_xacro_file)
    # go2_description = go2_description_config.toxml()
    go2_description = doc.toxml()

    # xacro_path = os.path.join(
    #     get_package_share_directory('go2_description'),
    #     'xacro',
    #     'robot.xacro'
    # )

    urdf_path = os.path.join(
        get_package_share_directory('go2_description'),
        'urdf',
        'go2_description.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()


    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('go2_description'), 'launch', 'empty_world.launch.py'),
        )
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        name='robot_state_publisher',
        executable='robot_state_publisher',
        # parameters=[{"robot_description": robot_desc}]
        parameters=[{"robot_description": go2_description}]
    )

    static_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments = ["1.6", "-2.4", "-0.8", "3.14", "0", "0", "world", "odom"],
        output='screen')
    
    spawn_go2 = launch_ros.actions.Node(
    	package='gazebo_ros', 
        name='go2_spawner',
    	executable='spawn_entity.py',
        arguments=['-entity', 'go2_robot', 
                   '-topic', 'robot_description', 
                   '-timeout', '60', 
                #    '-spawn_service_timeout', '60',
                   # '-robot_namespace', 'go2',
                   '-x', '0.0', '-y', '0.0', '-z', '0.6', '-Y', '0.0'],
        # arguments=['-entity', 'go2', '-file', xacro_path, '-x', '0.0', '-y', '0.0', '-z', '0.6', '-Y', '0.0'],
        output='screen'
    )

    # go2_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("go2_description"),
    #         "config",
    #         "go2_control.yaml",
    #     ]
    # )
    # control_node = launch_ros.actions.Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[go2_controllers],
    #     output="both",
    #     remappings=[
    #         ("~/robot_description", "/robot_description"),
    #     ],
    # )
    # controller_manager = launch_ros.actions.Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    # )

    go2_joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["go2_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    load_go2_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'go2_joint_state_broadcaster']
    )
    load_go2_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'go2_joint_trajectory_controllers']
    )

    go2_joint_trajectory_controllers_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["go2_joint_trajectory_controllers", "--controller-manager", "/controller_manager"],
        # output="screen",
    )

    # control_node = launch_ros.actions.Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_controllers],
    #     output="both",
    #     remappings=[
    #         ("~/robot_description", "/robot_description"),
    #     ],
    # )

    delay_bot_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher_node,
            on_exit=[spawn_go2],
        )
    )
    delay_jsb_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_go2,
            on_exit=[go2_joint_state_broadcaster_spawner],
        )
    )

    delay_jtc_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=go2_joint_state_broadcaster_spawner,
            on_exit=[go2_joint_trajectory_controllers_spawner],
        )
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        start_world,
        robot_state_publisher_node,
        # delay_bot_spawn,
        spawn_go2,
        # load_go2_joint_state_broadcaster,
        # load_go2_joint_trajectory_controller
        # delay_jsb_spawner,
        go2_joint_state_broadcaster_spawner,
        # delay_jtc_spawner
        go2_joint_trajectory_controllers_spawner
        # spawn_controller_fr,
        # spawn_controller_fl,
        # spawn_controller_rr,
        # spawn_controller_rl
        # static_transform,
        # joint_state_publisher_node_quad,
        # joint_state_publisher_gui_node_quad
    ])