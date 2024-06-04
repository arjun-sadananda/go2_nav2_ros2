#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

import launch_ros
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'go2_description.urdf'

    # print('urdf_file_name : {}'.format(urdf_file_name))

    # urdf_path = os.path.join(
    #     get_package_share_directory('go2_description'),
    #     'urdf',
    #     urdf_file_name)
    xacro_path = os.path.join(
        get_package_share_directory('go2_description'),
        'xacro',
        'robot.xacro')

    with open(xacro_path, 'r') as infp:
        robot_desc = infp.read()

    pkg_share = launch_ros.substitutions.FindPackageShare(package='go2_description').find('go2_description')
    go2_xacro_file = os.path.join(pkg_share, 'xacro/', 'robot.xacro')
    # assert os.path.exists(go2_xacro_file), "The robot.xacro doesnt exist in "+str(go2_xacro_file)
    # doc = xacro.parse(open(go2_xacro_file))
    # xacro.process_doc(doc)
    go2_description_config = xacro.process_file(go2_xacro_file)
    go2_description = go2_description_config.toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': go2_description
            }],
        ),
    ])
