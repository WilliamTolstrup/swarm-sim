# Copyright (c) 2018 Intel Corporation
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

"""
Example for spawing multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_robot_list(num_robots):

    robot_list = []

    for i in range(1, num_robots + 1):
        robot_name = "robot" + str(i)
        x_pos = float(i)
        robot_list.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.5, 'z_pose': 0.01})

    return robot_list

def generate_custom_list(start, end):
    robot_list = []

    for i in range(start, end + 1):
        robot_name = "robot" + str(i)
        x_pos = float(i)
        robot_list.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.5, 'z_pose': 0.01})
    
    return robot_list

def generate_launch_description():
    bringup_dir = get_package_share_directory('remote')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Names and poses of the robots

    robots = generate_robot_list(3)

    # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'factory_world.world'),
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'factory_world.yaml'),
        description='Full path to map file to load')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the stacks')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    return ld
