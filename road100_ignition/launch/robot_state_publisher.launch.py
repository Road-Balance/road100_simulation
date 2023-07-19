# Copyright 2023 Road Balance
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

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    # Package Directories
    pkg_path = get_package_share_directory('road100_ignition')
    xacro_path = os.path.join(pkg_path, "urdf", "road100.urdf.xacro")

    # Launch Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {"use_sim_time": True},
            #'frame_prefix': f"{namespace}/", # Reimplemented https://github.com/ros/robot_state_publisher/pull/169
            {'robot_description': Command([
                'xacro ', xacro_path,
            ])}
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "road100",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
            "-Y", "0.0",
        ]
    )

    return LaunchDescription([
        # Nodes and Launches
        robot_state_publisher,
        # gz_spawn_entity,
    ])
