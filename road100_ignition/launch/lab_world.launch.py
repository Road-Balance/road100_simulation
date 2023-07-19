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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    
    pkg_path = get_package_share_directory('road100_ignition')

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_path, 'models'), ':' +
            os.path.join(pkg_path, 'worlds'), ':'
        ]
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r lab.sdf'}.items(),
    )

    # Get URDF via xacro
    rsp_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
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

    # Gz - ROS Bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # "/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            # "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # "/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/default/model/road100/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        remappings=[
            ('/world/default/model/road100/joint_state', 'road100/joint_states'),
            # ('/imu', 'bcr_bot/imu'),
            # ('/camera_info', 'bcr_bot/camera_info'),
        ]
    )

    # rqt robot steering
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen'
    )

    return LaunchDescription([
        # Nodes and Launches
        ign_resource_path,
        gazebo,
        rsp_launcher,
        gz_spawn_entity,
        gz_ros2_bridge,
        rqt_robot_steering,
    ])
