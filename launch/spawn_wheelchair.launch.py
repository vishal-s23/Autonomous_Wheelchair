#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_name = 'wheelchair_simulation'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_share, 'urdf', 'wheelchair.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    robot_description = xacro.process_file(urdf_file).toxml()

    # --- Gazebo (FIXED world handling) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # --- Robot State Publisher ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )

    # --- Spawn robot ---
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wheelchair',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # --- ROS ↔ Gazebo Bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        bridge
    ])
