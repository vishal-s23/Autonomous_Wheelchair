from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('wheelchair_simulation')

    world_path = os.path.join(
        pkg_share,
        'worlds',
        'turtlebot3_house.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'wheelchair.urdf.xacro'
    )

    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )

    spawn_wheelchair = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
    		'-name', 'wheelchair',
    		'-topic', 'robot_description',
    		'-x', '1.30',
    		'-y', '-1.54',
    		'-z', '-0.13',
		'-Y', '1.53'
	],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_wheelchair
    ])
