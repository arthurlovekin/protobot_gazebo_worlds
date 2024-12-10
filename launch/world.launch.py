from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, 
                            IncludeLaunchDescription)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_description = LaunchDescription()

    world_arg = DeclareLaunchArgument(
        'world_file',
        default_value='simple.sdf',
        choices=['simple.sdf'],
        description='Worlds to load into Gazebo'
    )
    launch_description.add_action(world_arg)

    world_file_path = PathJoinSubstitution([
                        FindPackageShare('protobot_gazebo_worlds'), 
                        'worlds',
                        LaunchConfiguration('world_file')
                    ])


    # launch desired Gazebo world
    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('ros_gz_sim'), 
                    'launch', 
                    'gz_sim.launch.py'
                ])
            ),
            launch_arguments={
                'gz_args': [world_file_path],
                'on_exit_shutdown': 'True'
            }.items(),
        )
    launch_description.add_action(gazebo_launch)

    return launch_description
