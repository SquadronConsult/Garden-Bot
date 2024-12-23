from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    garden_bot_path = get_package_share_directory('garden_bot')
    world_file = os.path.join(garden_bot_path, 'gazebo', 'worlds', 'testing.world')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_path, 'launch', 'empty_world.launch.py')),
            launch_arguments={'world': world_file}.items(),
        ),
    ])

