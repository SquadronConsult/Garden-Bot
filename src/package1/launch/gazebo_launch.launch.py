from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():
    # Get the package share directory for package1
    pkg_package1 = get_package_share_directory('package1')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='testing.world',
        description='World to load into Gazebo'
    )

    declare_world_sdf_file_cmd = DeclareLaunchArgument(
        'world_sdf_file',
        default_value=PathJoinSubstitution([pkg_package1, 'worlds', 'testing.world']),
        description='Path to the SDF world file'
    )

    declare_world_sdf_string_cmd = DeclareLaunchArgument(
        'world_sdf_string',
        default_value='',
        description='SDF world string'
    )

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name',
        default_value='default_bridge',
        description='Name of the ROS-Gazebo bridge'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_package1, 'config', 'bridge_config.yaml']),
        description='Path to the bridge configuration file'
    )

    # Set the environment variable for Gazebo model path
    gz_model_path = PathJoinSubstitution([pkg_package1, 'gazebo', 'models'])
    set_gz_model_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path)

    # Gazebo server action
    gz_server_action = GzServer(
        world_sdf_file=LaunchConfiguration('world_sdf_file'),
        world_sdf_string=LaunchConfiguration('world_sdf_string'),
    )

    # ROS-Gazebo bridge action
    ros_gz_bridge_action = RosGzBridge(
        bridge_name=LaunchConfiguration('bridge_name'),
        config_file=LaunchConfiguration('config_file'),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_world_sdf_file_cmd)
    ld.add_action(declare_world_sdf_string_cmd)
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(set_gz_model_path)

    # Add the actions to launch all of the bridge + gz_server nodes
    ld.add_action(gz_server_action)
    ld.add_action(ros_gz_bridge_action)

    return ld