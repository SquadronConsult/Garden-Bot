from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Specify world file name'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
    ]

    # Initialize Arguments
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the package share directory for ros_gz_sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set the path to the world file
    world_file = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'worlds', world]
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py']
        ),
        launch_arguments={'gz_args': world_file}.items(),
    )

    # Define the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)

    return ld