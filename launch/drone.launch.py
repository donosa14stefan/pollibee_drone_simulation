# File: launch/drone_simulation.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('pollibee_drone_simulation')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    world_file = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_dir, 'worlds', 'greenhouse.world'),
        description='Full path to the Gazebo world file to simulate'
    )

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        launch_arguments={
            'world': LaunchConfiguration('world_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Launch drone control node
    drone_control = Node(
        package='pollibee_drone_simulation',
        executable='control_drone',
        name='drone_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch YOLO detection node
    yolo_detection = Node(
        package='pollibee_drone_simulation',
        executable='yolo_detection',
        name='yolo_detector',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch pollination motor node
    pollination_motor = Node(
        package='pollibee_drone_simulation',
        executable='pollination_motor',
        name='pollinator',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch visualization node
    visualize_results = Node(
        package='pollibee_drone_simulation',
        executable='visualize_results',
        name='visualizer',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch data logging node
    data_logging = Node(
        package='pollibee_drone_simulation',
        executable='data_logging',
        name='data_logger',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time,
        world_file,
        gazebo,
        drone_control,
        yolo_detection,
        pollination_motor,
        visualize_results,
        data_logging
    ])
