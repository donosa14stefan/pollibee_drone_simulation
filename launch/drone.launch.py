import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'pollibee_drone_simulation'
    package_share_dir = get_package_share_directory(package_name)

    # Calea către fișierul world
    world_file = os.path.join(package_share_dir, 'worlds', 'pollibee_world.world')

    # Calea către modelul dronei
    drone_model = os.path.join(package_share_dir, 'models', 'drone_model', 'model.sdf')

    # Lansarea Gazebo cu world-ul nostru
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # Spawning the drone
    spawn_drone = Node(package='gazebo_ros', executable='spawn_entity.py',
                       arguments=['-entity', 'pollibee_drone',
                                  '-file', drone_model,
                                  '-x', '0', '-y', '0', '-z', '1'],
                       output='screen')

    # Nodurile tale
    control_drone = Node(
        package=package_name,
        executable='control_drone.py',
        name='control_drone',
        output='screen'
    )

    yolo_detection = Node(
        package=package_name,
        executable='yolo_detection.py',
        name='yolo_detection',
        output='screen'
    )

    pollination_motor = Node(
        package=package_name,
        executable='pollination_motor.py',
        name='pollination_motor',
        output='screen'
    )

    visualize_results = Node(
        package=package_name,
        executable='visualize_results.py',
        name='visualize_results',
        output='screen'
    )

    data_logging = Node(
        package=package_name,
        executable='data_logging.py',
        name='data_logging',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_drone,
        control_drone,
        yolo_detection,
        pollination_motor,
        visualize_results,
        data_logging
    ])
