import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'pollibee_drone_simulation'
    package_share_dir = get_package_share_directory(package_name)

    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_dir, 'launch', 'drone.launch.py')])
    )

    wind_simulation = Node(
        package=package_name,
        executable='wind_simulation.py',
        name='wind_simulation',
        output='screen'
    )

    battery_management = Node(
        package=package_name,
        executable='battery_management.py',
        name='battery_management',
        output='screen'
    )

    return LaunchDescription([
        drone_launch,
        wind_simulation,
        battery_management
    ])
