from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    package_name = 'pollibee_drone_simulation'
    
    return LaunchDescription([
        Node(
            package=package_name,
            executable='pollination_planner.py',
            name='pollination_planner',
            output='screen',
            parameters=[
                {'field_size': [10, 10, 2]},
                {'grid_size': 1.0},
                {'hover_height': 1.5}
            ]
        ),
        Node(
            package=package_name,
            executable='drone_controller.py',
            name='drone_controller',
            output='screen',
            parameters=[
                {'max_velocity': 1.0},
                {'position_tolerance': 0.1}
            ]
        ),
        Node(
            package=package_name,
            executable='flower_detection.py',
            name='flower_detection',
            output='screen'
        ),
        Node(
            package=package_name,
            executable='yolo_detector.py',
            name='yolo_detector',
            output='screen',
            parameters=[
                {'yolo_config': FindPackageShare(package=package_name).find(package_name) + '/config/yolov3.cfg'},
                {'yolo_weights': FindPackageShare(package=package_name).find(package_name) + '/config/yolov3.weights'},
                {'yolo_classes': FindPackageShare(package=package_name).find(package_name) + '/config/coco.names'},
                {'confidence_threshold': 0.5},
                {'nms_threshold': 0.4}
            ]
        ),
        # Adaugă restul nodurilor în mod similar
    ])
