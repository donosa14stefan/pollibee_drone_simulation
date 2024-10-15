cmake_minimum_required(VERSION 3.8)
project(pollibee_drone_simulation)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Find YOLO (assuming it's installed in a standard location)
find_package(YOLO REQUIRED)

# Declare the drone control executable
add_executable(control_drone src/control_drone.cpp)
ament_target_dependencies(control_drone rclcpp std_msgs geometry_msgs)

# Declare the YOLO detection executable
add_executable(yolo_detection src/yolo_detection.cpp)
ament_target_dependencies(yolo_detection rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(yolo_detection ${OpenCV_LIBS} ${YOLO_LIBRARIES})

# Declare the pollination motor executable
add_executable(pollination_motor src/pollination_motor.cpp)
ament_target_dependencies(pollination_motor rclcpp std_msgs)

# Declare the visualization executable
add_executable(visualize_results src/visualize_results.cpp)
ament_target_dependencies(visualize_results rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(visualize_results ${OpenCV_LIBS})

# Declare the data logging executable
add_executable(data_logging src/data_logging.cpp)
ament_target_dependencies(data_logging rclcpp std_msgs)

# Gazebo plugin
add_library(gazebo_plugin SHARED plugins/gazebo/gazebo_plugin.cpp)
ament_target_dependencies(gazebo_plugin rclcpp gazebo_ros_pkgs)

# ROS plugin
add_library(ros_plugin SHARED plugins/ros/ros_plugin.cpp)
ament_target_dependencies(ros_plugin rclcpp)

# RViz plugin
add_library(rviz_plugin SHARED plugins/rviz/rviz_plugin.cpp)
ament_target_dependencies(rviz_plugin rclcpp rviz_common)

# Install executables and libraries
install(TARGETS
  control_drone
  yolo_detection
  pollination_motor
  visualize_results
  data_logging
  gazebo_plugin
  ros_plugin
  rviz_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

catkin_install_python(PROGRAMS
  scripts/yolo_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
