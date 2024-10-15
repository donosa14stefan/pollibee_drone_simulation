cmake_minimum_required(VERSION 3.5)

project(pollibee_drone_simulation)

# Set the ROS 2 version
set(ROS2_VERSION "humble")

# Find the ROS 2 package
find_package(ros2 ${ROS2_VERSION} REQUIRED)

# Include the ROS 2 CMake module
include(${ros2_CMAKE_MODULE_PATH}/ros2.cmake)

# Set the Gazebo version
set(GAZEBO_VERSION "11.10.0")

# Find the Gazebo package
find_package(gazebo ${GAZEBO_VERSION} REQUIRED)

# Include the Gazebo CMake module
include(${gazebo_CMAKE_MODULE_PATH}/gazebo.cmake)

# Set the YOLO version
set(YOLO_VERSION "11")

# Find the YOLO package
find_package(yolo ${YOLO_VERSION} REQUIRED)

# Include the YOLO CMake module
include(${yolo_CMAKE_MODULE_PATH}/yolo.cmake)

# Add the executable for the drone control script
add_executable(control_drone scripts/control_drone.py)

# Add the executable for the YOLO detection script
add_executable(yolo_detection scripts/yolo_detection.py)

# Add the executable for the pollination motor script
add_executable(pollination_motor scripts/pollination_motor.py)

# Add the executable for the visualization script
add_executable(visualize_results scripts/visualize_results.py)

# Add the executable for the data logging script
add_executable(data_logging scripts/data_logging.py)

# Add the plugin for Gazebo
add_library(gazebo_plugin SHARED plugins/gazebo/gazebo_plugin.cpp)
target_link_libraries(gazebo_plugin ${GAZEBO_LIBRARIES})

# Add the plugin for ROS
add_library(ros_plugin SHARED plugins/ros/ros_plugin.cpp)
target_link_libraries(ros_plugin ${ROS2_LIBRARIES})

# Add the plugin for RViz
add_library(rviz_plugin SHARED plugins/rviz/rviz_plugin.cpp)
target_link_libraries(rviz_plugin ${RVIZ_LIBRARIES})

# Add the plugin for YOLO
add_library(yolo_plugin SHARED plugins/yolo/yolo_plugin.cpp)
target_link_libraries(yolo_plugin ${YOLO_LIBRARIES})

# Add the plugin for Python
add_library(python_plugin SHARED plugins/python/python_plugin.cpp)
target_link_libraries(python_plugin ${PYTHON_LIBRARIES})

# Set the installation directory
set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})

# Install the executables and plugins
install(TARGETS control_drone yolo_detection pollination_motor visualize_results data_logging
        RUNTIME DESTINATION ${INSTALL_DIR})
install(TARGETS gazebo_plugin ros_plugin rviz_plugin yolo_plugin python_plugin
        LIBRARY DESTINATION ${INSTALL_DIR})
