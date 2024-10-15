cmake_minimum_required(VERSION 3.8)
project(pollibee_drone_simulation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/control_drone.py
  scripts/data_logging.py
  scripts/pollination_motor.py
  scripts/visualize_results.py
  scripts/yolo_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install models
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY
  world
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
