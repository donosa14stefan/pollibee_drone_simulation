# PollieBee Drone Simulation Plugins

## Overview

This directory contains the essential plugins for the PollieBee pollination drone simulation. These plugins are designed to provide realistic and comprehensive simulation capabilities, covering various aspects of drone operation, environmental factors, and mission-specific functionalities.

## Directory Structure

The plugins/ directory is organized as follows:

gazebo/
  - battery_plugin/
    - battery_plugin.cpp
    - battery_plugin.h
  - wind_plugin/
    - wind_plugin.cpp
    - wind_plugin.h
  - gazebo_plugin.cpp
  - gazebo_plugin.h

ros/
  - mission_planner_plugin/
    - mission_planner_plugin.cpp
    - mission_planner_plugin.h
  - pollination_control_plugin/
    - pollination_control_plugin.cpp
    - pollination_control_plugin.h
  - ros_plugin.cpp
  - ros_plugin.h

rviz/
  - rviz_plugin.cpp
  - rviz_plugin.h

yolo/
  - yolo_plugin.cpp
  - yolo_plugin.h
  - yolov11.cfg
  - yolo_config.json

## Detailed Plugin Descriptions

### Gazebo Plugins

1. **Battery Plugin** (battery_plugin.cpp, battery_plugin.h)
   - Simulates realistic battery behavior
   - Tracks power consumption based on drone operations
   - Implements battery discharge and recharge cycles
   - Provides low battery warnings and automated return-to-home functionality

2. **Wind Plugin** (wind_plugin.cpp, wind_plugin.h)
   - Simulates variable wind conditions
   - Affects drone flight dynamics realistically
   - Allows for configuration of wind patterns and intensities

3. **Gazebo Plugin** (gazebo_plugin.cpp, gazebo_plugin.h)
   - Main interface for integrating custom functionalities with Gazebo
   - Manages physics interactions and world updates

### ROS Plugins

1. **Mission Planner Plugin** (mission_planner_plugin.cpp, mission_planner_plugin.h)
   - Implements high-level mission planning algorithms
   - Generates optimal flight paths for pollination missions
   - Handles dynamic replanning based on environmental factors and flower detection

2. **Pollination Control Plugin** (pollination_control_plugin.cpp, pollination_control_plugin.h)
   - Manages the pollination process
   - Integrates with flower detection systems
   - Controls pollination mechanism activation and deactivation

3. **ROS Plugin** (ros_plugin.cpp, ros_plugin.h)
   - Facilitates communication between Gazebo simulation and ROS
   - Publishes simulation data to ROS topics
   - Subscribes to ROS commands for drone control

### RViz Plugin (rviz_plugin.cpp, rviz_plugin.h)
- Provides custom visualization tools for the PollieBee simulation in RViz
- Displays drone status, flight path, detected flowers, and environmental data

### YOLO Plugin (yolo_plugin.cpp, yolo_plugin.h)
- Implements YOLO (You Only Look Once) object detection algorithm
- Specialized for detecting and localizing flowers in the simulated environment
- Uses yolov11.cfg for network configuration and yolo_config.json for detection parameters

## Usage Instructions

To integrate these plugins into your PollieBee drone simulation:

1. Dependencies:
   - Ensure Gazebo (version X.X or higher) is installed
   - ROS (version X.X or higher) must be set up
   - OpenCV (version X.X) is required for image processing
   - CUDA (optional, for GPU acceleration of YOLO detection)

2. Configuration:
   - Add the plugin paths to your Gazebo model files
   - Configure plugin parameters in the respective config files (e.g., yolo_config.json)

3. Launching the Simulation:
   - Use the provided launch file: `roslaunch polliebee_simulation full_simulation.launch `
   - This will start the Gazebo simulation with all plugins enabled

## Contributing

Contributions to improve these plugins are welcome. To contribute:

1. Fork the repository
2. Create a new branch for your feature or bug fix
3. Make your changes and test them thoroughly
4. Create a pull request with a detailed description of your changes

## License

These plugins are distributed under a custom license agreement. Please refer to the LICENSE file in the project's root directory for the complete terms and conditions of use, modification, and distribution of these plugins.

Note: Use of these plugins in commercial projects or distribution outside of the PollieBee project requires explicit permission from the copyright holders.
