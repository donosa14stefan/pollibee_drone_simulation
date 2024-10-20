# Scripts Directory

This directory contains the Python scripts for the PollieBee pollination drone simulation. Below is a detailed description of each script and its main functionalities.

## main.py
- Main entry point for the simulation
- Initializes all modules
- Manages execution flow

## control_drone.py
- Implements basic drone control logic
- Includes vibration arm control
- Adjusts trajectory based on wind conditions

## yolo_detection.py
- Implements YOLOv8 for object detection
- Processes camera images to detect flowers and obstacles

## pollination_motor.py
- Controls the pollination mechanism
- Manages activation and deactivation of the pollination arm

## trajectory_planner.py
- Plans optimal trajectories for the drone
- Avoids obstacles and optimizes for efficient pollination

## communication_handler.py
- Manages communication between the drone and base station
- Implements message encryption and acknowledgment system

## data_logging.py
- Logs all relevant simulation data
- Includes YOLOv8 performance logging and data compression

## battery_monitor.py
- Monitors drone battery levels
- Predicts remaining flight time based on current conditions

## obstacle_avoidance.py
- Implements obstacle detection and avoidance algorithms
- Integrates data from multiple sensors for robust detection

## wind_simulation.py
- Simulates wind conditions
- Includes turbulence models for realistic wind effects

## mission_planner.py
- High-level mission planning and management
- Handles different mission types (survey, targeted pollination, etc.)

## sensor_fusion.py
- Fuses data from multiple sensors
- Implements Kalman or particle filter for state estimation

## error_handling.py
- Centralizes error and exception handling
- Implements recovery mechanisms for various error scenarios

## performance_metrics.py
- Calculates and reports performance metrics
- Includes specific metrics for flower detection and pollination efficiency

## simulation_interface.py
- Interfaces with the Gazebo simulation environment
- Controls simulation parameters and extracts data

## Usage

To run the simulation:

1. Ensure all dependencies are installed
2. Launch the Gazebo simulation environment
3. Run `main.py`

For detailed usage of individual scripts, refer to the docstrings within each file.

## Dependencies

- ROS Noetic
- Python 3.8+
- YOLOv8
- OpenCV
- NumPy
- Matplotlib

## Contributing

Please refer to the main project README for contribution guidelines.

## License

This project is licensed under the MIT License - see the LICENSE file in the main project directory for details.
