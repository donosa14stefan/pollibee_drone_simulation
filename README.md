# Pollibee Drone Simulation

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2: Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)

## Project Description

Pollibee Drone Simulation is an advanced robotic simulation project that models the behavior of pollination drones in a greenhouse environment. This project utilizes ROS2, Gazebo, and YOLO object detection to create a realistic simulation of drone-assisted pollination in controlled agricultural settings.

## Features

- Realistic drone flight simulation using Gazebo
- Plant growth and pollination state modeling
- YOLO-based object detection for identifying plants and flowers
- ROS2 integration for distributed robotic control
- Data logging and visualization tools

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo 11
- OpenCV 4.x
- YOLO v11

## Installation

1. Clone the repository:```git clone https://github.com/donosa14stefan/pollibee_drone_simulation.git```
2. Navigate to the project directory:```cd pollibee_drone_simulation```
3. Build the project:```colcon build```
4. Source the setup file:```source install/setup.bash```
## Usage

1. Launch the simulation environment:```ros2 launch pollibee_drone_simulation drone.launch.py```
2. Run the drone control node:```ros2 run pollibee_drone_simulation control_drone```
3. Start the YOLO detection:```ros2 run pollibee_drone_simulation yolo_detection```
4. Visualize results:```ros2 run pollibee_drone_simulation visualize_results```

## Configuration

You can modify the simulation parameters in the `config/simulation_params.yaml` file.

## Contributing

We welcome contributions to the Pollibee Drone Simulation project. Please read our [CONTRIBUTING.md](CONTRIBUTING.md) file for details on our code of conduct and the process for submitting pull requests.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ROS2 community
- Gazebo simulation platform
- YOLO object detection framework

## Contact

For any queries, please open an issue or contact the maintainer:

Stefan Donosa - [stefandonosa07@outlook.com](mailto:stefandonosa07@outlook.com)
