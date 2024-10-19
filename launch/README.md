# Launch Files for PollieBee Drone Simulation

This folder contains launch files for the PollieBee Drone Simulation project. These files are used to start various components of the simulation.

## Available Launch Files

1. `drone.launch.py`: Launches the basic drone simulation in Gazebo.
2. `rviz.launch.py`: Launches RViz for visualization.
3. `simulation.launch.py`: Launches the complete simulation, including additional nodes.

## Usage

To launch the simulation, use the following command:
```ros2 launch pollibee_drone_simulation simulation.launch.py```

### Optional Arguments

- `world_file`: Path to the Gazebo world file (default: pollibee_world.world)
- `drone_model`: Path to the drone model file (default: drone_model/model.sdf)
- `debug`: Enable debug mode (default: false)
- `rviz`: Launch RViz for visualization (default: true)

Example with custom arguments:
```ros2 launch pollibee_drone_simulation simulation.launch.py world_file:=custom_world.world debug:=true```

## Configuration

- Adjust parameters in the launch files to customize the simulation setup.
- Modify the RViz configuration file (`config/pollibee_rviz.rviz`) to change visualization settings.

## Troubleshooting

If you encounter issues while launching the simulation:

1. Ensure all dependencies are installed.
2. Check that paths to world and model files are correct.
3. Verify that all required nodes are present in the package.

For more detailed information, refer to the main project README or contact the development team.
