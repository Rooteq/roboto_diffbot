# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 package for a differential drive robot called "roboto_diffbot" that supports both simulation and real hardware operation. The robot features:
- Differential drive locomotion with wheel encoders
- LiDAR sensor integration (SLLIDAR C1)
- Navigation stack with SLAM and autonomous navigation
- Arduino-based hardware interface via serial communication
- GUI integration for remote control and monitoring
- Docking functionality for autonomous charging

## Build System

This package uses the standard ROS2 ament_cmake build system. Build commands should be run from the ROS2 workspace root (`/home/rooteq/ros2_ws`):

```bash
# Build the package
colcon build --packages-select roboto_diffbot

# Build with debug symbols
colcon build --packages-select roboto_diffbot --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Source the workspace after building
source install/setup.bash
```

## Key Launch Files

- `launch_roboto.launch.py`: Main launch file for real robot operation with navigation stack
- `sim_launch.py`: Simulation launch file for Gazebo
- `sim_slam_launch.py`: Simulation with SLAM mapping
- `controls_launch.py`: Robot control and hardware interface
- `navigation_launch.py`: Navigation stack (Nav2)
- `localization_launch.py`: AMCL localization
- `online_async_launch.py`: SLAM mapping

## Hardware Interface

The robot uses a custom hardware interface (`diffbot_system.cpp`) that:
- Communicates with Arduino via serial CAN interface
- Implements ros2_control SystemInterface for wheel control
- Manages wheel encoders and velocity commands
- Configured via `config/controllers.yaml`

## Custom Nodes

- `gui_integration_node`: Lifecycle node providing GUI integration with trigger service and pose publishing
- `detected_dock_pose_publisher`: Publishes detected dock poses for autonomous docking

## Configuration Files

- `config/controllers.yaml`: ros2_control controller configuration
- `config/nav2_params.yaml`: Navigation stack parameters
- `config/mapper_params_online_async.yaml`: SLAM mapping parameters
- `config/dock_database.yaml`: Docking station database
- `config/bridge_params.yaml`: ROS bridge parameters

## Development Commands

From the `some_nodes.txt` file, common development commands include:

```bash
# Teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_key

# SLAM mapping
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/roboto_diffbot/config/mapper_params_online_async.yaml use_sim_time:=true

# Map server
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=./my_map.yaml -p use_sim_time:=true
ros2 run nav2_util lifecycle_bringup map_server

# AMCL localization
ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
ros2 run nav2_util lifecycle_bringup amcl
```

## Robot Description

The robot URDF is defined in `description/robot/robot.urdf.xacro` with:
- Modular xacro files for different components
- Separate configurations for simulation and real hardware
- Integration with ros2_control and Gazebo

## Simulation

Gazebo simulation files are in `sim/` directory:
- `world/world.sdf`: Main simulation world with docking station
- `gazebo/`: Gazebo-specific robot configuration
- Custom meshes and materials for the environment

## Important Notes

- The `use_sim_time` parameter must be set appropriately (true for simulation, false for real robot)
- Hardware interface requires Arduino with CAN communication capability
- Navigation stack uses Nav2 with AMCL for localization
- SLAM mapping uses slam_toolbox with async processing
- GUI integration uses lifecycle nodes with trigger services for state management