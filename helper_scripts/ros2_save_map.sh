#!/bin/bash

# Define the directory to save the map
MAP_DIR=~/ros2_ws/src/roboto_diffbot/sim/map

# Define the map name
MAP_NAME=dupa

# Create the directory if it doesn't exist
mkdir -p "$MAP_DIR"

# Save the map using the nav2_map_server command
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME" --ros-args -p map_subscribe_transient_local:=true

# Check if the command was successful (based on $? exit code)
if [ $? -eq 0 ]; then
    echo "Map saved successfully at $MAP_DIR/$MAP_NAME"
else
    echo "Failed to save the map."
fi