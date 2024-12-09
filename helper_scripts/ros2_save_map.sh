#!/bin/bash

MAP_DIR=~/ros2_ws/src/roboto_diffbot/sim/map
MAP_NAME=my_map
mkdir -p "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME" --ros-args -p map_subscribe_transient_local:=true

if [ $? -eq 0 ]; then
    echo "Saved map at $MAP_DIR/$MAP_NAME"
else
    echo "Error while saving map"
fi