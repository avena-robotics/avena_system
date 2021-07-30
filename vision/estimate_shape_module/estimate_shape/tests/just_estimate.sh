#!/bin/bash

set -e

label=$1
# amount=$2
echo "Test case: $label"


echo -e "\e[34m* scene_publisher \e[0m"

ros2 run cli scene_publisher
sleep 1

echo -e "\e[34m* detect \e[0m"
ros2 run cli detect
sleep 1

echo -e "\e[34m* compose_items \e[0m"
ros2 run cli filter_detections
sleep 1

echo -e "\e[34m* compose_items \e[0m"
ros2 run cli compose_items
sleep 1

echo -e "\e[34m* estimate_shape \e[0m"
ros2 run cli estimate_shape
sleep 1

echo -e "\e[34m* get_occupancy_grid \e[0m"
ros2 run cli get_occupancy_grid
sleep 1

echo -e "\e[34m* item_select $label operating_area include nearest\e[0m"
ros2 run cli item_select $label whole include nearest
sleep 1

echo -e "\e[34m* octomap_filter \e[0m"
ros2 run cli octomap_filter
sleep 1

echo -e "\e[34m* spawn_collision_items \e[0m"
ros2 run cli spawn_collision_items
sleep 1

exit 0
