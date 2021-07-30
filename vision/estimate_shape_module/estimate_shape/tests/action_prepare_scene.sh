#!/bin/bash

set -e

echo -e "\e[34m* virtualscene set_status stop \e[0m"
ros2 run virtualscene set_status stop

echo -e "\e[34m* virtualscene set_status stop coppelia_brain \e[0m"
ros2 run virtualscene set_status stop coppelia_brain

echo -e "\e[34m* sleep 1 \e[0m"
sleep 1

echo -e "\e[34m* virtualscene clear \e[0m"
ros2 run virtualscene clear

echo -e "\e[34m* virtualscene set_status start \e[0m"
ros2 run virtualscene set_status start

echo -e "\e[34m* virtualscene set_status start coppelia_brain \e[0m"
ros2 run virtualscene set_status start coppelia_brain

echo -e "\e[34m* sleep 1 \e[0m"
sleep 1
