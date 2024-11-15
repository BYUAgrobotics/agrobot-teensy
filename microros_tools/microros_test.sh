#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Tests each of the expected micro-ROS topics
# - Use this after setting up a new PCB to test the agent and Teensy board connections

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

cleanup() {
  killall micro_ros_agent
  wait
  exit 0
}
trap cleanup SIGINT

if [ -z "$(tycmd list | grep Teensy)" ]; then
  printError "No Teensy boards avaliable to connect to"
  exit 1
else 
  source ~/microros_ws/install/setup.bash
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &

  sleep 5
fi

source ~/ros2_ws/install/setup.bash

echo ""
ros2 topic list # TODO: Make this more specific
echo ""

cleanup
