#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Syncs agrobot_interfaces changes with the agrobot-teensy repo and recompiles the micro-ROS library

rsync -avc --delete ~/ros2_ws/src/agrobot_interfaces ~/teensy_ws/agrobot/extra_packages

cd ~/teensy_ws/agrobot
pio run --target clean_microros
pio lib install
pio run
