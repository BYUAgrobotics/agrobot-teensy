#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Syncs frost_interfaces changes with the cougars-teensy repo
# and recompiles the micro-ROS library

rsync -avc --delete ~/ros2_ws/src/frost_interfaces ~/teensy_ws/cougars/extra_packages

cd ~/teensy_ws/cougars
pio run --target clean_microros
pio lib install
pio run
