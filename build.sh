#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Builds the firmware.hex file from the PIO workspace

# Not needed, but gets rid of random warning message
/home/agrobot/.platformio/penv/bin/python -m pip install --upgrade pip

cd ~/teensy_ws/agrobot
pio run
