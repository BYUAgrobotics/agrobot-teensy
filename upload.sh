#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Uploads hex files to the Teensy 4.1 board
# - Specify a file in firmware_options using 'bash 
#   upload.sh <file.hex>'
# - If this fails, check the USB connections and the
#   current teensy power states

sudo bash ~/gpio/power.sh on

case $1 in
    "")
        sudo python3 ~/gpio/gpio_tools/program.py

        cd ~/teensy_ws/cougars/.pio/build/teensy41
        tycmd upload firmware.hex
        ;;
    *)
        sudo python3 ~/gpio/gpio_tools/program.py

        cd ~/teensy_ws/firmware_options
        tycmd upload $1
        ;;
esac
