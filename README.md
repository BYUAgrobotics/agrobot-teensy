This repo contains micro-ROS dev environments and bash scripts used by the BYU FRoSt lab's **Cooperative Underwater Group of Autonomous Robots (CoUGARs)**. 
If you're doing code development, feel free to clone this repo directly, make a new branch, and start coding and pushing some changes.
Otherwise (if you're looking to set up a new Coug-UV from scratch), see this repo for instructions on how to get our custom Docker image running instead: https://github.com/BYU-FRoSt-Lab/CoUGARs

A quick high-level overview of the repo:
- **cougars/** - PlatformIO dev environment for the Teensy.
Source code and header files are included in "cougars/src/" and "cougars/include/", and dependencies can be imported by modifying the "cougars/platformio.ini" file.
Different sensors and actuators can be enabled/disabled by commenting out the #define statements at the top of "cougars/src/main.cpp" and rebuilding.
- **firmware_options/** - pre-compiled firmware hex files that can be uploaded to the vehicles.
Custom hex files added to this directory can be uploaded to the Teensy boards using the "upload.sh" script.
- **microros_tools/** - helpful scripts for launching the microROS agent ("agent.sh"), syncing custom message changes ("msg_sync.sh"), and testing Teensy board connections ("test.sh").
TODO: Add a more helpful description here in the future.
- **scripts**
A description of what each bash script does is included as a header comment in the file.
