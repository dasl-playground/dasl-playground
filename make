#!/bin/bash
colcon build --symlink-install  --packages-select dasl_interface 
source ./install/setup.bash
colcon build --symlink-install
source ./install/setup.bash
