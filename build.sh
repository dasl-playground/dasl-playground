#!/bin/bash



colcon build --packages-select --symlink-install dasl_playground
source ~/dev_ws/install/setup.bash
colcon build --symlink-install

