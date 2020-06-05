#!/bin/bash

colcon build --packages-select dasl_playground
source /home/dasl-vision/dev_ws/install/setup.bash
colcon build 

