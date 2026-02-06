#!/bin/bash
source ~/Documents/CPSL_UAV_Tracking/uav_perception/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/Documents/CPSL_UAV_Tracking/install/setup.bash
ros2 launch uav_tracking uav_tracking_launch.py
