#!/usr/bin/env bash
set -euo pipefail

SESSION="UAV_TRACKER_PX4"

# Always restart (kill old session if it exists)
tmux kill-session -t "${SESSION}" 2>/dev/null || true

# Helper to create tmux window
new_win () {
  local name="$1"
  shift
  tmux new-window -t "${SESSION}" -n "${name}" "bash -lc '$*'; exec bash"
}

# --- OPTIONAL: If you need your base ROS env, uncomment the next line in each window ---
# source /opt/ros/jazzy/setup.bash &&

# ============ Core sensors / perception (same as your original) ============

# Realsense
tmux new-session -d -s "${SESSION}" -n "realsense" "bash -lc '
  cd ~/Documents/realsense &&
  source install/setup.bash &&
  ros2 launch realsense2_camera rs_launch.py
'; exec bash"

# Sensors bringup
new_win "sensors" "
  cd ~/Documents/CPSL_ROS2_Sensors &&
  source install/setup.bash &&
  ros2 launch cpsl_ros2_sensors_bringup uav_sensor_bringup.launch.py \
    lidar_enable:=true lidar_scan_enable:=true camera_enable:=false \
    radar_enable:=false platform_description_enable:=true rviz:=true \
    namespace:=cpsl_uav_1
"

# Visual detection (YOLO)
new_win "yolo" "
  cd ~/Documents/CPSL_UAV_Tracking &&
  source uav_perception/bin/activate &&
  source install/setup.bash &&
  ros2 run visual_detection yolo_detector
"

# Fusion / frustum filter launch
new_win "fusion" "
  cd ~/Documents/CPSL_UAV_Tracking &&
  source uav_perception/bin/activate &&
  source install/setup.bash &&
  ros2 launch camera_lidar_fusion fusion_frustum_filter_launch.py
"

# Point cloud tracker
new_win "tracker" "
  cd ~/Documents/CPSL_UAV_Tracking &&
  source uav_perception/bin/activate &&
  source install/setup.bash &&
  ros2 run camera_lidar_fusion point_cloud_tracker
"

# ============ PX4 / micro-ROS ============

# Micro XRCE Agent (DDS bridge)
new_win "microxrceagent" "
  MicroXRCEAgent udp4 -p 8888
"

# # PX4 Controller (give agent a moment to bind)
# new_win "px4_ctrl" "
#   cd ~/Documents/CPSL_ROS2_PX4 &&
#   source install/setup.bash &&
#   sleep 2 &&
#   ros2 launch px4_controller joy_control_launch.py \
#     joy_enable:=false control_enable:=true namespace:=cpsl_uav_1
# "

# PX4 Vicon Bridge
# new_win "px4_vicon_odom" "
#   cd ~/Documents/CPSL_ROS2_PX4 &&
#   source install/setup.bash &&
#   ros2 launch cpsl_px4_bridge vicon_to_px4_ev.launch.py
# "

echo "Started fresh tmux session with PX4: ${SESSION}"
tmux attach -t "${SESSION}"
