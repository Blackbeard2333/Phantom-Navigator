#!/usr/bin/env bash
set -euo pipefail

SESSION="UAV_TRACKER"

# Always restart (kill old session if it exists)
tmux kill-session -t "${SESSION}" 2>/dev/null || true

# Helper to create tmux window running a full bash login/rc so sourcing works reliably
new_win () {
  local name="$1"
  shift
  tmux new-window -t "${SESSION}" -n "${name}" "bash -lc '$*'; exec bash"
}

# Start tmux session with the first window
tmux has-session -t "${SESSION}" 2>/dev/null && {
  echo "Session ${SESSION} already exists. Attach with: tmux attach -t ${SESSION}"
  exit 0
}

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

echo "Launched tmux session: ${SESSION}"
echo "Attach with: tmux attach -t ${SESSION}"
tmux attach -t "${SESSION}"
