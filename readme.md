# Phantom-Navigator
## Submission Artifact for USENIX-2026

## Real-world Experiments Pipeline
## Sensor bringups
### Building and sourcing
```
colcon build
source install/setup.bash
```

### Activate the python virtual environment
```
cd ~/Documents/CPSL_UAV_Tracking
source uav_perception/bin/activate
```

### Launch the Realsense ROS2 Node
```
cd ~/Documents/realsense
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

### Access the lidar point cloud
```
/cpsl_uav_1/livox/lidar
```

### Launch the Lidar ROS2 Node

```
cd CPSL_ROS2_Sensors
source install/setup.bash
ros2 launch cpsl_ros2_sensors_bringup uav_sensor_bringup.launch.py lidar_enable:=true lidar_scan_enable:=true camera_enable:=false radar_enable:=false platform_description_enable:=true rviz:=true namespace:=cpsl_uav_1
```

### Launch the YOLO detector ROS2 Node
```
ros2 run visual_detection yolo_detector
```

### Launch the Detector ROS2 Node
```
colcon build --packages-select camera_lidar_fusion
source install/setup.bash
ros2 launch camera_lidar_fusion fusion_frustum_filter_launch.py
ros2 run camera_lidar_fusion point_cloud_tracker
```

### Change PX4 namespace - QGC-Analyze Tools - Mavlink Console
```
uxrce_dds_client stop
uxrce_dds_client start -n cpsl_uav_1
```

### A tmux command that runs the above
```
~/Documents/CPSL_UAV_Tracking/scripts/run_uav_tracker_stack.sh
tmux ls
tmux kill-session -t UAV_TRACKER
```

```
~/Documents/CPSL_UAV_Tracking/scripts/run_uav_tracker_stack_px4.sh
tmux ls
tmux kill-session -t UAV_TRACKER_PX4
```

  #### Tmux tips
    Switch windows: Ctrl-b n / Ctrl-b p (or Ctrl-b then the window number).
    Detach without stopping anything: Ctrl-b d.
    Reattach later: tmux attach -t UAV_TRACKER.
    Stop everything at once: detach, then tmux kill-session -t UAV_TRACKER.


## UAV-Redirection
```
cd ~/Documents/CPSL_UAV_Tracking
ros2 launch redirection_commander redirection_commander.launch.py
ros2 launch redirection_commander redirection_commander_gps.launch.py

ros2 run redirection_commander redirection_socket_server 
```

## PX4 Control Pipeline
### Building and sourcing
```
colcon build --packages-select px4_controller
```

### Connect PX4 to ROS2
```
MicroXRCEAgent udp4 -p 8888
```

### Launch the PX4 control node
```
cd CPSL_ROS2_PX4
source install/setup.bash
ros2 launch px4_controller joy_control_launch.py joy_enable:=false control_enable:=true namespace:=cpsl_uav_1
```

### Lanuch the PX4 vicon bridge node
```
ros2 launch cpsl_px4_bridge vicon_to_px4_ev.launch.py
```

### Spoofing PX4 vicon bridge
```
ros2 launch cpsl_px4_bridge spoof_vicon_to_px4_ev.launch.py
```

### Launch the victim waypoint controller
```
ros2 launch cpsl_px4_vicon_controller waypoint_mission_launch.py   params_file:=/home/cpsl/Documents/CPSL_ROS2_PX4/src/cpsl_px4_vicon_controller/cpsl_px4_vicon_controller/config/mission_example.yaml
```
