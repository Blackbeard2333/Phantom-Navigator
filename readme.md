# Phantom-Navigator
# Visual-Lidar UAV Tracking and Spoofing Pipeline

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

### Launch the waypoint controller
```
ros2 launch cpsl_px4_vicon_controller waypoint_mission_launch.py   params_file:=/home/cpsl/Documents/CPSL_ROS2_PX4/src/cpsl_px4_vicon_controller/cpsl_px4_vicon_controller/config/mission_example.yaml
```

## Recording ROS2 topics
```
/camera/camera/color/image_raw
/visual_detection/bbox_image
/cpsl_uav_1/livox/lidar
/camera_lidar_fusion/frustum_filtered
/camera_lidar_fusion/uav_cluster
/tracked_uav_cluster
/tracked_uav_pose
/tracked_uav_velocity
/vicon/x500_3/x500_3
/vicon/x500_4/x500_4
```

```
ros2 bag record -o $YOUR_BAG_NAME /tf /tf_static /visual_detection/bbox_image /tracked_uav_pose /tracked_uav_cluster /tracked_uav_velocity /spoofing_accel_cmd /spoofing_velocity_cmd /spoofing_position_cmd /vicon/x500_3/x500_3 /vicon/x500_4/x500_4 
```

```
ros2 bag record -o rosbag_100801 /tf /tf_static /cpsl_uav_1/livox/lidar /camera/camera/color/image_raw /camera/camera/color/camera_info /visual_detection/detections /visual_detection/bbox_image /camera_lidar_fusion/frustum_filtered /camera_lidar_fusion/uav_cluster /tracked_uav_pose /tracked_uav_cluster /tracked_uav_velocity /tracked_uav_pose_vicon /spoofing_position_cmd_vicon /vicon/x500_3/x500_3 /vicon/x500_4/x500_4 
```

### Collect attack results (no perception data)
ros2 bag record -o rosbag_102201_5_attacks /tf /tf_static /cpsl_uav_4/fmu/out/vehicle_odometry /tracked_uav_pose_vicon /spoofing_position_cmd_vicon /redirection_commander/target /vicon/x500_3/x500_3 /vicon/x500_4/x500_4


### Collect attack results with perception data
ros2 bag record -o rosbag_102301_track_attack /tf /tf_static /cpsl_uav_1/livox/lidar /camera/camera/color/image_raw /camera/camera/color/camera_info /visual_detection/detections /visual_detection/bbox_image /camera_lidar_fusion/frustum_filtered /camera_lidar_fusion/uav_cluster /tracked_uav_pose /tracked_uav_cluster /tracked_uav_velocity /tracked_uav_pose_vicon /spoofing_position_cmd_vicon /vicon/x500_3/x500_3 /vicon/x500_4/x500_4 /cpsl_uav_4/fmu/out/vehicle_odometry

### Collect outdoor tracking dataset
ros2 bag record -o rosbag_110403_outdoor_track /tf /tf_static /cpsl_uav_1/livox/lidar /camera/camera/color/image_raw /camera/camera/color/camera_info /visual_detection/detections /visual_detection/bbox_image /camera_lidar_fusion/frustum_filtered /camera_lidar_fusion/uav_cluster /tracked_uav_pose /tracked_uav_cluster /tracked_uav_velocity /fmu/out/vehicle_gps_position /fmu/out/vehicle_local_position /fmu/out/vehicle_odometry /cpsl_uav_4/fmu/out/vehicle_gps_position /cpsl_uav_4/fmu/out/vehicle_local_position /cpsl_uav_4/fmu/out/vehicle_odometry 


### Outdoor tracking and spoofing
ros2 bag record -o rosbag_110901_track_and_spoof_01 /tf /tf_static /cpsl_uav_1/livox/lidar /camera/camera/color/image_raw /camera/camera/color/camera_info /visual_detection/detections /visual_detection/bbox_image /camera_lidar_fusion/frustum_filtered /camera_lidar_fusion/uav_cluster /tracked_uav_pose /tracked_uav_cluster /tracked_uav_velocity /fmu/out/vehicle_gps_position /fmu/out/vehicle_local_position /fmu/out/vehicle_odometry /fmu/out/vehicle_attitude /victim/estimated_sensor_gps /victim/estimated_local_position /spoofing_gps_bias/latlon /spoofing_gps_bias/ned


## ros domian id
export ROS_DOMAIN_ID=11