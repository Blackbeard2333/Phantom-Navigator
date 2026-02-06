from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        # FMU-only topics (SensorGps + VehicleAttitude)
        'tracked_pose_body_topic': '/tracked_uav_pose',
        'attacker_gps_topic': '/fmu/out/vehicle_gps_position',            # px4_msgs/SensorGps
        'attacker_local_topic': '/fmu/out/vehicle_local_position',
        'attacker_attitude_topic': '/fmu/out/vehicle_attitude',# px4_msgs/VehicleAttitude
        'target_lla_topic': '/redirection_commander_gps/target_lla',  # optional
        # Default target (used if no target topic published)
        'default_target_lat_deg': 36.0054164,
        'default_target_lon_deg': -78.9392033,
        'default_target_alt_m': float('nan'),  # use victim estimated alt if NaN
        # Frame id (kept for legacy/debug; not used for the three requested outputs)
        'map_frame_id': 'map',
    }

    # If your ROS2-PX4 bridge exposes GPS under a different topic, remap here.
    # For example:
    # remappings = [
    #     ('/fmu/out/sensor_gps', '/fmu/out/vehicle_gps_position'),
    # ]
    remappings = []

    return LaunchDescription([
        Node(
            package='redirection_commander',
            executable='redirection_commander_gps',
            name='redirection_commander_gps',
            output='screen',
            parameters=[params],
            remappings=remappings,
        )
    ])
