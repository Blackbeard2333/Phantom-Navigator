from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_lidar_fusion',
            executable='projector',
            name='fusion_projector',
            output='screen',
            parameters=[]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_to_camera_tf',
            arguments=[
                '0.07', '0.0', '-0.04',  # Translation: lidar is behind and below camera
                '1', '0', '0', '0',  # Rotation: FLU to optical frame
                'cpsl_uav_1/livox_frame', 'camera_link'
            ], 
            output='screen'
        ),
    ])
