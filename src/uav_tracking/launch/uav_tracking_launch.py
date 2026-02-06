from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_tracking',
            executable='detector',
            name='detector',
            output='screen'
        ),
        Node(
            package='uav_tracking',
            executable='tracker',
            name='tracker',
            output='screen'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_camera_to_base',
        #     arguments=['0.10', '0', '-0.10', '0', '0', '0', 'cpsl_uav_1/base_link', 'camera_link'],
        #     output='screen'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_to_lidar_tf',
        #     arguments=[
        #         '-0.07', '0.0', '-0.04',  # Translation: lidar is behind and below camera
        #         '-0.5', '0.5', '-0.5', '0.5',  # Rotation: FLU to optical frame
        #         'camera_color_optical_frame', 'cpsl_uav_1/livox_frame'
        #     ],
        #     output='screen'
        # )

    ])

