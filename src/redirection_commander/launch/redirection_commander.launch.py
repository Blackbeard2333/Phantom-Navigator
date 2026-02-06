from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='redirection_commander',
            executable='redirection_commander',
            name='redirection_commander',
            output='screen',
            parameters=[{
                'lambda_a': 0.5,
                'v_ref_max': 0.2,
                'v_spoof_max': 0.2,  # [m/s]
                'deadband_radius': 0.3,
                'control_rate_hz': 10.0,
                'frame_id': 'vicon/world',
                'attack_start_delay_s': 0.0,
                'base_Kp_pos': 0.6, 'Ki_pos': 0.005, 'Kd_pos': 0.1, 'pos_gain_decay': 0.1,
                'Kp_vel': 0.05, 'Ki_vel': 0.002, 'Kd_vel': 0.001,
            }],
        ),
        # Node(
        #     package='redirection_commander',                 # change if you put it in another package
        #     executable='redirection_target_playlist_sender', # must match your entry point / script name
        #     name='redirection_target_playlist_sender',
        #     output='screen',
        #     parameters=[{
        #         'frame_id': 'vicon/world',
        #         'radius': 2.0,                 # circle radius [m]
        #         'n_points': 5,                # number of random targets
        #         'dwell_time_s': 20.0,          # seconds per target
        #         'attack_start_delay_s': 0.0,  # wait this long after node start before publishing targets
        #         'random_seed': 42,             # <0 => random each run (set to e.g. 42 for determinism)
        #         'loop': False,                 # set True to cycle forever
        #         'reannounce_rate_hz': 0.0,     # >0 to periodically republish current target
        #         'publish_before_attack': False # set True to publish (0,0) before start delay
        #     }],
        # ) 
    ])