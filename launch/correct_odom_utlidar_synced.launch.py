from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / 'config' / 'utlidar_synced.yaml'


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=str(DEFAULT_CONFIG),
        description='Point-LIO parameter file path.',
    )

    laser_mapping_params = [
        LaunchConfiguration('config_file'),
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_imu_as_input': False,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,
            'space_down_sample': True,
            'filter_size_surf': 0.1,
            'filter_size_map': 0.1,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,
            'odom_only': True,
            'odom_header_frame_id': 'odom',
            'odom_child_frame_id': 'base_link',
        },
    ]

    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        laser_mapping_node,
    ])
