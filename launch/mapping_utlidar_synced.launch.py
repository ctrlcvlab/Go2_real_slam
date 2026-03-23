from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / 'config' / 'utlidar_synced.yaml'


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Flag to launch RViz.',
    )
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
        },
    ]

    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg',
            'loam_livox.rviz',
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice',
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        config_file_arg,
        laser_mapping_node,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
