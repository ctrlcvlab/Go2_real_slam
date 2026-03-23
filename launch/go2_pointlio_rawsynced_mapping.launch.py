from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch Point-LIO RViz view.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    imu_time_inte_arg = DeclareLaunchArgument(
        'imu_time_inte',
        default_value='0.01',
        description='Point-LIO IMU integration time step.',
    )
    gravity_align_arg = DeclareLaunchArgument(
        'gravity_align',
        default_value='false',
        description='Align world Z axis with gravity.',
    )
    lid_topic_arg = DeclareLaunchArgument(
        'lid_topic',
        default_value='/utlidar/cloud_synced',
        description='Raw synced LiDAR cloud topic for Point-LIO.',
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/utlidar/imu_synced',
        description='Raw synced IMU topic for Point-LIO.',
    )

    # LiDAR pose in IMU frame, derived from the static transforms published in
    # time_sync_bridge.py:
    #   base_link -> utlidar_imu   = (-0.02557, 0.0,  0.04232), R=I
    #   base_link -> utlidar_lidar = ( 0.28945, 0.0, -0.046825), pitch=2.8782
    # Therefore lidar pose in IMU frame is:
    #   T = (0.31502, 0.0, -0.089145)
    #   R = RotY(2.8782025850555556)
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config',
            'utlidar_synced.yaml',
        ]),
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'common.lid_topic': LaunchConfiguration('lid_topic'),
            'common.imu_topic': LaunchConfiguration('imu_topic'),
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
            'mapping.imu_time_inte': LaunchConfiguration('imu_time_inte'),
            'mapping.gravity_align': LaunchConfiguration('gravity_align'),
            'mapping.extrinsic_T': [0.31502, 0.0, -0.089145],
            'mapping.extrinsic_R': [
                -0.9655129059697726, 0.0, 0.26035519661763046,
                0.0, 1.0, 0.0,
                -0.26035519661763046, 0.0, -0.9655129059697726,
            ],
            'publish.path_en': True,
            'publish.scan_publish_en': True,
            'publish.scan_bodyframe_pub_en': False,
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
        imu_time_inte_arg,
        gravity_align_arg,
        lid_topic_arg,
        imu_topic_arg,
        laser_mapping_node,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
