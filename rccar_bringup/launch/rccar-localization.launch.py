#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # map_pcd_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'smalltown', 'map.pcd')
    map_pcd_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'cafe', 'map.pcd')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gnss_only = LaunchConfiguration('gnss_only')
    use_gnss = LaunchConfiguration('use_gnss')
    use_camera = LaunchConfiguration('use_camera')
    use_lidar = LaunchConfiguration('use_lidar')
    map_file = LaunchConfiguration('map')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_gnss_only_cmd = DeclareLaunchArgument(
        'gnss_only',
        default_value='false',
        description='Use only gnss if true'
    )

    declare_use_gnss_cmd = DeclareLaunchArgument(
        'use_gnss',
        default_value='true',
        description='Use gnss if true'
    )

    declare_use_camera_cmd = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Use camera if true'
    )

    declare_use_lidar_cmd = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Use lidar if true'
    )

    declare_map_pcd_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_pcd_file,
        description='Path for map pcd file to load')

    included_navsat_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_robot_localization' ),
                'launch',
                'rccar-navsat-transform.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        # condition = IfCondition( use_gnss )
        condition = IfCondition( gnss_only )
    )

    included_ekf_gnss_only_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_robot_localization' ),
                'launch',
                'rccar-ekf-gnss-only.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition = IfCondition( gnss_only )
    )

    included_fast_lio_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_fast_lio' ),
                'launch',
                'rccar-odometry.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition = IfCondition( use_lidar )
    )

    included_ekf_local_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_robot_localization' ),
                'launch',
                'rccar-ekf-local.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition = IfCondition( use_lidar )
    )

    included_pcl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_pcl_localization' ),
                'launch',
                'rccar-pcl-localization.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
        }.items(),
        condition = IfCondition( use_lidar )
    )

    included_ekf_global_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_robot_localization' ),
                'launch',
                'rccar-ekf-global.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition = IfCondition( use_lidar )
    )

    delayed_ekf_local_launch = TimerAction(
        period = 10.0,
        actions=[included_ekf_local_launch]
    )

    delayed_pcl_launch = TimerAction(
        period = 15.0,
        actions=[included_pcl_localization_launch]
    )

    delayed_ekf_global_launch = TimerAction(
        period = 25.0,
        actions=[included_ekf_global_launch]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_gnss_only_cmd,
        declare_use_gnss_cmd,
        declare_use_camera_cmd,
        declare_use_lidar_cmd,
        declare_map_pcd_cmd,
        included_navsat_transform_launch,
        included_ekf_gnss_only_launch,
        included_fast_lio_odometry_launch,
        delayed_ekf_local_launch,
        delayed_pcl_launch,
        delayed_ekf_global_launch,
        # included_ekf_local_launch,
        # included_pcl_localization_launch,
        # included_ekf_global_launch,
    ])
