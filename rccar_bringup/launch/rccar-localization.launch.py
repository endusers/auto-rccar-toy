#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, OrSubstitution
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # map_pcd_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'smalltown', 'map.pcd')
    map_pcd_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'cafe', 'map.pcd')

    # map_posegraph_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'smalltown', 'map')
    map_posegraph_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'cafe', 'map')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gnss = LaunchConfiguration('use_gnss')
    use_camera = LaunchConfiguration('use_camera')
    use_lidar = LaunchConfiguration('use_lidar')
    use_3d_matching = LaunchConfiguration('use_3d_matching')
    use_2d_matching = LaunchConfiguration('use_2d_matching')
    use_gnss_fix = LaunchConfiguration('use_gnss_fix')
    map_pcd = LaunchConfiguration('map_pcd')
    map_posegraph = LaunchConfiguration('map_posegraph')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
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

    declare_use_3d_matching_cmd = DeclareLaunchArgument(
        'use_3d_matching',
        default_value='false',
        description='Use 3d scan matching if true'
    )

    declare_use_2d_matching_cmd = DeclareLaunchArgument(
        'use_2d_matching',
        default_value='false',
        description='Use 2d scan matching if true'
    )

    declare_use_gnss_fix_cmd = DeclareLaunchArgument(
        'use_gnss_fix',
        default_value='true',
        description='Use gnss fix if true'
    )

    declare_map_pcd_cmd = DeclareLaunchArgument(
        'map_pcd',
        default_value=map_pcd_file,
        description='Path for map pcd file to load')

    declare_map_posegraph_cmd = DeclareLaunchArgument(
        'map_posegraph',
        default_value=map_posegraph_file,
        description='Path for map posegraph file to load')

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
        condition = IfCondition( use_gnss )
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
            'map': map_pcd,
        }.items(),
        condition = IfCondition( use_3d_matching )
    )

    included_slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_slam_toolbox' ),
                'launch',
                'rccar-localization.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_posegraph,
        }.items(),
        condition = IfCondition( use_2d_matching )
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
        condition = IfCondition( OrSubstitution( use_2d_matching, use_3d_matching ) )
    )

    included_ekf_global_gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_robot_localization' ),
                'launch',
                'rccar-ekf-global-gnss.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition = IfCondition( use_gnss_fix )
    )

    delayed_ekf_local_launch = TimerAction(
        period = 10.0,
        actions=[included_ekf_local_launch]
    )

    delayed_pcl_launch = TimerAction(
        period = 15.0,
        actions=[included_pcl_localization_launch, included_slam_toolbox_launch]
    )

    delayed_ekf_global_launch = TimerAction(
        period = 25.0,
        actions=[included_ekf_global_launch]
    )

    delayed_ekf_global_gnss_launch = TimerAction(
        period = 25.0,
        actions=[included_ekf_global_gnss_launch]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_gnss_cmd,
        declare_use_camera_cmd,
        declare_use_lidar_cmd,
        declare_use_3d_matching_cmd,
        declare_use_2d_matching_cmd,
        declare_use_gnss_fix_cmd,
        declare_map_pcd_cmd,
        declare_map_posegraph_cmd,
        included_navsat_transform_launch,
        included_fast_lio_odometry_launch,
        delayed_ekf_local_launch,
        delayed_pcl_launch,
        delayed_ekf_global_launch,
        delayed_ekf_global_gnss_launch,
        # included_ekf_local_launch,
        # included_pcl_localization_launch,
        # included_slam_toolbox_launch,
        # included_ekf_global_launch,
        # included_ekf_global_gnss_launch,
    ])
