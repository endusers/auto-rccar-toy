#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    included_fast_lio_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_fast_lio' ),
                'launch',
                'rccar-mapping.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
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
        }.items()
    )

    included_slam_online_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_slam_toolbox' ),
                'launch',
                'rccar-online-async.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    delayed_ekf_local_launch = TimerAction(
        period = 10.0,
        actions=[included_ekf_local_launch]
    )

    delayed_slam_launch = TimerAction(
        period = 15.0,
        actions=[included_slam_online_async_launch]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        included_fast_lio_mapping_launch,
        delayed_ekf_local_launch,
        delayed_slam_launch,
        # included_ekf_local_launch,
        # included_slam_online_async_launch
    ])
