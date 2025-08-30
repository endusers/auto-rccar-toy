#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('rccar_description')
    # model_file = os.path.join(pkg_robot_description, 'models', 'rccar-unimog', 'rccar-unimog.urdf.xacro')
    model_file = os.path.join(pkg_robot_description, 'models', 'rccar-bronco', 'rccar-bronco.urdf.xacro')

    gnss_only = LaunchConfiguration('gnss_only')
    use_gnss = LaunchConfiguration('use_gnss')
    use_camera = LaunchConfiguration('use_camera')
    use_lidar = LaunchConfiguration('use_lidar')
    use_multi_imu = LaunchConfiguration('use_multi_imu')
    model = LaunchConfiguration('model')

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

    declare_use_multi_imu_cmd = DeclareLaunchArgument(
        'use_multi_imu',
        default_value='true',
        description='Use multi imu if true'
    )

    declare_model_file_cmd = DeclareLaunchArgument(
        'model',
        default_value=model_file,
        description='Path for model file to load')

    included_robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_description' ),
                'launch',
                'rccar-robot-state-publisher.launch.py'
            )
        ),
        launch_arguments={
            'model': model,
        }.items()
    )

    included_micro_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_bringup' ),
                'launch',
                'rccar-micro-ros-agent.launch.py'
            )
        ),
    )

    included_bno055_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_bringup' ),
                'launch',
                'rccar-bno055.launch.py'
            )
        ),
    )

    included_gnss_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_bringup' ),
                'launch',
                'rccar-gnss.launch.xml'
            )
        ),
        launch_arguments={
            'gnss_only': gnss_only,
        }.items(),
        condition = IfCondition( use_gnss )
    )

    included_camera_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_bringup' ),
                'launch',
                'rccar-camera.launch.xml'
            )
        ),
        condition = IfCondition( use_camera )
    )

    included_lidar_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_bringup' ),
                'launch',
                'rccar-lidar.launch.xml'
            )
        ),
        condition = IfCondition( use_lidar )
    )

    included_multi_imu_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_bringup' ),
                'launch',
                'rccar-multi-imu.launch.xml'
            )
        ),
        condition = IfCondition( use_multi_imu )
    )

    delayed_gnss_launch = TimerAction(
        period = 5.0,
        actions=[included_gnss_launch]
    )

    delayed_camera_launch = TimerAction(
        period = 10.0,
        actions=[included_camera_launch]
    )

    delayed_lidar_launch = TimerAction(
        period = 20.0,
        actions=[included_lidar_launch]
    )

    delayed_multi_imu_launch = TimerAction(
        period = 30.0,
        actions=[included_multi_imu_launch]
    )

    return LaunchDescription([
        declare_gnss_only_cmd,
        declare_use_gnss_cmd,
        declare_use_camera_cmd,
        declare_use_lidar_cmd,
        declare_use_multi_imu_cmd,
        declare_model_file_cmd,
        included_robot_state_launch,
        included_micro_ros_launch,
        # included_bno055_launch,
        delayed_gnss_launch,
        delayed_camera_launch,
        delayed_lidar_launch,
        delayed_multi_imu_launch,
        # included_gnss_launch,
        # included_camera_launch,
        # included_lidar_launch,
        # included_multi_imu_launch,
    ])
