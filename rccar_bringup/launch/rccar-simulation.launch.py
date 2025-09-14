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
    # world_file = 'empty.world'
    # world_file = 'smalltown.world'
    world_file = 'cafe.world'
    # model_file = os.path.join(pkg_robot_description, 'models', 'rccar-unimog', 'rccar-unimog.urdf.xacro')
    model_file = os.path.join(pkg_robot_description, 'models', 'rccar-bronco', 'rccar-bronco.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gnss_only = LaunchConfiguration('gnss_only')
    use_gnss = LaunchConfiguration('use_gnss')
    use_camera = LaunchConfiguration('use_camera')
    use_lidar = LaunchConfiguration('use_lidar')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
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

    declare_world_file_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='File name for world file to load')

    declare_model_file_cmd = DeclareLaunchArgument(
        'model',
        default_value=model_file,
        description='Path for model file to load')

    included_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_gazebo' ),
                'launch',
                'rccar-gazebo.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'model': model,
        }.items()
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
            'use_sim_time': use_sim_time,
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
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
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
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition = IfCondition( use_lidar )
    )

    included_imu_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory( 'rccar_bringup' ),
                'launch',
                'rccar-imu.launch.xml'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    delayed_launch = TimerAction(
        period = 20.0,
        actions=[included_gnss_launch, included_camera_launch, included_lidar_launch, included_imu_launch]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_gnss_only_cmd,
        declare_use_gnss_cmd,
        declare_use_camera_cmd,
        declare_use_lidar_cmd,
        declare_world_file_cmd,
        declare_model_file_cmd,
        included_gazebo_launch,
        delayed_launch,
        # included_gnss_launch,
        # included_camera_launch,
        # included_lidar_launch,
        # included_imu_launch,
    ])
