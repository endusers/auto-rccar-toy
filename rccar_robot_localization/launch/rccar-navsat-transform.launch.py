#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    wait_for_datum = LaunchConfiguration('wait_for_datum')
    datum = LaunchConfiguration('datum')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'wait_for_datum',
            default_value='false',
            description='Wait for datum if true'
        ),

        DeclareLaunchArgument(
            'datum',
            default_value='[55.944904, -3.186693, 0.0]',
            description='Specify the datum (origin)'
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            remappings=[('/imu','/bno055/imu'), ('/gps/fix','/gnss/fix'), ('/odometry/filtered','/odometry/initial'), ('/odometry/gps','/odometry/gps_raw')],
            # remappings=[('/imu','/camera/imu'), ('/gps/fix','/gnss/fix'), ('/odometry/filtered','/odometry/initial'), ('/odometry/gps','/odometry/gps_raw')],
            parameters=[
                os.path.join(get_package_share_directory("rccar_robot_localization"), 'config', 'rccar_navsat_transform.yaml'),
                {"use_sim_time": use_sim_time},
                {"wait_for_datum": wait_for_datum},
                {"datum": datum},
            ],
        ),
    ])
