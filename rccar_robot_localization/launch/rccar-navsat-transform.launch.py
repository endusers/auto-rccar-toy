#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            #remappings=[('/imu','/bno055/imu'), ('/gps/fix','/gnss/fix'), ('/odometry/filtered','/odometry/initial')],
            remappings=[('/imu','/camera/imu'), ('/gps/fix','/gnss/fix'), ('/odometry/filtered','/odometry/initial')],
            parameters=[
                os.path.join(get_package_share_directory("rccar_robot_localization"), 'config', 'rccar_navsat_transform.yaml'),
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
