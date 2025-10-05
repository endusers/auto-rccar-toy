#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_localization_dir = get_package_share_directory('rccar_robot_localization')
    parameters_file_dir = os.path.join(robot_localization_dir, 'config')
    parameters_file_path = os.path.join(parameters_file_dir, 'rccar_ekf_local.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_local_node',
            output='both',
            parameters=[
                parameters_file_path,
                {"use_sim_time": use_sim_time},
            ],
            remappings=[('/odometry/filtered','/odometry/local')],
        ),

        Node(
            package='topic_tools',
            executable='transform',
            name='relay_odometry_local_sigma',
            parameters=[
                {
                    'use_sim_time' : use_sim_time,
                }
            ],
            arguments=[
                '/odometry/local',
                '/odometry/local_sigma_horizontal',
                'std_msgs/Float32',
                'std_msgs.msg.Float32(data=numpy.sqrt(m.pose.covariance[0]+m.pose.covariance[7]))',
                '--import',
                'std_msgs',
                'numpy',
            ],
            output='both',
        ),
    ])
