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
    parameters_file_path = os.path.join(parameters_file_dir, 'rccar_ekf_global_gnss.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='odometry_frame_remap',
            executable='odometry_frame_remap',
            name='odometry_gps_frame_remap_node',
            parameters=[
                {
                    'use_sim_time' : use_sim_time,
                    'new_frame_id' : 'map',
                    'new_child_frame_id' : '',
                    'publish_tf' : False,
                    'enable_transform' : False,
                }
            ],
            remappings=[
                ('/odom/in','/odometry/gps_raw'),
                ('/odom/out','/odometry/gps'),
            ],
            output='both',
        ),

        Node(
            package='odometry_frame_remap',
            executable='odometry_frame_remap',
            name='odometry_local_frame_remap_node',
            parameters=[
                {
                    'use_sim_time' : use_sim_time,
                    'new_frame_id' : 'map',
                    'new_child_frame_id' : '',
                    'publish_tf' : False,
                    'enable_transform' : True,
                }
            ],
            remappings=[
                ('/odom/in','/odometry/local'),
                ('/odom/out','/odometry/local_in_map'),
            ],
            output='both',
        ),

        Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_global_gnss_node',
            output='screen',
            parameters=[
                parameters_file_path,
                {"use_sim_time": use_sim_time},
            ],
            remappings=[('/odometry/filtered','/odometry/global_raw'), ('/set_pose', '/set_pose')],
        ),

        Node(
            package='odometry_filter',
            executable='odometry_filter',
            name='odometry_global_filter_node',
            parameters=[
                {
                    'use_sim_time' : use_sim_time,
                    'map_frame' : 'map',
                    'odom_frame' : 'odom',
                    'base_link_frame' : 'base_link',
                    'twist_publish_rate' : 1.0,
                    'initial_pose' : [0.0, 0.0, 0.0],
                    'update_yaw_only' : True,
                    'publish_tf' : True,
                    'enable_odom_update_covariance_check' : True,
                    'odom_update_covariance_sigma_threshold' : 1.0,
                }
            ],
            remappings=[
                ('/odom/in','/odometry/global_raw'),
                ('/odom/out','/odometry/global'),
                ('/odom/out_twist_resampler','/odometry/global_twist_resampler'),
                ('/odom/odom_update_check','/odometry/gps'),
                ('/initialpose','/initialpose'),
            ],
            output='both',
        ),

        Node(
            package='topic_tools',
            executable='transform',
            name='relay_odometry_global_sigma',
            parameters=[
                {
                    'use_sim_time' : use_sim_time,
                }
            ],
            arguments=[
                '/odometry/global',
                '/odometry/global_sigma_horizontal',
                'std_msgs/Float32',
                'std_msgs.msg.Float32(data=numpy.sqrt(m.pose.covariance[0]+m.pose.covariance[7]))',
                '--import',
                'std_msgs',
                'numpy',
            ],
            output='both',
        ),
    ])

# Memo
# ros2 param set /odometry_global_filter_node initial_pose "[0.0, 0.0, 0.0]"
