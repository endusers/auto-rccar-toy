#!/usr/bin/env python3

import os
import yaml
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def launch_setup(context):

    robot_localization_dir = get_package_share_directory('rccar_robot_localization')
    config_file_dir = os.path.join(robot_localization_dir, 'config')
    config_file_path_w_gnss = os.path.join(config_file_dir, 'rccar_ekf_global_w_gnss.yaml')
    config_file_path_wo_gnss = os.path.join(config_file_dir, 'rccar_ekf_global_wo_gnss.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gnss = LaunchConfiguration('use_gnss')
    initial_pose_xyz = yaml.safe_load( LaunchConfiguration('initial_pose_xyz').perform(context) )
    initial_pose_rpy = yaml.safe_load( LaunchConfiguration('initial_pose_rpy').perform(context) )
    initial_pose_yaw_deg = math.degrees(initial_pose_rpy[2])

    odom_frame_remap_node = Node(
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
                'transform_target_frame' : 'map',
                'transform_source_frame' : 'odom',
            }
        ],
        remappings=[
            ('/odom/in','/odometry/local'),
            ('/odom/out','/odometry/local_in_map'),
        ],
        output='both',
    )

    global_w_gnss_node = Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_global_w_gnss_node',
        parameters=[
            config_file_path_w_gnss,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[('/odometry/filtered','/odometry/global_raw'), ('/set_pose', '/set_pose')],
        output='both',
        condition = IfCondition( use_gnss )
    )

    global_wo_gnss_node = Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_global_wo_gnss_node',
        parameters=[
            config_file_path_wo_gnss,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[('/odometry/filtered','/odometry/global')],
        # remappings=[('/odometry/filtered','/odometry/global'), ('/set_pose', '/initialpose')],
        output='both',
        condition = UnlessCondition( use_gnss )
    )

    global_filter_node = Node(
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
                'initial_pose' : [0.0, 0.0, initial_pose_yaw_deg],
                # 'initial_pose' : [initial_pose_xyz[0], initial_pose_xyz[1], initial_pose_yaw_deg],
                'update_yaw_only' : False,
                'publish_tf' : True,
                'enable_odom_update_covariance_check' : False,
                'odom_update_covariance_sigma_threshold' : 0.5,
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
        condition = IfCondition( use_gnss )
    )

    relay_global_sigma_node = Node(
        package='topic_tools',
        executable='transform',
        name='relay_odometry_global_sigma',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
            }
        ],
        arguments=[
            '--wait-for-start',
            '/odometry/global',
            '/odometry/global_sigma_horizontal',
            'std_msgs/Float32',
            'std_msgs.msg.Float32(data=numpy.sqrt(m.pose.covariance[0]+m.pose.covariance[7]))',
            '--import',
            'std_msgs',
            'numpy',
        ],
        output='both',
    )

    launch_actions = []

    launch_actions.append(odom_frame_remap_node)
    launch_actions.append(global_w_gnss_node)
    launch_actions.append(global_wo_gnss_node)
    launch_actions.append(global_filter_node)
    launch_actions.append(relay_global_sigma_node)

    return launch_actions

def generate_launch_description():

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

    declare_initial_pose_xyz_cmd = DeclareLaunchArgument(
        'initial_pose_xyz', default_value='["0.0", "0.0", "0.0"]',
        description='Specify the initial pose (x, y, z)'
    )
    declare_initial_pose_rpy_cmd = DeclareLaunchArgument(
        'initial_pose_rpy', default_value='["0.0", "0.0", "0.0"]',
        description='Specify the initial pose (roll, pitch, yaw)'
    )

    deferred_launch_action = OpaqueFunction(function=launch_setup)

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_gnss_cmd)
    ld.add_action(declare_initial_pose_xyz_cmd)
    ld.add_action(declare_initial_pose_rpy_cmd)
    ld.add_action(deferred_launch_action)

    return ld

# Memo
# ros2 param set /odometry_global_filter_node initial_pose "[0.0, 0.0, 0.0]"
