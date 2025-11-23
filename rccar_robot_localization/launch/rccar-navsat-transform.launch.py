#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def launch_setup(context):

    use_sim_time = LaunchConfiguration('use_sim_time')
    gnss_only = LaunchConfiguration('gnss_only')
    wait_for_datum = LaunchConfiguration('wait_for_datum')
    datum = LaunchConfiguration('datum')
    initial_pose_xyz = yaml.safe_load( LaunchConfiguration('initial_pose_xyz').perform(context) )
    initial_pose_rpy = yaml.safe_load( LaunchConfiguration('initial_pose_rpy').perform(context) )

    initial_pose_xyz_str = [str(float(v)) for v in initial_pose_xyz]
    initial_pose_rpy_str = [str(float(v)) for v in initial_pose_rpy]

    tf_static_gnss_only_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
            }
        ],
        arguments=[
            '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link'
        ],
        output='both',
        condition = IfCondition( gnss_only ),
    )

    tf_static_gnss_init_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gnss_init',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
            }
        ],
        arguments=[
            initial_pose_xyz_str[0], initial_pose_xyz_str[1], initial_pose_xyz_str[2],
            initial_pose_rpy_str[0], initial_pose_rpy_str[1], initial_pose_rpy_str[2],
            'base_link', 'gnss_init'
        ],
        output='both',
    )

    odom_frame_remap_node = Node(
        package='odometry_frame_remap',
        executable='odometry_frame_remap',
        name='odometry_gnss_frame_remap_node',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
                'new_frame_id' : 'map',
                'new_child_frame_id' : '',
                'publish_tf' : False,
                'enable_transform' : True,
                'transform_target_frame' : 'base_link',
                'transform_source_frame' : 'gnss_init',
            }
        ],
        remappings=[
            ('/odom/in','/odometry/gps_raw'),
            ('/odom/out','/odometry/gps'),
        ],
        output='both',
    )

    initial_odometry_node = Node(
        package='initial_odometry',
        executable='initial_odometry',
        name='initial_odometry_node',
        remappings=[('/odometry/initial','/odometry/initial'), ('/odometry/gps','/odometry/gps_raw')],
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'frame_id': 'gnss_init',
                'child_frame_id': 'gnss_anchor',
                'initial_pose': [0.0, 0.0, 0.0],
                'publish_tf': True,
                'enable_parent_frame': False,
                'parent_frame': 'base_link',
                'parent_initial_pose': [0.0, 0.0, 0.0],
            },
        ],
        output='both',
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        remappings=[('/imu','/bno055/imu'), ('/gps/fix','/gnss/fix'), ('/odometry/filtered','/odometry/initial'), ('/odometry/gps','/odometry/gps_raw')],
        # remappings=[('/imu','/camera/imu'), ('/gps/fix','/gnss/fix'), ('/odometry/filtered','/odometry/initial'), ('/odometry/gps','/odometry/gps_raw')],
        parameters=[
            os.path.join(get_package_share_directory("rccar_robot_localization"), 'config', 'rccar_navsat_transform.yaml'),
            {"use_sim_time": use_sim_time},
            {"wait_for_datum": wait_for_datum},
            {"datum": datum},
        ],
        output='both',
    )

    launch_actions = []

    launch_actions.append(tf_static_gnss_only_node)
    launch_actions.append(tf_static_gnss_init_node)
    launch_actions.append(odom_frame_remap_node)
    launch_actions.append(initial_odometry_node)
    launch_actions.append(navsat_transform_node)

    return launch_actions

def generate_launch_description():

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

    declare_wait_for_datum_cmd = DeclareLaunchArgument(
        'wait_for_datum',
        default_value='false',
        description='Wait for datum if true'
    )
    declare_datum_cmd = DeclareLaunchArgument(
        'datum',
        default_value='[55.944904, -3.186693, 0.0]',
        description='Specify the datum (origin)'
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
    ld.add_action(declare_gnss_only_cmd)
    ld.add_action(declare_wait_for_datum_cmd)
    ld.add_action(declare_datum_cmd)
    ld.add_action(declare_initial_pose_xyz_cmd)
    ld.add_action(declare_initial_pose_rpy_cmd)
    ld.add_action(deferred_launch_action)

    return ld
