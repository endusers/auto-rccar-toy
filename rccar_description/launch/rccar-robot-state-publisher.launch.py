#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    xacro_file = [ os.path.join( get_package_share_directory('rccar_description'), 'models', "rccar-unimog", "rccar-unimog.urdf.xacro" ) ]
    declare_model_file_cmd = DeclareLaunchArgument(
        'model',
        default_value=xacro_file,
        description='Path for model file to load')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration('model'),
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description_content,
        }],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="both",
        condition=UnlessCondition(use_sim_time),
    )

    return LaunchDescription([
        declare_model_file_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
