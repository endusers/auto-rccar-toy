#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    packages_name = "rccar_description"
    models_path = "models"
    model_path = "rccar"
    xacro_file_name = "rccar.urdf.xacro"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), models_path, model_path, xacro_file_name]
            ),
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
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
