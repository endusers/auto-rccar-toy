#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='File name for world file to load')

    world_file = [ os.path.join( get_package_share_directory('rccar_gazebo'), 'worlds', '' ), LaunchConfiguration('world') ]
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('rccar_description')
    robot_name_in_model = 'rccar'
    robot_model_file_path = os.path.join(pkg_robot_description, 'models', 'rccar', 'rccar.sdf')

    gazebo_resource_path = os.environ["GAZEBO_RESOURCE_PATH"] if "GAZEBO_RESOURCE_PATH" in os.environ else "" \
                            + ':' + "/usr/share/gazebo-11"
    gazebo_model_path = os.environ["GAZEBO_MODEL_PATH"] if "GAZEBO_MODEL_PATH" in os.environ else "" \
                            + ':' + os.path.join(pkg_robot_description, 'models')

    os.environ["GAZEBO_RESOURCE_PATH"] = gazebo_resource_path
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_model_path

    gzserver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'debug': 'true',
            'verbose': 'true',
        }.items(),
    )

    gzclient_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_description, 'launch', 'rccar-robot-state-publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='both',
        arguments=[
            '-topic', 'robot_description',
            #'-file', robot_model_file_path,
            '-entity', robot_name_in_model,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
    )

    return LaunchDescription([
        declare_world_file_cmd,
        gzserver_node,
        gzclient_node,
        robot_state_publisher_node,
        spawn_entity_node,
    ])
