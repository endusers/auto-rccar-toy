#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # map_yaml_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'empty', 'empty_100m.yaml')
    # map_yaml_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'empty', 'empty_1000m.yaml')
    # map_yaml_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'smalltown', 'smalltown_world.yaml')
    # map_yaml_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'smalltown', 'map.yaml')
    map_yaml_file = os.path.join(get_package_share_directory('rccar_navigation2'), 'map', 'cafe', 'map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gnss_only = LaunchConfiguration('gnss_only')
    map_file = LaunchConfiguration('map')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_gnss_only_cmd = DeclareLaunchArgument(
        'gnss_only',
        default_value='false',
        description='Use only gnss if true'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Path for map yaml file to load')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='both',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_file}],
            remappings=remappings
    )

    lifecycle_manager_mapserver = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapserver',
        output='both',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    navigation2_gnss_only_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rccar_navigation2'), 'launch', 'rccar-nav2-gnss-only.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_lifecycle_mgr': 'false',
            'map_subscribe_transient_local': 'true',
        }.items(),
        condition = IfCondition( gnss_only )
    )

    navigation2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rccar_navigation2'), 'launch', 'rccar-nav2.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_lifecycle_mgr': 'false',
            'map_subscribe_transient_local': 'true',
        }.items(),
        condition = UnlessCondition( gnss_only )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gnss_only_cmd)
    ld.add_action(declare_map_yaml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_mapserver)
    ld.add_action(navigation2_gnss_only_nodes)
    ld.add_action(navigation2_nodes)

    return ld
