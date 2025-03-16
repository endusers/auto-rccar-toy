import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_file = LaunchConfiguration('map')

    declare_map_pcd_cmd = DeclareLaunchArgument(
        'map',
        default_value='map.pcd',
        description='Path for map pcd file to load')

    localization_param_dir = LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('rccar_pcl_localization'),
            'config',
            'rccar_pcl_localization.yaml'))

    pcl_localization = launch_ros.actions.LifecycleNode(
        name='pcl_localization',
        namespace='',
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        parameters=[localization_param_dir, {'map_path' : map_file}],
        remappings=[('/velodyne_points','/lidar/points_raw_PointCloud2'), ('/imu', '/camera/imu')],
        # remappings=[('/velodyne_points','/cloud_registered'), ('/imu', '/camera/imu')],
        # remappings=[('/velodyne_points','/velodyne_points'), ('/imu', '/camera/imu')],
        output='both')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    ld = launch.LaunchDescription()

    ld.add_action(declare_map_pcd_cmd)
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    ld.add_action(pcl_localization)
    ld.add_action(to_inactive)

    return ld