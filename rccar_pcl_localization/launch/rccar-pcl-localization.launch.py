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
    use_sim_time = LaunchConfiguration('use_sim_time')

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
        parameters=[localization_param_dir, {'map_path' : map_file}, {"use_sim_time": use_sim_time},],
        remappings=[
            ('/velodyne_points','/lidar/points_raw_PointCloud2'),
            # ('/velodyne_points','/cloud_registered'),
            # ('/velodyne_points','/lidar/velodyne_points'),
            # ('/imu', '/camera/imu'),
            ('/imu', '/bno055/imu'),
            ('/map', '/map/pcl_localization'),
            ('/pcl_pose', '/pose/scan_matching')
        ],
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

    relay_odometry_sm3d_node = Node(
        package='topic_tools',
        executable='relay_field',
        name='relay_odometry_sm3d_pose',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
            }
        ],
        arguments=[
            '/pose/scan_matching',
            '/odometry/sm3d_raw',
            'nav_msgs/Odometry',
            '{header: m.header, pose: m.pose}',
        ],
        output='both',
    )

    remap_odometry_sm3d_node = Node(
        package='odometry_frame_remap',
        executable='odometry_frame_remap',
        name='odometry_sm3d_frame_remap_node',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
                'new_frame_id' : 'odom',
                'new_child_frame_id' : 'base_link',
                'publish_tf' : False,
                'enable_transform' : False,
                'enable_override_covariance' : True,
                'override_covariance_xyz' : [0.0225, 0.0225, 0.0225],
                'override_covariance_rpy' : [0.000625, 0.000625, 0.000625],
                'override_covariance_vxvyvz' : [0.0, 0.0, 0.0],
                'override_covariance_wxwywz' : [0.0, 0.0, 0.0],
            }
        ],
        remappings=[
            ('/odom/in','/odometry/sm3d_raw'),
            ('/odom/out','/odometry/sm3d'),
        ],
        output='both',
    )

    ld = launch.LaunchDescription()

    ld.add_action(declare_map_pcd_cmd)
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    ld.add_action(pcl_localization)
    ld.add_action(to_inactive)

    ld.add_action(relay_odometry_sm3d_node)
    ld.add_action(remap_odometry_sm3d_node)

    return ld