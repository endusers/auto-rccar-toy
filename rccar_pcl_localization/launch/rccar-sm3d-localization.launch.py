import os
import yaml
import math

from ament_index_python.packages import get_package_share_directory

import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

import lifecycle_msgs.msg


def launch_setup(context):

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gnss = LaunchConfiguration('use_gnss')
    map_file = LaunchConfiguration('map')
    initial_pose_xyz = yaml.safe_load( LaunchConfiguration('initial_pose_xyz').perform(context) )
    initial_pose_rpy = yaml.safe_load( LaunchConfiguration('initial_pose_rpy').perform(context) )
    initial_pose_orientation = [
        0.0,
        0.0,
        math.sin( initial_pose_rpy[2] / 2.0 ),
        math.cos( initial_pose_rpy[2] / 2.0 ),
    ]

    pcl_localization = launch_ros.actions.LifecycleNode(
        name='pcl_localization',
        namespace='',
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        parameters=[
            os.path.join(get_package_share_directory("rccar_pcl_localization"), 'config', 'rccar_pcl_localization.yaml'),
            { 'use_sim_time': use_sim_time },
            {
                'map_path' : map_file,
                'initial_pose_x': initial_pose_xyz[0],
                'initial_pose_y': initial_pose_xyz[1],
                'initial_pose_z': initial_pose_xyz[2],
                'initial_pose_qx': initial_pose_orientation[0],
                'initial_pose_qy': initial_pose_orientation[1],
                'initial_pose_qz': initial_pose_orientation[2],
                'initial_pose_qw': initial_pose_orientation[3],
            },
        ],
        remappings=[
            ('/velodyne_points','/lidar/points_raw_PointCloud2'),
            # ('/velodyne_points','/cloud_registered'),
            # ('/velodyne_points','/lidar/velodyne_points'),
            # ('/imu', '/camera/imu'),
            ('/imu', '/bno055/imu'),
            # ('/map', '/map/pcl_localization'),
            ('/initial_map', '/map/pcl_localization'),
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
            '--wait-for-start',
            '/pose/scan_matching',
            '/odometry/sm3d_raw',
            'nav_msgs/Odometry',
            '{header: m.header, pose: m.pose}',
        ],
        output='both',
    )

    override_covariance_param = {
        'enable_override_covariance' : True,
        'override_covariance_xyz' : [0.0225, 0.0225, 0.0225],
        'override_covariance_rpy' : [0.000625, 0.000625, 0.000625],
        'override_covariance_vxvyvz' : [0.0, 0.0, 0.0],
        'override_covariance_wxwywz' : [0.0, 0.0, 0.0],
    }

    remap_odometry_sm3d_w_gnss_node = Node(
        package='odometry_frame_remap',
        executable='odometry_frame_remap',
        name='odometry_sm3d_frame_remap_node',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
                'new_frame_id' : 'map',
                'new_child_frame_id' : '',
                'publish_tf' : False,
                'enable_transform' : False,
            },
            override_covariance_param,
        ],
        remappings=[
            ('/odom/in','/odometry/sm3d_raw'),
            ('/odom/out','/odometry/sm3d'),
        ],
        output='both',
        condition = IfCondition( use_gnss )
    )

    remap_odometry_sm3d_wo_gnss_node = Node(
        package='odometry_frame_remap',
        executable='odometry_frame_remap',
        name='odometry_sm3d_frame_remap_node',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
                'new_frame_id' : 'map',
                'new_child_frame_id' : '',
                'publish_tf' : False,
                'enable_transform' : False,
            },
            override_covariance_param,
        ],
        remappings=[
            ('/odom/in','/odometry/sm3d_raw'),
            ('/odom/out','/odometry/sm3d'),
        ],
        output='both',
        condition = UnlessCondition( use_gnss )
    )

    launch_actions = []

    launch_actions.append(from_unconfigured_to_inactive)
    launch_actions.append(from_inactive_to_active)
    launch_actions.append(pcl_localization)
    launch_actions.append(to_inactive)
    launch_actions.append(relay_odometry_sm3d_node)
    launch_actions.append(remap_odometry_sm3d_w_gnss_node)
    launch_actions.append(remap_odometry_sm3d_wo_gnss_node)

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

    declare_map_pcd_cmd = DeclareLaunchArgument(
        'map',
        default_value='map.pcd',
        description='Path for map pcd file to load')

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
    ld.add_action(declare_map_pcd_cmd)
    ld.add_action(declare_initial_pose_xyz_cmd)
    ld.add_action(declare_initial_pose_rpy_cmd)
    ld.add_action(deferred_launch_action)

    return ld
