import os.path
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def launch_setup(context):

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    initial_pose_xyz = yaml.safe_load( LaunchConfiguration('initial_pose_xyz').perform(context) )
    initial_pose_rpy = yaml.safe_load( LaunchConfiguration('initial_pose_rpy').perform(context) )

    initial_pose_xyz_str = [str(float(v)) for v in initial_pose_xyz]
    initial_pose_rpy_str = [str(float(v)) for v in initial_pose_rpy]

    tf_static_fastlio_init_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_init',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
            }
        ],
        arguments=[
            initial_pose_xyz_str[0], initial_pose_xyz_str[1], initial_pose_xyz_str[2],
            initial_pose_rpy_str[0], initial_pose_rpy_str[1], initial_pose_rpy_str[2],
            'base_link', 'camera_init'
        ],
        output='both',
    )

    odom_frame_remap_node = Node(
        package='odometry_frame_remap',
        executable='odometry_frame_remap',
        name='odometry_lidar_frame_remap_node',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
                'new_frame_id' : 'odom',
                'new_child_frame_id' : 'base_link',
                'publish_tf' : False,
                'enable_transform' : True,
                'transform_target_frame' : 'base_link',
                'transform_source_frame' : 'camera_init',
                'scale_covariance_xyz' : [1e5, 1e5, 1e5],
                'scale_covariance_rpy' : [1e5, 1e5, 1e5],
                'scale_covariance_vxvyvz' : [1e4, 1e4, 1e4],
                'scale_covariance_wxwywz' : [1e4, 1e4, 1e4],
            }
        ],
        remappings=[
            ('/odom/in','/odometry/lidar_raw'),
            ('/odom/out','/odometry/lidar'),
        ],
        output='both',
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {
                'use_sim_time' : use_sim_time,
                'common.lid_topic' : '/lidar/points_raw_PointCloud2',
                'common.imu_topic' : '/lidar/imu' ,
            }
        ],
        remappings=[
            ('/Odometry','/odometry/lidar_raw'),
        ],
        output='both',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    relay_odom_sigma_node = Node(
        package='topic_tools',
        executable='transform',
        name='relay_odometry_lidar_sigma',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
            }
        ],
        arguments=[
            '--wait-for-start',
            '/odometry/lidar',
            '/odometry/lidar_sigma_horizontal',
            'std_msgs/Float32',
            'std_msgs.msg.Float32(data=numpy.sqrt(m.pose.covariance[0]+m.pose.covariance[7]))',
            '--import',
            'std_msgs',
            'numpy',
        ],
        output='both',
    )

    launch_actions = []

    launch_actions.append(tf_static_fastlio_init_node)
    launch_actions.append(odom_frame_remap_node)
    launch_actions.append(fast_lio_node)
    launch_actions.append(rviz_node)
    launch_actions.append(relay_odom_sigma_node)

    return launch_actions

def generate_launch_description():

    package_path = get_package_share_directory('rccar_fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='rccar_mid360_odometry.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
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
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_initial_pose_xyz_cmd)
    ld.add_action(declare_initial_pose_rpy_cmd)
    ld.add_action(deferred_launch_action)

    return ld
