import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('rccar_fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )

    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='rccar_mid360_mapping.yaml',
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

    odom_frame_remap_node = Node(
        package='odometry_frame_remap',
        executable='odometry_frame_remap',
        name='rccar_odometry_frame_remap_node',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
                'new_frame_id' : 'odom',
                'new_child_frame_id' : 'base_link',
                'publish_tf' : False,
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
                'common.imu_topic' : '/lidar/imu',
            }
        ],
        remappings=[
            ('/Odometry','/odometry/lidar_raw'),
        ],
        output='both'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(odom_frame_remap_node)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
