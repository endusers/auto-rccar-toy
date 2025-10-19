import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gnss = LaunchConfiguration('use_gnss')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_use_gnss_cmd = DeclareLaunchArgument(
        'use_gnss',
        default_value='true',
        description='Use gnss if true'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("rccar_slam_toolbox"),
                                   'config', 'rccar_mapper_params_localization.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_map_posegraph_cmd = DeclareLaunchArgument(
        'map',
        default_value='test_steve',
        description='Path for map posegraph file to load')

    start_localization_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time},
          {'map_file_name' : map_file},
          {'map_start_pose': [0.0, 0.0, 0.0]},
          {'map_start_at_dock' : True}
        ],
        remappings=[('/map','/map/slam_toolbox'), ('/pose','/pose/scan_matching')],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='both')

    relay_odometry_sm2d_node = Node(
        package='topic_tools',
        executable='relay_field',
        name='relay_odometry_sm2d_pose',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
            }
        ],
        arguments=[
            '/pose/scan_matching',
            '/odometry/sm2d_raw',
            'nav_msgs/Odometry',
            '{header: m.header, pose: m.pose}',
        ],
        output='both',
    )

    remap_odometry_sm2d_wo_gnss_node = Node(
        package='odometry_frame_remap',
        executable='odometry_frame_remap',
        name='odometry_sm2d_frame_remap_node',
        parameters=[
            {
                'use_sim_time' : use_sim_time,
                'new_frame_id' : 'map',
                'new_child_frame_id' : 'odom',
                'publish_tf' : False,
                'enable_transform' : False,
            }
        ],
        remappings=[
            ('/odom/in','/odometry/sm2d_raw'),
            ('/odom/out','/odometry/sm2d'),
        ],
        output='both',
        condition = UnlessCondition( use_gnss )
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_localization_slam_toolbox_node)
    ld.add_action(relay_odometry_sm2d_node)
    ld.add_action(remap_odometry_sm2d_wo_gnss_node)

    return ld
