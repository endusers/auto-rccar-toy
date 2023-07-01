#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='both',
            #arguments=["udp4", "-p", "8888", "-v6"]
            #arguments=["udp4", "--port", "8888"]
            #arguments=["serial", "--dev", "/dev/ttyUSB0", "--baud", "115200"]
            arguments=["serial", "--dev", "/dev/ttyTHS0", "--baud", "115200"]
        )
    ])

