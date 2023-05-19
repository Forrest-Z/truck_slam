'''
Descripttion: 
Author: Gang Wang
Date: 2023-01-31 17:28:35
'''
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnss_ins_msgs',
            executable='gnss_ins_msg_decode_node',
            name='gnss_ins_msg_decode_node',
            output="screen"
        )
    ])
