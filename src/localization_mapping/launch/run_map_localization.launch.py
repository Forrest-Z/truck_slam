'''
Descripttion: 
Author: Gang Wang
Date: 2023-02-08 11:57:48
'''
import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('localization_mapping'), 'config', 'map_localization.yaml')
    rviz_config = os.path.join(get_package_share_directory('localization_mapping')+'/rviz/run_map_localization.rviz')

    return LaunchDescription([
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config]),
        # 固定odom系和map系， 发布tf
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            name = 'odom2map_tf',
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output="screen"
        ),
        Node(
            package = 'localization_mapping',
            executable = 'data_pretreat_node',
            name = 'data_pretreat_node',
            output="screen"
        ),
        Node(
            package = 'localization_mapping',
            executable = 'map_matching_node',
            name = 'map_matching_node',
            parameters = [config]
        ),
        Node(
            package = 'localization_mapping',
            executable = 'imu_preintegration_node',
            name = 'imu_preintegration_node',
        )
    ])