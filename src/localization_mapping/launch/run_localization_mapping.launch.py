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
    config = os.path.join(get_package_share_directory('localization_mapping'), 'config', 'ekf_gps.yaml')

    rviz_config = os.path.join(get_package_share_directory('localization_mapping')+'/rviz/run.rviz')
    urdf = os.path.join(get_package_share_directory('localization_mapping'), 'config', 'robot.urdf.xacro')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config]),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': robot_desc}],
        #     arguments=[urdf]
        # ),
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
            executable = 'front_end_node',
            name = 'front_end_node',
        ),
        Node(
            package = 'localization_mapping',
            executable = 'back_end_node',
            name = 'back_end_node',
        ),
        Node(
            package = 'localization_mapping',
            executable = 'loop_closure_node',
            name = 'loop_closure_node',
        ),
        Node(
            package = 'localization_mapping',
            executable = 'imu_preintegration_node',
            name = 'imu_preintegration_node',
        ),
        # Node(
        #     package = 'robot_localization',
        #     executable = 'navsat_transform_node',
        #     name = 'navsat',
        #     output='screen',
        #     parameters=[config],
        #     remappings=[
        #        ('/imu', 'imu_correct'),
        #        ('gps/fix', 'gps/fix'),
        #        ('/odometry/filtered', 'odometry/navsat') 
        #     ]
        # ),
        # Node(
        #     package = 'robot_localization',
        #     executable = 'ekf_node',
        #     name = 'ekf_gps',
        #     output='screen',
        #     parameters=[config],
        #     remappings=[
        #        ('/odometry/filtered', 'odometry/navsat')
        #     ]
        # )
    ])