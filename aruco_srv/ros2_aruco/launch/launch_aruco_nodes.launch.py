import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_srv = Node(
        package='ros2_aruco',
        executable='aruco_pose_server'
    )

    drop_pose = Node(
        package='ros2_aruco',
        executable='get_drop_pose'
    )

    find_aruco = Node(
        package='ros2_aruco',
        executable='find_aruco'
    )

    return LaunchDescription([
        aruco_srv,
        drop_pose,
        find_aruco
    ])
