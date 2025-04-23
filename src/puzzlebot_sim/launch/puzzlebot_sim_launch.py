# IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os

from ament_index_python.packages import get_package_share_directory

# IMPORTS REQUIRED FOR Launching Nodes
from launch import LaunchDescription

# IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_name = "puzzlebot.urdf"
    urdf_default_path = os.path.join(
        get_package_share_directory("puzzlebot_sim"), "urdf", urdf_file_name
    )

    rviz_file_name = "test1.rviz"
    rviz_default_path = os.path.join(
        get_package_share_directory("puzzlebot_sim"), "rviz", rviz_file_name
    )

    with open(urdf_default_path, "r") as infp:
        robot_desc = infp.read()

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
        arguments=[urdf_default_path],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_default_path,
        ],
    )

    l_d = LaunchDescription([robot_state_pub_node, joint_state_publisher_node, rviz2])

    return l_d
