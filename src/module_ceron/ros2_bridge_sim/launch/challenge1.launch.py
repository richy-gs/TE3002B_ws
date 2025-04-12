from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_bridge_sim",
                executable="image_subscriber",
                name="image_subscriber_node",
                output="screen",
            ),
            # Node(
            #     package="ros2_bridge_sim",
            #     executable="teleop_controller",
            #     name="teleop_controller_node",
            #     output="screen",
            # ),
        ]
    )
