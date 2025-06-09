# your_ws/src/ros2_coppeliasim_ws/launch/snake.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 1) Nodo oscilador
            Node(
                package="ros2_coppeliasim_ws",
                executable="snake_oscillator",
                name="snake_oscillator",
                output="screen",
            ),
            # 2) teleop_twist_keyboard como Node con emulate_tty
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
