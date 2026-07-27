"""Compatibility launch name that now starts the safety-gated one-arm nodes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("servo_controller"))
    config = share / "config" / "safe_one_arm.yaml"
    return LaunchDescription(
        [
            Node(
                package="servo_controller",
                executable="safe_one_arm_servo",
                name="safe_one_arm_servo",
                output="screen",
                parameters=[str(config)],
            ),
            Node(
                package="servo_controller",
                executable="safe_gripper_controller",
                name="safe_gripper_controller",
                output="screen",
                parameters=[str(config)],
            ),
        ]
    )
