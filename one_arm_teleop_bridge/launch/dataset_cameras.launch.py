from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("one_arm_teleop_bridge"))
    config = share / "config" / "dataset_cameras.yaml"
    return LaunchDescription(
        [
            Node(
                package="one_arm_teleop_bridge",
                executable="usb_camera_publisher",
                name="head_camera_publisher",
                output="screen",
                parameters=[str(config)],
            ),
            Node(
                package="one_arm_teleop_bridge",
                executable="usb_camera_publisher",
                name="wrist_camera_publisher",
                output="screen",
                parameters=[str(config)],
            ),
        ]
    )
