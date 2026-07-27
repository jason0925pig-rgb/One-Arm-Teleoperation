from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("one_arm_teleop_bridge"))
    config = share / "config" / "teleop_bridge.yaml"
    return LaunchDescription(
        [
            Node(
                package="one_arm_teleop_bridge",
                executable="udp_leader_bridge",
                name="udp_leader_bridge",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
