from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = Path(get_package_share_directory("one_arm_teleop_bridge"))
    servo_share = Path(get_package_share_directory("servo_controller"))
    bridge_config = bridge_share / "config" / "teleop_bridge.yaml"
    servo_config = servo_share / "config" / "safe_one_arm.yaml"

    return LaunchDescription(
        [
            Node(
                package="servo_controller",
                executable="safe_one_arm_servo",
                name="safe_one_arm_servo",
                output="screen",
                parameters=[str(servo_config)],
            ),
            Node(
                package="servo_controller",
                executable="safe_gripper_controller",
                name="safe_gripper_controller",
                output="screen",
                parameters=[str(servo_config)],
            ),
            Node(
                package="one_arm_teleop_bridge",
                executable="udp_leader_bridge",
                name="udp_leader_bridge",
                output="screen",
                parameters=[str(bridge_config)],
            ),
        ]
    )
