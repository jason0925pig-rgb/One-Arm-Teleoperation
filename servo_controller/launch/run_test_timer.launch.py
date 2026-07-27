import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_inspire").to_moveit_configs()

    robot_monitor_node = Node(
        package="servo_controller",
        executable="robot_timer",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("simple_planning_demo"),
        "launch",
        "moveit_cpp_walle.rviz",
    )    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    return LaunchDescription([robot_monitor_node])