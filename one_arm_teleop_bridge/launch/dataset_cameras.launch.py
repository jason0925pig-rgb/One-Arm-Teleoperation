from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start one fixed primary camera and the right-wrist Orbbec RGB stream.

    The primary camera can be named ``camera_head`` for the legacy robot or
    ``camera_chest`` for the Humble robot. Cameras not selected by serial
    number are deliberately absent from the dataset.
    """
    orbbec_share = get_package_share_directory("orbbec_camera")
    driver_launch = PythonLaunchDescriptionSource(
        f"{orbbec_share}/launch/gemini_330_series.launch.py"
    )
    primary_camera_name = LaunchConfiguration("primary_camera_name")
    head_serial = LaunchConfiguration("head_serial")
    wrist_serial = LaunchConfiguration("wrist_serial")

    common_arguments = {
        "device_num": "2",
        "color_width": "1280",
        "color_height": "720",
        "color_fps": "30",
        "color_format": "MJPG",
        "enable_color": "true",
        "enable_depth": "false",
        "enable_left_ir": "false",
        "enable_right_ir": "false",
        "enable_accel": "false",
        "enable_gyro": "false",
        "enable_ldp": "false",
        "enable_laser": "false",
        "enable_point_cloud": "false",
        "enable_colored_point_cloud": "false",
        "publish_tf": "false",
        "enable_frame_sync": "true",
        "enable_sync_host_time": "true",
        "use_hardware_time": "true",
        # These cameras do not share a hardware sync cable.  The Humble
        # Orbbec driver also uses standalone in its own multi-camera launch.
        "sync_mode": "standalone",
    }

    head = IncludeLaunchDescription(
        driver_launch,
        launch_arguments={
            **common_arguments,
            "camera_name": primary_camera_name,
            "serial_number": head_serial,
        }.items(),
    )
    wrist = IncludeLaunchDescription(
        driver_launch,
        launch_arguments={
            **common_arguments,
            "camera_name": "camera_wrist",
            "serial_number": wrist_serial,
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "primary_camera_name",
                default_value="camera_head",
                description="ROS name for the fixed primary camera",
            ),
            DeclareLaunchArgument(
                "head_serial",
                default_value="CPCD7530003J",
                description="Serial number of the fixed primary camera",
            ),
            DeclareLaunchArgument(
                "wrist_serial",
                default_value="CPCBC5300077",
                description="Serial number of the right-wrist camera",
            ),
            GroupAction([head]),
            # Stagger USB initialization to avoid both Gemini devices opening
            # their color endpoints in the same scheduler tick.
            TimerAction(period=3.0, actions=[GroupAction([wrist])]),
        ]
    )
