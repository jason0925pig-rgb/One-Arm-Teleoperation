from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Start only the head and right-wrist Orbbec RGB streams.

    The Armstrong also has two chest cameras. They are deliberately absent
    from this launch description and therefore cannot enter the dataset.
    Serial numbers were verified against the legacy mocap recorder on
    2026-07-31.
    """
    orbbec_share = get_package_share_directory("orbbec_camera")
    driver_launch = PythonLaunchDescriptionSource(
        f"{orbbec_share}/launch/gemini_330_series.launch.py"
    )

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
        "sync_mode": "free_run",
    }

    head = IncludeLaunchDescription(
        driver_launch,
        launch_arguments={
            **common_arguments,
            "camera_name": "camera_head",
            "serial_number": "CPCD7530003J",
        }.items(),
    )
    wrist = IncludeLaunchDescription(
        driver_launch,
        launch_arguments={
            **common_arguments,
            "camera_name": "camera_wrist",
            "serial_number": "CPCBC5300077",
        }.items(),
    )
    return LaunchDescription(
        [
            GroupAction([head]),
            # Stagger USB initialization to avoid both Gemini devices opening
            # their color endpoints in the same scheduler tick.
            TimerAction(period=3.0, actions=[GroupAction([wrist])]),
        ]
    )
