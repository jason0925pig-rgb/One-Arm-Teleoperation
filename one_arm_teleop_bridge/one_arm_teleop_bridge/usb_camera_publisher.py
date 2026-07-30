#!/usr/bin/env python3
"""Publish a stable USB camera stream as timestamped ROS 2 JPEG frames."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def _resolve_device(explicit_device: str, serial: str) -> str:
    if explicit_device:
        return explicit_device
    if not serial:
        raise RuntimeError("set either the device or device_serial parameter")

    by_id = Path("/dev/v4l/by-id")
    candidates = sorted(
        path for path in by_id.glob(f"*{serial}*") if path.exists()
    )
    if not candidates:
        raise RuntimeError(
            f"no V4L2 camera containing serial {serial!r} was found in {by_id}"
        )
    preferred = [
        path for path in candidates if "video-index0" in path.name
    ]
    return str((preferred or candidates)[0])


class UsbCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("usb_camera_publisher")
        self.declare_parameter("camera_name", "camera")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("device", "")
        self.declare_parameter("device_serial", "")
        self.declare_parameter(
            "compressed_topic", "/cameras/camera/image_raw/compressed"
        )
        self.declare_parameter("status_topic", "/cameras/camera/status")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("fourcc", "MJPG")
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("strict_format", True)
        self.declare_parameter("auto_exposure", True)
        self.declare_parameter("exposure", -1.0)

        self.camera_name = str(self.get_parameter("camera_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.target_fps = float(self.get_parameter("fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.strict_format = bool(self.get_parameter("strict_format").value)
        if self.width <= 0 or self.height <= 0 or self.target_fps <= 0.0:
            raise RuntimeError("width, height and fps must be positive")
        if not 1 <= self.jpeg_quality <= 100:
            raise RuntimeError("jpeg_quality must be within 1..100")

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "OpenCV is required; install Ubuntu package python3-opencv"
            ) from exc
        self.cv2 = cv2

        device = _resolve_device(
            str(self.get_parameter("device").value),
            str(self.get_parameter("device_serial").value),
        )
        self.device = device
        capture_device: str | int = device
        if device.isdecimal():
            capture_device = int(device)
        self.capture = cv2.VideoCapture(capture_device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f"failed to open V4L2 camera {device}")

        fourcc = str(self.get_parameter("fourcc").value)
        if len(fourcc) != 4:
            self.capture.release()
            raise RuntimeError("fourcc must contain exactly four characters")
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.target_fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        auto_exposure = bool(self.get_parameter("auto_exposure").value)
        exposure = float(self.get_parameter("exposure").value)
        self.capture.set(
            cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if auto_exposure else 0.25
        )
        if not auto_exposure and exposure >= 0.0:
            self.capture.set(cv2.CAP_PROP_EXPOSURE, exposure)

        self.actual_width = int(
            round(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        )
        self.actual_height = int(
            round(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        )
        self.actual_reported_fps = float(
            self.capture.get(cv2.CAP_PROP_FPS)
        )
        format_errors: list[str] = []
        if self.actual_width != self.width or self.actual_height != self.height:
            format_errors.append(
                f"resolution is {self.actual_width}x{self.actual_height}, "
                f"requested {self.width}x{self.height}"
            )
        if (
            self.actual_reported_fps > 0.5
            and abs(self.actual_reported_fps - self.target_fps)
            > max(0.5, self.target_fps * 0.05)
        ):
            format_errors.append(
                f"reported FPS is {self.actual_reported_fps:.3f}, "
                f"requested {self.target_fps:.3f}"
            )
        if format_errors and self.strict_format:
            self.capture.release()
            raise RuntimeError("; ".join(format_errors))
        for error in format_errors:
            self.get_logger().warning(error)

        self.publisher = self.create_publisher(
            CompressedImage,
            str(self.get_parameter("compressed_topic").value),
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.frame_count = 0
        self.read_failures = 0
        self.encode_failures = 0
        self.deadline_misses = 0
        self.started_monotonic = time.monotonic()
        self.last_tick_monotonic: float | None = None
        self.timer = self.create_timer(
            1.0 / self.target_fps, self._capture_frame
        )
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"camera={self.camera_name} device={self.device} "
            f"format={self.actual_width}x{self.actual_height}"
            f"@{self.actual_reported_fps:.3f} topic="
            f"{self.get_parameter('compressed_topic').value}"
        )

    def _capture_frame(self) -> None:
        tick = time.monotonic()
        if self.last_tick_monotonic is not None:
            period = tick - self.last_tick_monotonic
            if period > (1.5 / self.target_fps):
                self.deadline_misses += 1
        self.last_tick_monotonic = tick

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.read_failures += 1
            self.get_logger().warning(
                "camera read failed",
                throttle_duration_sec=1.0,
            )
            return
        ok, encoded = self.cv2.imencode(
            ".jpg",
            frame,
            [self.cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            self.encode_failures += 1
            return

        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.publisher.publish(message)
        self.frame_count += 1

    def _publish_status(self) -> None:
        elapsed = max(time.monotonic() - self.started_monotonic, 1e-9)
        status = String()
        status.data = (
            f"camera={self.camera_name};device={self.device};"
            f"target_fps={self.target_fps:.3f};"
            f"measured_publish_fps={self.frame_count / elapsed:.3f};"
            f"frames={self.frame_count};read_failures={self.read_failures};"
            f"encode_failures={self.encode_failures};"
            f"deadline_misses={self.deadline_misses};"
            f"width={self.actual_width};height={self.actual_height}"
        )
        self.status_publisher.publish(status)

    def destroy_node(self) -> bool:
        if hasattr(self, "capture"):
            self.capture.release()
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node: UsbCameraPublisher | None = None
    try:
        node = UsbCameraPublisher()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"FATAL: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
