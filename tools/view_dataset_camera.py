#!/usr/bin/env python3
"""Display one dataset camera without invoking ROS image_transport plugins."""

from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


CAMERA_TOPICS = {
    "chest": "/camera_chest/color/image_raw/compressed",
    "wrist": "/camera_wrist/color/image_raw/compressed",
}


class CompressedCameraViewer(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("one_arm_camera_viewer")
        self.topic = topic
        self.frame = None
        self.frame_count = 0
        self.create_subscription(
            CompressedImage,
            topic,
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_image(self, message: CompressedImage) -> None:
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is not None:
            self.frame = frame
            self.frame_count += 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("camera", choices=tuple(CAMERA_TOPICS))
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--snapshot",
        help="Save the first decoded frame and exit instead of opening a window.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    topic = CAMERA_TOPICS[args.camera]
    rclpy.init()
    node = CompressedCameraViewer(topic)
    deadline = time.monotonic() + args.timeout
    print(f"Waiting for {args.camera} camera: {topic}")
    try:
        while rclpy.ok() and node.frame is None:
            rclpy.spin_once(node, timeout_sec=0.10)
            if time.monotonic() >= deadline:
                print(f"ERROR: no frame received from {topic}", file=sys.stderr)
                return 2

        if args.snapshot:
            if not cv2.imwrite(args.snapshot, node.frame):
                print(f"ERROR: could not write {args.snapshot}", file=sys.stderr)
                return 3
            print(f"Saved snapshot: {args.snapshot}")
            return 0

        print("Camera window opened. Press Q or Esc in the image window to close.")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            cv2.imshow(f"One-Arm {args.camera}", node.frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
