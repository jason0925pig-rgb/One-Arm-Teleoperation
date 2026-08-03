#!/usr/bin/env python3
"""Serve two ROS 2 CompressedImage topics in one lightweight web page."""

from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


PAGE = b"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>One-Arm camera preview</title>
  <style>
    html,body { margin:0; height:100%; background:#111; color:#eee;
      font-family:Segoe UI,Arial,sans-serif; overflow:hidden; }
    header { height:42px; display:flex; align-items:center; padding:0 16px;
      background:#202020; font-size:16px; box-sizing:border-box; }
    main { height:calc(100% - 42px); display:grid; grid-template-columns:1fr 1fr;
      gap:6px; padding:6px; box-sizing:border-box; }
    section { position:relative; display:flex; align-items:center;
      justify-content:center; min-width:0; background:#050505; }
    img { width:100%; height:100%; object-fit:contain; }
    span { position:absolute; top:10px; left:10px; padding:5px 9px;
      border-radius:4px; background:rgba(0,0,0,.65); }
  </style>
</head>
<body>
  <header>One-Arm Teleoperation / live preview</header>
  <main>
    <section><span>Chest camera</span><img src="/stream/chest"></section>
    <section><span>Right wrist camera</span><img src="/stream/wrist"></section>
  </main>
</body>
</html>
"""


class Feed:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.condition = threading.Condition()
        self.data = b""
        self.content_type = "image/jpeg"
        self.sequence = 0
        self.updated_monotonic = 0.0

    def update(self, message: CompressedImage) -> None:
        image_format = message.format.lower()
        content_type = "image/png" if "png" in image_format else "image/jpeg"
        with self.condition:
            self.data = bytes(message.data)
            self.content_type = content_type
            self.sequence += 1
            self.updated_monotonic = time.monotonic()
            self.condition.notify_all()


class PreviewNode(Node):
    def __init__(self, chest_topic: str, wrist_topic: str) -> None:
        super().__init__("one_arm_dual_camera_preview")
        self.feeds = {
            "chest": Feed(chest_topic),
            "wrist": Feed(wrist_topic),
        }
        self.create_subscription(
            CompressedImage,
            chest_topic,
            self.feeds["chest"].update,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            wrist_topic,
            self.feeds["wrist"].update,
            qos_profile_sensor_data,
        )


def make_handler(node: PreviewNode):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, _format: str, *_args: object) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            path = self.path.split("?", 1)[0]
            if path == "/":
                self._send_bytes("text/html; charset=utf-8", PAGE)
                return
            if path == "/health":
                now = time.monotonic()
                payload = {
                    name: {
                        "topic": feed.topic,
                        "frames": feed.sequence,
                        "age_seconds": (
                            None
                            if feed.updated_monotonic == 0.0
                            else round(now - feed.updated_monotonic, 3)
                        ),
                    }
                    for name, feed in node.feeds.items()
                }
                self._send_bytes(
                    "application/json",
                    json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                )
                return
            if path.startswith("/stream/"):
                name = path.removeprefix("/stream/")
                feed = node.feeds.get(name)
                if feed is not None:
                    self._stream(feed)
                    return
            self.send_error(HTTPStatus.NOT_FOUND)

        def _send_bytes(self, content_type: str, payload: bytes) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def _stream(self, feed: Feed) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.end_headers()
            last_sequence = -1
            try:
                while rclpy.ok():
                    with feed.condition:
                        feed.condition.wait_for(
                            lambda: feed.sequence != last_sequence, timeout=2.0
                        )
                        if not feed.data or feed.sequence == last_sequence:
                            continue
                        payload = feed.data
                        content_type = feed.content_type
                        last_sequence = feed.sequence
                    header = (
                        "--frame\r\n"
                        f"Content-Type: {content_type}\r\n"
                        f"Content-Length: {len(payload)}\r\n\r\n"
                    ).encode("ascii")
                    self.wfile.write(header)
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument(
        "--chest-topic", default="/camera_chest/color/image_raw/compressed"
    )
    parser.add_argument(
        "--wrist-topic", default="/camera_wrist/color/image_raw/compressed"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = PreviewNode(args.chest_topic, args.wrist_topic)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(node))
    server.daemon_threads = True
    def interrupt_server(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    # Background jobs commonly inherit SIGINT as ignored. Reinstall explicit
    # handlers so the dataset lifecycle can stop this process promptly.
    signal.signal(signal.SIGINT, interrupt_server)
    signal.signal(signal.SIGTERM, interrupt_server)
    node.get_logger().info(
        f"Dual-camera preview listening on http://{args.host}:{args.port}"
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
