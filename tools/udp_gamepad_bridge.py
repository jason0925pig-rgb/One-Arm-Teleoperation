#!/usr/bin/env python3
"""Receive authenticated-by-source Windows gamepad joint targets on the Orin.

This node never powers or enables hardware.  It owns the sole ROS command
publisher and only publishes while /teleop/set_enabled is true.  Network loss,
an explicit STOP, or stale joint feedback closes the gate and publishes STOP.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class UdpGamepadBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("udp_gamepad_bridge")
        self.expected_ip = args.expected_source_ip
        self.packet_timeout = args.packet_timeout
        self.state_timeout = args.state_timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((args.bind_host, args.port))
        self.peer: tuple[str, int] | None = None
        self.last_packet = 0.0
        self.last_state = 0.0
        self.state: tuple[float, ...] | None = None
        self.enabled = False
        self.session = "none"
        self.last_sequence = -1
        self.accepted = 0
        self.rejected = 0
        self.last_reason = "none"
        self.command_pub = self.create_publisher(
            JointState, "/right_arm/teleop_joint_command", 10
        )
        self.gripper_pub = self.create_publisher(Bool, "/right_arm/gripper_command", 10)
        self.stop_pub = self.create_publisher(Bool, "/teleop/stop_request", 10)
        self.status_pub = self.create_publisher(String, "/teleop/bridge_status", 10)
        self.create_subscription(
            JointState, "/right_arm/joint_states", self.on_joint_state, 10
        )
        self.create_service(SetBool, "/teleop/set_enabled", self.set_enabled)
        self.create_timer(0.005, self.receive)
        self.create_timer(0.05, self.send_state)
        self.create_timer(0.05, self.watchdog)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().warning(
            f"Windows gamepad UDP bridge listening on {args.bind_host}:{args.port}; "
            f"expected_source_ip={self.expected_ip}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        if len(msg.position) < 7:
            return
        values = tuple(float(v) for v in msg.position[:7])
        if all(math.isfinite(v) for v in values):
            self.state = values
            self.last_state = time.monotonic()

    def reject(self, reason: str) -> None:
        self.rejected += 1
        self.last_reason = reason

    def request_stop(self, reason: str) -> None:
        if self.enabled:
            self.stop_pub.publish(Bool(data=True))
        self.enabled = False
        self.last_reason = reason
        self.get_logger().error(f"GAMEPAD_BRIDGE_STOP: {reason}")

    def receive(self) -> None:
        while True:
            try:
                payload, address = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            if address[0] != self.expected_ip:
                self.reject(f"unexpected_source:{address[0]}")
                continue
            try:
                packet = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.reject("invalid_json")
                continue
            if packet.get("protocol") != "onearm_gamepad_v1":
                self.reject("protocol")
                continue
            incoming_session = str(packet.get("session", "none"))[:80]
            kind = packet.get("type")
            if kind == "hello" and incoming_session != self.session:
                self.last_sequence = -1
            sequence = packet.get("sequence")
            if not isinstance(sequence, int) or sequence <= self.last_sequence:
                self.reject("sequence")
                continue
            # Laptop and Orin wall clocks are not assumed synchronized. Freshness
            # is enforced from local receive time plus source/session/sequence.
            self.peer = address
            self.last_packet = time.monotonic()
            self.last_sequence = sequence
            self.session = incoming_session
            if kind == "stop":
                self.request_stop("operator_stop")
                self.accepted += 1
                continue
            if kind == "hello":
                self.accepted += 1
                continue
            if kind != "command":
                self.reject("packet_type")
                continue
            positions = packet.get("positions")
            if not isinstance(positions, list) or len(positions) != 7:
                self.reject("positions")
                continue
            positions = [float(v) for v in positions]
            if not all(math.isfinite(v) for v in positions):
                self.reject("nonfinite_positions")
                continue
            if not self.enabled:
                self.reject("gate_disabled")
                continue
            out = JointState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.name = [f"right_joint_{i}" for i in range(1, 8)]
            out.position = positions
            self.command_pub.publish(out)
            self.gripper_pub.publish(Bool(data=bool(packet.get("gripper_open", True))))
            self.accepted += 1

    def send_state(self) -> None:
        if self.peer is None or self.state is None:
            return
        packet = {
            "protocol": "onearm_gamepad_v1",
            "type": "state",
            "wall_time": time.time(),
            "positions": self.state,
        }
        try:
            self.sock.sendto(json.dumps(packet, separators=(",", ":")).encode(), self.peer)
        except OSError as exc:
            self.last_reason = f"state_send:{exc}"

    def set_enabled(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            self.request_stop("gate_disabled_by_service")
            response.success = True
            response.message = "gamepad mapping disabled and STOP published"
            return response
        reasons: list[str] = []
        now = time.monotonic()
        if self.peer is None or now - self.last_packet > self.packet_timeout:
            reasons.append("no recent Windows gamepad packet")
        if self.state is None or now - self.last_state > self.state_timeout:
            reasons.append("joint state missing or stale")
        if reasons:
            response.success = False
            response.message = "; ".join(reasons)
            return response
        self.enabled = True
        response.success = True
        response.message = "gamepad mapping enabled"
        return response

    def watchdog(self) -> None:
        if not self.enabled:
            return
        now = time.monotonic()
        if now - self.last_packet > self.packet_timeout:
            self.request_stop("Windows gamepad packet timeout")
        elif now - self.last_state > self.state_timeout:
            self.request_stop("joint state timeout")

    def publish_status(self) -> None:
        age = time.monotonic() - self.last_packet if self.last_packet else -1.0
        state_age = time.monotonic() - self.last_state if self.last_state else -1.0
        msg = String()
        msg.data = (
            f"mode=windows_gamepad;enabled={int(self.enabled)};session={self.session};"
            f"source={self.peer[0] if self.peer else 'none'};packet_age={age:.3f};"
            f"state_age={state_age:.3f};accepted={self.accepted};rejected={self.rejected};"
            f"last_reason={self.last_reason}"
        )
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5010)
    parser.add_argument("--expected-source-ip", required=True)
    parser.add_argument("--packet-timeout", type=float, default=0.35)
    parser.add_argument("--state-timeout", type=float, default=1.5)
    args = parser.parse_args()
    rclpy.init()
    node = UdpGamepadBridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
