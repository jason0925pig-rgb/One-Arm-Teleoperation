"""Receive Windows leader frames and publish safety-gated ROS 2 targets."""

from __future__ import annotations

import socket
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool

from .core import (
    JOINT_COUNT,
    MappingConfig,
    MultiJointUnwrapper,
    OffsetAbsoluteMapper,
    PacketError,
    SafetyError,
    parse_leader_packet,
)


class UdpLeaderBridge(Node):
    def __init__(self) -> None:
        super().__init__("udp_leader_bridge")
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 5005)
        self.declare_parameter("expected_source_ip", "")
        self.declare_parameter("arm_name", "right")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("calibration_complete", False)
        self.declare_parameter("leader_period_pulses", 2500)
        self.declare_parameter("max_leader_step_pulses", 800.0)
        self.declare_parameter("packet_timeout_seconds", 0.30)
        self.declare_parameter("follower_state_timeout_seconds", 0.30)
        self.declare_parameter("joint_signs", [1.0] * JOINT_COUNT)
        self.declare_parameter("scale_rad_per_pulse", [0.0] * JOINT_COUNT)
        self.declare_parameter("joint_lower_limits", [0.0] * JOINT_COUNT)
        self.declare_parameter("joint_upper_limits", [0.0] * JOINT_COUNT)

        self.arm_name = str(self.get_parameter("arm_name").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.calibration_complete = bool(
            self.get_parameter("calibration_complete").value
        )
        self.packet_timeout = float(
            self.get_parameter("packet_timeout_seconds").value
        )
        self.follower_timeout = float(
            self.get_parameter("follower_state_timeout_seconds").value
        )
        self.expected_source_ip = str(
            self.get_parameter("expected_source_ip").value
        )
        self.joint_names = [
            f"{self.arm_name}_joint{index}" for index in range(1, 8)
        ]

        self.mapping_error: str | None = None
        self.mapper: OffsetAbsoluteMapper | None = None
        try:
            config = MappingConfig(
                signs=self._float_tuple("joint_signs"),
                scale_rad_per_pulse=self._float_tuple("scale_rad_per_pulse"),
                lower_limits=self._float_tuple("joint_lower_limits"),
                upper_limits=self._float_tuple("joint_upper_limits"),
            )
            self.mapper = OffsetAbsoluteMapper(config)
        except SafetyError as exc:
            self.mapping_error = str(exc)

        self.unwrapper = MultiJointUnwrapper(
            int(self.get_parameter("leader_period_pulses").value),
            float(self.get_parameter("max_leader_step_pulses").value),
        )
        self.current_session: str | None = None
        self.last_sequence: int | None = None
        self.last_leader_position: tuple[float, ...] | None = None
        self.last_leader_received = 0.0
        self.last_follower_position: tuple[float, ...] | None = None
        self.last_follower_received = 0.0
        self.mapping_enabled = False

        bind_host = str(self.get_parameter("bind_host").value)
        bind_port = int(self.get_parameter("bind_port").value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((bind_host, bind_port))

        prefix = f"/{self.arm_name}_arm"
        self.command_pub = self.create_publisher(
            JointState, prefix + "/teleop_joint_command", 10
        )
        self.preview_pub = self.create_publisher(
            JointState, "/teleop/target_preview", 10
        )
        self.raw_pub = self.create_publisher(
            Float64MultiArray, "/teleop/leader_pulses", 10
        )
        self.gripper_pub = self.create_publisher(
            Bool, prefix + "/gripper_command", 10
        )
        self.status_pub = self.create_publisher(String, "/teleop/bridge_status", 10)
        self.enabled_pub = self.create_publisher(Bool, "/teleop/enabled", 10)
        self.create_subscription(
            JointState,
            prefix + "/joint_states",
            self._follower_state_callback,
            10,
        )
        self.create_service(SetBool, "/teleop/set_enabled", self._set_enabled)
        self.create_timer(0.005, self._receive_packets)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().warn(
            f"UDP bridge listening on {bind_host}:{bind_port}; "
            f"dry_run={self.dry_run}, calibration_complete={self.calibration_complete}"
        )
        if self.mapping_error:
            self.get_logger().warn(f"Mapping is not usable: {self.mapping_error}")

    def _float_tuple(self, parameter_name: str) -> tuple[float, ...]:
        return tuple(float(value) for value in self.get_parameter(parameter_name).value)

    def destroy_node(self) -> bool:
        self.sock.close()
        return super().destroy_node()

    def _set_enabled(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if not request.data:
            self._disable("operator request")
            response.success = True
            response.message = "teleoperation mapping disabled"
            return response

        now = time.monotonic()
        reasons: list[str] = []
        if not self.calibration_complete:
            reasons.append("calibration_complete is false")
        if self.mapper is None:
            reasons.append(self.mapping_error or "mapping configuration is invalid")
        if self.last_leader_position is None or now - self.last_leader_received > self.packet_timeout:
            reasons.append("no recent leader packet")
        if (
            self.last_follower_position is None
            or now - self.last_follower_received > self.follower_timeout
        ):
            reasons.append("no recent follower state")
        if reasons:
            response.success = False
            response.message = "; ".join(reasons)
            return response

        assert self.mapper is not None
        assert self.last_leader_position is not None
        assert self.last_follower_position is not None
        self.mapper.set_reference(
            self.last_leader_position,
            self.last_follower_position,
        )
        self.mapping_enabled = True
        response.success = True
        response.message = (
            "mapping enabled in preview-only dry-run mode"
            if self.dry_run
            else "mapping enabled; command publication active"
        )
        self.get_logger().warn(response.message)
        return response

    def _disable(self, reason: str) -> None:
        if self.mapping_enabled:
            self.get_logger().error(f"Teleoperation disabled: {reason}")
        self.mapping_enabled = False

    def _follower_state_callback(self, msg: JointState) -> None:
        try:
            self.last_follower_position = self._ordered_positions(msg)
            self.last_follower_received = time.monotonic()
        except SafetyError as exc:
            self._disable(f"invalid follower state: {exc}")

    def _ordered_positions(self, msg: JointState) -> tuple[float, ...]:
        if len(msg.position) != JOINT_COUNT:
            raise SafetyError("follower state does not contain seven positions")
        if msg.name:
            if len(msg.name) != JOINT_COUNT:
                raise SafetyError("follower state names do not contain seven entries")
            lookup = dict(zip(msg.name, msg.position))
            if not all(name in lookup for name in self.joint_names):
                raise SafetyError("follower joint names do not match configured arm")
            return tuple(float(lookup[name]) for name in self.joint_names)
        return tuple(float(value) for value in msg.position)

    def _receive_packets(self) -> None:
        while True:
            try:
                payload, source = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                self._disable(f"UDP receive error: {exc}")
                return
            if self.expected_source_ip and source[0] != self.expected_source_ip:
                continue
            try:
                frame = parse_leader_packet(payload)
                if frame.session_id != self.current_session:
                    self.current_session = frame.session_id
                    self.last_sequence = None
                    self.unwrapper.reset()
                    self._disable("leader session changed")
                if self.last_sequence is not None and frame.sequence <= self.last_sequence:
                    raise PacketError("sequence is not newer than the previous packet")
                leader_position = self.unwrapper.update(frame.joint_pulses)
            except (PacketError, SafetyError) as exc:
                self._disable(str(exc))
                continue

            self.last_sequence = frame.sequence
            self.last_leader_position = leader_position
            self.last_leader_received = time.monotonic()
            raw_msg = Float64MultiArray()
            raw_msg.data = [float(value) for value in leader_position]
            self.raw_pub.publish(raw_msg)

            if not self.mapping_enabled or self.mapper is None:
                continue
            try:
                target = self.mapper.map(leader_position)
            except SafetyError as exc:
                self._disable(str(exc))
                continue

            target_msg = JointState()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.name = self.joint_names
            target_msg.position = list(target)
            self.preview_pub.publish(target_msg)
            if not self.dry_run:
                self.command_pub.publish(target_msg)
                if frame.gripper_state in {"OPEN", "CLOSED"}:
                    gripper_msg = Bool()
                    gripper_msg.data = frame.gripper_state == "OPEN"
                    self.gripper_pub.publish(gripper_msg)

    def _watchdog(self) -> None:
        if not self.mapping_enabled:
            return
        now = time.monotonic()
        if now - self.last_leader_received > self.packet_timeout:
            self._disable("leader packet timeout")
        elif now - self.last_follower_received > self.follower_timeout:
            self._disable("follower state timeout")

    def _publish_status(self) -> None:
        enabled = Bool()
        enabled.data = self.mapping_enabled
        self.enabled_pub.publish(enabled)
        status = String()
        status.data = (
            f"enabled={self.mapping_enabled};dry_run={self.dry_run};"
            f"calibration_complete={self.calibration_complete};"
            f"session={self.current_session or 'none'};"
            f"sequence={self.last_sequence if self.last_sequence is not None else -1}"
        )
        self.status_pub.publish(status)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UdpLeaderBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
