"""Receive Windows leader frames and publish safety-gated ROS 2 targets."""

from __future__ import annotations

import socket
import time
from collections import deque
from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool

from .core import (
    JOINT_COUNT,
    JointPulseDropoutGuard,
    LeaderSignalFilter,
    MappingConfig,
    MultiJointUnwrapper,
    OffsetAbsoluteMapper,
    PacketError,
    PersistentJointDropoutError,
    SafetyError,
    StopFrame,
    parse_teleop_packet,
    validate_session_packet_timestamp,
)


class UdpLeaderBridge(Node):
    def __init__(self) -> None:
        super().__init__("udp_leader_bridge")
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 5005)
        self.declare_parameter("expected_source_ip", "")
        self.declare_parameter("require_expected_source_ip_for_motion", True)
        self.declare_parameter("require_single_command_subscriber_for_motion", True)
        self.declare_parameter("arm_name", "right")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("calibration_complete", False)
        self.declare_parameter("gripper_only_mode", False)
        self.declare_parameter("arm_before_deadman", False)
        self.declare_parameter("leader_period_pulses", 2000)
        self.declare_parameter("leader_min_valid_pulse", 500)
        self.declare_parameter("leader_max_valid_pulse", 2500)
        self.declare_parameter("leader_joint_dropout_hold_seconds", 0.0)
        self.declare_parameter("max_leader_step_pulses", 800.0)
        self.declare_parameter("median_filter_window", 1)
        self.declare_parameter("low_pass_alpha", 1.0)
        self.declare_parameter("leader_deadband_pulses", 0.0)
        self.declare_parameter("require_deadman_for_motion", True)
        self.declare_parameter("enforce_packet_timestamps_for_motion", True)
        self.declare_parameter("max_packet_age_seconds", 0.50)
        self.declare_parameter("max_future_skew_seconds", 0.25)
        self.declare_parameter("packet_timeout_seconds", 0.30)
        self.declare_parameter("stop_on_packet_timeout", True)
        self.declare_parameter("stop_on_rejected_packet", False)
        self.declare_parameter("minimum_packet_rate_hz_for_motion", 0.0)
        self.declare_parameter("follower_state_timeout_seconds", 0.30)
        self.declare_parameter("stop_on_follower_state_timeout", True)
        self.declare_parameter("joint_signs", [1.0] * JOINT_COUNT)
        self.declare_parameter("scale_rad_per_pulse", [0.0] * JOINT_COUNT)
        self.declare_parameter("joint_lower_limits", [0.0] * JOINT_COUNT)
        self.declare_parameter("joint_upper_limits", [0.0] * JOINT_COUNT)

        self.arm_name = str(self.get_parameter("arm_name").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.calibration_complete = bool(
            self.get_parameter("calibration_complete").value
        )
        self.gripper_only_mode = bool(
            self.get_parameter("gripper_only_mode").value
        )
        self.arm_before_deadman = bool(
            self.get_parameter("arm_before_deadman").value
        )
        self.packet_timeout = float(
            self.get_parameter("packet_timeout_seconds").value
        )
        self.stop_on_packet_timeout = bool(
            self.get_parameter("stop_on_packet_timeout").value
        )
        self.stop_on_rejected_packet = bool(
            self.get_parameter("stop_on_rejected_packet").value
        )
        self.minimum_packet_rate_hz = float(
            self.get_parameter("minimum_packet_rate_hz_for_motion").value
        )
        self.follower_timeout = float(
            self.get_parameter("follower_state_timeout_seconds").value
        )
        self.stop_on_follower_timeout = bool(
            self.get_parameter("stop_on_follower_state_timeout").value
        )
        self.expected_source_ip = str(
            self.get_parameter("expected_source_ip").value
        )
        self.require_expected_source_ip = bool(
            self.get_parameter("require_expected_source_ip_for_motion").value
        )
        self.require_single_command_subscriber = bool(
            self.get_parameter(
                "require_single_command_subscriber_for_motion"
            ).value
        )
        self.require_deadman = bool(
            self.get_parameter("require_deadman_for_motion").value
        )
        self.enforce_packet_timestamps = bool(
            self.get_parameter("enforce_packet_timestamps_for_motion").value
        )
        self.max_packet_age = float(
            self.get_parameter("max_packet_age_seconds").value
        )
        self.max_future_skew = float(
            self.get_parameter("max_future_skew_seconds").value
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
        self.leader_min_valid_pulse = int(
            self.get_parameter("leader_min_valid_pulse").value
        )
        self.leader_max_valid_pulse = int(
            self.get_parameter("leader_max_valid_pulse").value
        )
        if self.leader_min_valid_pulse >= self.leader_max_valid_pulse:
            raise ValueError(
                "leader_min_valid_pulse must be below leader_max_valid_pulse"
            )
        self.joint_dropout_guard = JointPulseDropoutGuard(
            self.leader_min_valid_pulse,
            self.leader_max_valid_pulse,
            float(
                self.get_parameter("leader_joint_dropout_hold_seconds").value
            ),
        )
        self.signal_filter = LeaderSignalFilter(
            median_window=int(self.get_parameter("median_filter_window").value),
            low_pass_alpha=float(self.get_parameter("low_pass_alpha").value),
            deadband_pulses=float(
                self.get_parameter("leader_deadband_pulses").value
            ),
        )
        self.current_session: str | None = None
        self.session_sender_monotonic_origin_ns: int | None = None
        self.session_receiver_monotonic_origin_ns: int | None = None
        self.last_sequence: int | None = None
        self.last_leader_position: tuple[float, ...] | None = None
        self.last_filtered_position: tuple[float, ...] | None = None
        self.last_deadman_held = False
        self.last_packet_age_seconds: float | None = None
        self.last_leader_received = 0.0
        self.accepted_packet_times: deque[float] = deque(maxlen=64)
        self.last_follower_position: tuple[float, ...] | None = None
        self.last_follower_received = 0.0
        self.mapping_enabled = False
        self.accepted_packets = 0
        self.consecutive_accepted_packets = 0
        self.rejected_packets = 0
        self.last_rejection_reason = "none"
        self.stop_requests = 0
        self.held_joint_samples = 0
        self.last_held_joint_indices: tuple[int, ...] = ()
        self.last_dropout_warning = 0.0

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
        self.filtered_pub = self.create_publisher(
            Float64MultiArray, "/teleop/leader_filtered_pulses", 10
        )
        self.gripper_pub = self.create_publisher(
            Bool, prefix + "/gripper_command", 10
        )
        self.stop_pub = self.create_publisher(Bool, "/teleop/stop_request", 10)
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
        if not self.dry_run and self.require_expected_source_ip and not self.expected_source_ip:
            self.get_logger().error(
                "Real command publication is locked until expected_source_ip is configured."
            )

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
            self._request_stop("operator disabled teleoperation")
            response.success = True
            response.message = "teleoperation mapping disabled and STOP published"
            return response

        now = time.monotonic()
        reasons: list[str] = []
        if (
            self.last_filtered_position is None
            or now - self.last_leader_received > self.packet_timeout
        ):
            reasons.append("no recent leader packet")
        packet_rate = self._recent_packet_rate_hz()
        if packet_rate < self.minimum_packet_rate_hz:
            reasons.append(
                f"leader packet rate {packet_rate:.2f} Hz is below "
                f"{self.minimum_packet_rate_hz:.2f} Hz"
            )
        if not self.gripper_only_mode:
            if not self.calibration_complete:
                reasons.append("calibration_complete is false")
            if self.mapper is None:
                reasons.append(
                    self.mapping_error or "mapping configuration is invalid"
                )
            if (
                self.last_follower_position is None
                or now - self.last_follower_received > self.follower_timeout
            ):
                reasons.append("no recent follower state")
        if not self.dry_run:
            if self.require_expected_source_ip and not self.expected_source_ip:
                reasons.append("expected_source_ip is empty")
            if self.require_single_command_subscriber:
                active_publisher = (
                    self.gripper_pub
                    if self.gripper_only_mode
                    else self.command_pub
                )
                subscriber_count = active_publisher.get_subscription_count()
                if subscriber_count != 1:
                    command_kind = (
                        "gripper" if self.gripper_only_mode else "joint"
                    )
                    reasons.append(
                        f"expected exactly one {command_kind} command "
                        f"subscriber, found {subscriber_count}"
                    )
            if not self.enforce_packet_timestamps:
                reasons.append("packet timestamp enforcement is disabled")
            if (
                not (self.gripper_only_mode or self.arm_before_deadman)
                and self.require_deadman
                and not self.last_deadman_held
            ):
                reasons.append("Windows deadman is not held")
        if reasons:
            response.success = False
            response.message = "; ".join(reasons)
            return response

        if not self.gripper_only_mode:
            assert self.mapper is not None
            assert self.last_filtered_position is not None
            assert self.last_follower_position is not None
            self.mapper.set_reference(
                self.last_filtered_position,
                self.last_follower_position,
            )
        self.mapping_enabled = True
        response.success = True
        if self.gripper_only_mode:
            response.message = (
                "gripper-only bridge armed; waiting for Windows Space"
            )
        else:
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

    def _request_stop(self, reason: str) -> None:
        self._disable(reason)
        message = Bool()
        message.data = True
        self.stop_pub.publish(message)
        self.stop_requests += 1
        self.get_logger().error(f"STOP published: {reason}")

    def _recent_packet_rate_hz(self) -> float:
        if len(self.accepted_packet_times) < 2:
            return 0.0
        duration = (
            self.accepted_packet_times[-1] -
            self.accepted_packet_times[0]
        )
        if duration <= 0.0:
            return 0.0
        return (len(self.accepted_packet_times) - 1) / duration

    def _follower_state_callback(self, msg: JointState) -> None:
        try:
            self.last_follower_position = self._ordered_positions(msg)
            self.last_follower_received = time.monotonic()
        except SafetyError as exc:
            if self.mapping_enabled:
                self._request_stop(f"invalid follower state: {exc}")

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
                if self.mapping_enabled:
                    self._request_stop(f"UDP receive error: {exc}")
                return
            if self.expected_source_ip and source[0] != self.expected_source_ip:
                self.rejected_packets += 1
                self.consecutive_accepted_packets = 0
                self.last_rejection_reason = (
                    f"unexpected_source:{source[0]}"
                )
                continue
            try:
                frame = parse_teleop_packet(payload)
                if isinstance(frame, StopFrame):
                    self._request_stop(
                        f"Windows STOP ({frame.reason}, session={frame.session_id})"
                    )
                    continue
                received_monotonic_ns = time.monotonic_ns()
                new_session = frame.session_id != self.current_session
                if new_session:
                    sender_origin_ns = frame.sender_monotonic_ns
                    receiver_origin_ns = received_monotonic_ns
                else:
                    if (
                        self.session_sender_monotonic_origin_ns is None
                        or self.session_receiver_monotonic_origin_ns is None
                    ):
                        raise PacketError("packet session clock origin is missing")
                    sender_origin_ns = self.session_sender_monotonic_origin_ns
                    receiver_origin_ns = (
                        self.session_receiver_monotonic_origin_ns
                    )
                if self.enforce_packet_timestamps:
                    packet_age = validate_session_packet_timestamp(
                        frame.sender_monotonic_ns,
                        sender_origin_ns,
                        received_monotonic_ns,
                        receiver_origin_ns,
                        self.max_packet_age,
                        self.max_future_skew,
                    )
                else:
                    packet_age = 0.0
                if (
                    not new_session
                    and self.last_sequence is not None
                    and frame.sequence <= self.last_sequence
                ):
                    raise PacketError("sequence is not newer than the previous packet")
                if new_session:
                    self.current_session = frame.session_id
                    self.session_sender_monotonic_origin_ns = sender_origin_ns
                    self.session_receiver_monotonic_origin_ns = receiver_origin_ns
                    self.last_sequence = None
                    self.accepted_packet_times.clear()
                    self.unwrapper.reset()
                    self.signal_filter.reset()
                    self.joint_dropout_guard.reset()
                    if self.mapping_enabled:
                        self._request_stop("leader session changed")
                sanitized_pulses = frame.joint_pulses
                held_joint_indices: tuple[int, ...] = ()
                if not self.gripper_only_mode:
                    sanitized_pulses, held_joint_indices = (
                        self.joint_dropout_guard.update(
                            frame.joint_pulses,
                            received_monotonic_ns / 1_000_000_000.0,
                        )
                    )
                leader_position = self.unwrapper.update(sanitized_pulses)
                filtered_position = self.signal_filter.update(leader_position)
            except PersistentJointDropoutError as exc:
                self.rejected_packets += 1
                self.consecutive_accepted_packets = 0
                self.last_rejection_reason = str(exc).replace(";", ",")
                if self.mapping_enabled:
                    self._request_stop(str(exc))
                continue
            except (PacketError, SafetyError) as exc:
                self.rejected_packets += 1
                self.consecutive_accepted_packets = 0
                self.last_rejection_reason = str(exc).replace(";", ",")
                if self.mapping_enabled and self.stop_on_rejected_packet:
                    self._request_stop(str(exc))
                elif self.mapping_enabled:
                    # Never publish a rejected frame.  An isolated delayed,
                    # duplicated, malformed or encoder-spike packet is simply
                    # discarded; last_leader_received is deliberately not
                    # refreshed, so a sustained bad stream still reaches the
                    # normal packet watchdog and produces STOP.
                    self.get_logger().warn(
                        "Rejected leader packet while mapping is enabled; "
                        f"holding the last accepted target: {exc}"
                    )
                continue

            self.accepted_packets += 1
            self.consecutive_accepted_packets += 1
            self.last_sequence = frame.sequence
            self.last_leader_position = leader_position
            self.last_filtered_position = filtered_position
            self.last_deadman_held = frame.deadman_held
            self.last_packet_age_seconds = packet_age
            received_at = time.monotonic()
            self.last_held_joint_indices = held_joint_indices
            if held_joint_indices:
                self.held_joint_samples += 1
                if received_at - self.last_dropout_warning >= 1.0:
                    names = ",".join(
                        str(index + 1) for index in held_joint_indices
                    )
                    self.get_logger().warn(
                        "Holding last valid leader pulse for joint(s) "
                        f"{names}; other joints and gripper remain active"
                    )
                    self.last_dropout_warning = received_at
            self.last_leader_received = received_at
            self.accepted_packet_times.append(received_at)
            raw_msg = Float64MultiArray()
            raw_msg.data = [float(value) for value in leader_position]
            self.raw_pub.publish(raw_msg)
            filtered_msg = Float64MultiArray()
            filtered_msg.data = [float(value) for value in filtered_position]
            self.filtered_pub.publish(filtered_msg)

            if not self.mapping_enabled or self.mapper is None:
                if not self.mapping_enabled:
                    continue
            if self.require_deadman and not frame.deadman_held:
                # Gripper-only mode is explicitly armed before the operator
                # presses Space.  Released preview frames must not disarm it;
                # the second Space/Esc/Ctrl+C sends a dedicated STOP frame.
                if self.gripper_only_mode or self.arm_before_deadman:
                    continue
                self._request_stop("Windows deadman released")
                continue
            if self.gripper_only_mode:
                if not self.dry_run and frame.gripper_state in {"OPEN", "CLOSED"}:
                    gripper_msg = Bool()
                    gripper_msg.data = frame.gripper_state == "OPEN"
                    self.gripper_pub.publish(gripper_msg)
                continue
            assert self.mapper is not None
            try:
                target = self.mapper.map(filtered_position)
            except SafetyError as exc:
                self._request_stop(str(exc))
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
        if (
            self.stop_on_packet_timeout
            and now - self.last_leader_received > self.packet_timeout
        ):
            self._request_stop("leader packet timeout")
        elif (
            self.stop_on_follower_timeout
            and not self.gripper_only_mode
            and now - self.last_follower_received > self.follower_timeout
        ):
            self._request_stop("follower state timeout")

    def _publish_status(self) -> None:
        enabled = Bool()
        enabled.data = self.mapping_enabled
        self.enabled_pub.publish(enabled)
        status = String()
        status.data = (
            f"enabled={self.mapping_enabled};dry_run={self.dry_run};"
            f"gripper_only_mode={self.gripper_only_mode};"
            f"arm_before_deadman={self.arm_before_deadman};"
            f"calibration_complete={self.calibration_complete};"
            f"expected_source_ip={self.expected_source_ip or 'UNCONFIGURED'};"
            "timestamp_basis=sender_monotonic_vs_receiver_monotonic;"
            f"deadman_held={self.last_deadman_held};"
            f"session={self.current_session or 'none'};"
            f"sequence={self.last_sequence if self.last_sequence is not None else -1};"
            f"packet_age_s={self.last_packet_age_seconds if self.last_packet_age_seconds is not None else -1:.3f};"
            f"packet_rate_hz={self._recent_packet_rate_hz():.2f};"
            f"minimum_packet_rate_hz={self.minimum_packet_rate_hz:.2f};"
            f"stop_on_packet_timeout={self.stop_on_packet_timeout};"
            f"stop_on_follower_state_timeout={self.stop_on_follower_timeout};"
            f"stop_on_rejected_packet={self.stop_on_rejected_packet};"
            f"accepted_packets={self.accepted_packets};"
            f"consecutive_accepted_packets={self.consecutive_accepted_packets};"
            f"rejected_packets={self.rejected_packets};"
            f"held_joint_samples={self.held_joint_samples};"
            "held_joints="
            f"{','.join(str(index + 1) for index in self.last_held_joint_indices) or 'none'};"
            f"last_rejection={self.last_rejection_reason};"
            f"stop_requests={self.stop_requests}"
        )
        self.status_pub.publish(status)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UdpLeaderBridge()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
