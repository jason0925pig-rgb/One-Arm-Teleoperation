from __future__ import annotations

import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from lerobot.robots.robot import Robot

from one_arm_teleop_bridge.smolvla_guard import (
    PolicySafetyConfig,
    PolicySafetyError,
    guard_policy_action,
    validate_initial_pose,
)

from .configuration_armstrong_ros2 import ArmstrongRos2Config


JOINT_NAMES = tuple(f"right_joint{index}" for index in range(1, 8))
GRIPPER_NAME = "right_gripper_closed"


def ros_requested_open_to_lerobot_closed(requested_open: bool) -> bool:
    return not bool(requested_open)


def lerobot_closed_to_ros_requested_open(closed: bool) -> bool:
    return not bool(closed)


class ArmstrongRos2(Robot):
    """LeRobot hardware facade over the existing safe ROS 2 controllers.

    Connecting this adapter is observation-only.  ``send_action`` publishes
    nothing until ``/smolvla/set_enabled`` succeeds.  The underlying arm node
    retains its independent servo-mode gate, limits, slew limiter and watchdog.
    """

    config_class = ArmstrongRos2Config
    name = "armstrong_ros2"

    def __init__(self, config: ArmstrongRos2Config):
        super().__init__(config)
        self.config = config
        self._connected = False
        self._action_enabled = False
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._joint_state: tuple[float, ...] | None = None
        self._joint_time = 0.0
        self._chest: np.ndarray | None = None
        self._chest_time = 0.0
        self._wrist: np.ndarray | None = None
        self._wrist_time = 0.0
        self._gripper_closed = False
        self._last_gripper_command: bool | None = None
        self._last_safe_joint_command: tuple[float, ...] | None = None
        self._policy_queue_size = 0
        self._policy_expected_chunk_size = 0
        self._policy_chunk_ready = False
        self._owns_rclpy_context = False
        self._guard_config = PolicySafetyConfig(
            task_lower=tuple(config.task_lower),
            task_upper=tuple(config.task_upper),
            initial_lower=tuple(config.initial_lower),
            initial_upper=tuple(config.initial_upper),
            max_target_error_rad=config.max_target_error_rad,
            small_envelope_overshoot_rad=config.small_envelope_overshoot_rad,
            gripper_open_threshold=config.gripper_open_threshold,
            gripper_close_threshold=config.gripper_close_threshold,
        )

    @property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        return {
            **{name: float for name in JOINT_NAMES},
            GRIPPER_NAME: float,
            "chest": (self.config.image_height, self.config.image_width, 3),
            "wrist_right": (self.config.image_height, self.config.image_width, 3),
        }

    @property
    def action_features(self) -> dict[str, type]:
        return {
            **{name: float for name in JOINT_NAMES},
            GRIPPER_NAME: float,
        }

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    @property
    def action_enabled(self) -> bool:
        return self._action_enabled

    def update_policy_queue_state(
        self,
        queue_size: int,
        expected_chunk_size: int,
        chunk_ready: bool,
    ) -> None:
        """Expose async-policy preload state through ``/smolvla/status``."""
        with self._lock:
            self._policy_queue_size = max(0, int(queue_size))
            self._policy_expected_chunk_size = max(0, int(expected_chunk_size))
            self._policy_chunk_ready = bool(chunk_ready)

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        if self._connected:
            return
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True
        self._node = Node("armstrong_lerobot_client")
        self._joint_pub = self._node.create_publisher(
            JointState, self.config.joint_command_topic, 10
        )
        self._gripper_pub = self._node.create_publisher(
            Bool, self.config.gripper_command_topic, 10
        )
        self._stop_pub = self._node.create_publisher(Bool, self.config.stop_topic, 10)
        self._status_pub = self._node.create_publisher(String, "/smolvla/status", 10)
        self._node.create_subscription(
            JointState, self.config.joint_state_topic, self._joint_callback, 10
        )
        self._node.create_subscription(
            Bool, self.config.gripper_state_topic, self._gripper_callback, 10
        )
        self._node.create_subscription(
            CompressedImage,
            self.config.chest_topic,
            self._chest_callback,
            qos_profile_sensor_data,
        )
        self._node.create_subscription(
            CompressedImage,
            self.config.wrist_topic,
            self._wrist_callback,
            qos_profile_sensor_data,
        )
        self._node.create_service(SetBool, self.config.enable_service, self._set_enabled)
        self._node.create_timer(0.5, self._publish_status)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._connected = True
        if self.config.start_action_enabled:
            raise RuntimeError(
                "start_action_enabled=true is intentionally unsupported; arm through the ROS service"
            )
        self._node.get_logger().warn(
            "Armstrong LeRobot adapter connected in observation-only mode; no action is published"
        )

    def _decode_image(self, message: CompressedImage) -> np.ndarray:
        encoded = np.frombuffer(bytes(message.data), dtype=np.uint8)
        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("JPEG image could not be decoded")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if rgb.shape[:2] != (self.config.image_height, self.config.image_width):
            rgb = cv2.resize(
                rgb,
                (self.config.image_width, self.config.image_height),
                interpolation=cv2.INTER_AREA,
            )
        return np.ascontiguousarray(rgb, dtype=np.uint8)

    def _joint_callback(self, message: JointState) -> None:
        if len(message.position) != 7:
            return
        if message.name:
            lookup = dict(zip(message.name, message.position, strict=False))
            if any(name not in lookup for name in JOINT_NAMES):
                return
            ordered = tuple(float(lookup[name]) for name in JOINT_NAMES)
        else:
            ordered = tuple(float(value) for value in message.position)
        with self._lock:
            self._joint_state = ordered
            self._joint_time = time.monotonic()

    def _gripper_callback(self, message: Bool) -> None:
        # The safe gripper controller Bool means ``requested_open``. LeRobot
        # uses the opposite convention for this feature: 0=open, 1=closed.
        with self._lock:
            self._gripper_closed = ros_requested_open_to_lerobot_closed(
                message.data
            )

    def _chest_callback(self, message: CompressedImage) -> None:
        try:
            decoded = self._decode_image(message)
        except Exception as exc:
            if self._node is not None:
                self._node.get_logger().error(f"chest image decode failed: {exc}")
            return
        with self._lock:
            self._chest = decoded
            self._chest_time = time.monotonic()

    def _wrist_callback(self, message: CompressedImage) -> None:
        try:
            decoded = self._decode_image(message)
        except Exception as exc:
            if self._node is not None:
                self._node.get_logger().error(f"wrist image decode failed: {exc}")
            return
        with self._lock:
            self._wrist = decoded
            self._wrist_time = time.monotonic()

    def _snapshot(self) -> tuple[tuple[float, ...], bool, np.ndarray, np.ndarray]:
        now = time.monotonic()
        with self._lock:
            joints = self._joint_state
            gripper_closed = self._gripper_closed
            chest = None if self._chest is None else self._chest.copy()
            wrist = None if self._wrist is None else self._wrist.copy()
            joint_age = now - self._joint_time
            chest_age = now - self._chest_time
            wrist_age = now - self._wrist_time
        if joints is None or joint_age > self.config.state_timeout_seconds:
            raise RuntimeError(f"joint state is missing or stale ({joint_age:.3f}s)")
        if chest is None or chest_age > self.config.camera_timeout_seconds:
            raise RuntimeError(f"chest image is missing or stale ({chest_age:.3f}s)")
        if wrist is None or wrist_age > self.config.camera_timeout_seconds:
            raise RuntimeError(f"wrist image is missing or stale ({wrist_age:.3f}s)")
        return joints, gripper_closed, chest, wrist

    def get_observation(self) -> dict[str, Any]:
        if not self._connected:
            raise RuntimeError("robot adapter is not connected")
        joints, gripper_closed, chest, wrist = self._snapshot()
        return {
            **dict(zip(JOINT_NAMES, joints, strict=True)),
            GRIPPER_NAME: float(gripper_closed),
            "chest": chest,
            "wrist_right": wrist,
        }

    def _set_enabled(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if not request.data:
            self._action_enabled = False
            self._last_safe_joint_command = None
            self._last_gripper_command = None
            self._publish_stop()
            response.success = True
            response.message = "SmolVLA action gate disabled and STOP published"
            return response
        try:
            joints, gripper_closed, _, _ = self._snapshot()
            state = (*joints, float(gripper_closed))
            validate_initial_pose(
                state,
                self._guard_config,
                require_open_gripper=self.config.require_open_gripper_at_start,
            )
            if self._joint_pub.get_subscription_count() != 1:
                raise PolicySafetyError(
                    "expected exactly one safe arm-command subscriber"
                )
            if self._gripper_pub.get_subscription_count() != 1:
                raise PolicySafetyError(
                    "expected exactly one safe gripper-command subscriber"
                )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        self._action_enabled = True
        response.success = True
        response.message = "SmolVLA action gate enabled"
        return response

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self._connected:
            raise RuntimeError("robot adapter is not connected")
        if not self._action_enabled:
            return action
        joints, gripper_closed, _, _ = self._snapshot()
        predicted = tuple(float(action[name]) for name in (*JOINT_NAMES, GRIPPER_NAME))
        try:
            guarded_joints, guarded_gripper = guard_policy_action(
                predicted,
                (*joints, float(gripper_closed)),
                gripper_closed,
                self._guard_config,
            )
        except PolicySafetyError:
            self._action_enabled = False
            self._last_safe_joint_command = None
            self._last_gripper_command = None
            self._publish_stop()
            raise

        self._last_safe_joint_command = tuple(guarded_joints)
        self._publish_joint_command(self._last_safe_joint_command)
        if self._last_gripper_command is None or guarded_gripper != self._last_gripper_command:
            gripper_message = Bool()
            # ROS command=True requests OPEN; guarded_gripper=True means
            # CLOSED in the LeRobot action space, so the value must invert.
            gripper_message.data = lerobot_closed_to_ros_requested_open(guarded_gripper)
            self._gripper_pub.publish(gripper_message)
            self._last_gripper_command = guarded_gripper
        return {
            **dict(zip(JOINT_NAMES, guarded_joints, strict=True)),
            GRIPPER_NAME: float(guarded_gripper),
        }

    def _publish_joint_command(self, joints: tuple[float, ...]) -> None:
        message = JointState()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(joints)
        self._joint_pub.publish(message)

    def resend_last_joint_command(self) -> bool:
        """Keep the last guarded target alive while the next chunk is inferred."""
        if not self._connected or not self._action_enabled:
            return False
        command = self._last_safe_joint_command
        if command is None:
            return False
        self._publish_joint_command(command)
        return True

    def _publish_stop(self) -> None:
        if self._node is None:
            return
        message = Bool()
        message.data = True
        for _ in range(3):
            self._stop_pub.publish(message)

    def _publish_status(self) -> None:
        if self._node is None:
            return
        now = time.monotonic()
        with self._lock:
            joint_present = self._joint_state is not None
            chest_present = self._chest is not None
            wrist_present = self._wrist is not None
            joint_age = now - self._joint_time if joint_present else -1.0
            chest_age = now - self._chest_time if chest_present else -1.0
            wrist_age = now - self._wrist_time if wrist_present else -1.0
            policy_queue_size = self._policy_queue_size
            policy_expected_chunk_size = self._policy_expected_chunk_size
            policy_chunk_ready = self._policy_chunk_ready
        observation_ready = (
            joint_present
            and chest_present
            and wrist_present
            and joint_age <= self.config.state_timeout_seconds
            and chest_age <= self.config.camera_timeout_seconds
            and wrist_age <= self.config.camera_timeout_seconds
        )
        message = String()
        message.data = (
            f"connected={int(self._connected)};"
            f"action_enabled={int(self._action_enabled)};"
            f"observation_ready={int(observation_ready)};"
            f"policy_chunk_ready={int(policy_chunk_ready)};"
            f"action_queue_size={policy_queue_size};"
            f"expected_action_chunk_size={policy_expected_chunk_size};"
            f"joint_age_s={joint_age:.3f};"
            f"chest_age_s={chest_age:.3f};"
            f"wrist_age_s={wrist_age:.3f}"
        )
        self._status_pub.publish(message)

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._action_enabled = False
        self._last_safe_joint_command = None
        self._last_gripper_command = None
        self._publish_stop()
        self._connected = False
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._owns_rclpy_context and rclpy.ok():
            rclpy.shutdown()
