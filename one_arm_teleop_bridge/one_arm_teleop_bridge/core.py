"""Pure-Python protocol, unwrap and offset-absolute mapping logic."""

from __future__ import annotations

import json
import math
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Any, Union


PROTOCOL = "one_arm_teleop"
PROTOCOL_VERSION = 1
JOINT_COUNT = 7
EXPECTED_JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 8))


class PacketError(ValueError):
    pass


class SafetyError(ValueError):
    pass


@dataclass(frozen=True)
class LeaderFrame:
    session_id: str
    sequence: int
    timestamp_unix_ns: int
    joint_names: tuple[str, ...]
    joint_pulses: tuple[int, ...]
    gripper_state: str
    gripper_raw_pulse: int
    gripper_unwrapped_pulse: int
    deadman_held: bool


@dataclass(frozen=True)
class StopFrame:
    session_id: str
    sequence: int
    timestamp_unix_ns: int
    reason: str


TeleopPacket = Union[LeaderFrame, StopFrame]


def _decode_packet(payload: bytes) -> dict[str, Any]:
    try:
        raw = json.loads(payload.decode("ascii"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PacketError(f"packet is not valid ASCII JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise PacketError("packet root must be an object")
    if raw.get("protocol") != PROTOCOL or raw.get("version") != PROTOCOL_VERSION:
        raise PacketError("protocol name or version does not match")
    try:
        session_id = str(raw["session_id"])
        sequence = int(raw["sequence"])
        timestamp_unix_ns = int(raw["timestamp_unix_ns"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PacketError(f"packet field is missing or invalid: {exc}") from exc
    if not session_id or sequence < 0 or timestamp_unix_ns <= 0:
        raise PacketError("session, sequence or timestamp is invalid")
    return raw


def parse_teleop_packet(payload: bytes) -> TeleopPacket:
    raw = _decode_packet(payload)
    session_id = str(raw["session_id"])
    sequence = int(raw["sequence"])
    timestamp_unix_ns = int(raw["timestamp_unix_ns"])
    message_type = raw.get("message_type", "leader_frame")

    if message_type == "stop":
        reason = str(raw.get("reason", "remote_stop")).strip()
        if not reason or len(reason) > 160:
            raise PacketError("STOP reason must contain 1 to 160 characters")
        return StopFrame(
            session_id=session_id,
            sequence=sequence,
            timestamp_unix_ns=timestamp_unix_ns,
            reason=reason,
        )
    if message_type != "leader_frame":
        raise PacketError(f"unsupported message_type: {message_type}")
    if raw.get("complete") is not True:
        raise PacketError("incomplete leader frames are not accepted")

    try:
        joint_names = tuple(str(value) for value in raw["joint_names"])
        joint_pulses = tuple(int(value) for value in raw["joint_pulses"])
        gripper = raw["gripper"]
        gripper_state = str(gripper["state"])
        gripper_raw = int(gripper["raw_pulse"])
        gripper_unwrapped = int(gripper["unwrapped_pulse"])
        deadman_raw = raw.get("deadman_held", False)
    except (KeyError, TypeError, ValueError) as exc:
        raise PacketError(f"packet field is missing or invalid: {exc}") from exc

    if not isinstance(deadman_raw, bool):
        raise PacketError("deadman_held must be a JSON boolean")
    if joint_names != EXPECTED_JOINT_NAMES:
        raise PacketError(
            f"joint_names must be {EXPECTED_JOINT_NAMES}, got {joint_names}"
        )
    if len(joint_pulses) != JOINT_COUNT:
        raise PacketError("exactly seven leader joint pulses are required")
    if any(pulse < 0 for pulse in joint_pulses):
        raise PacketError("leader pulses cannot be negative")
    if gripper_state not in {"OPEN", "CLOSED", "UNKNOWN"}:
        raise PacketError("gripper state is invalid")

    return LeaderFrame(
        session_id=session_id,
        sequence=sequence,
        timestamp_unix_ns=timestamp_unix_ns,
        joint_names=joint_names,
        joint_pulses=joint_pulses,
        gripper_state=gripper_state,
        gripper_raw_pulse=gripper_raw,
        gripper_unwrapped_pulse=gripper_unwrapped,
        deadman_held=deadman_raw,
    )


def parse_leader_packet(payload: bytes) -> LeaderFrame:
    """Backward-compatible leader-only parser used by existing callers/tests."""
    packet = parse_teleop_packet(payload)
    if isinstance(packet, StopFrame):
        raise PacketError("STOP packet is not a leader frame")
    return packet


def validate_packet_timestamp(
    timestamp_unix_ns: int,
    now_unix_ns: int,
    max_age_seconds: float,
    max_future_skew_seconds: float,
) -> float:
    """Return packet age in seconds, rejecting stale or future-dated packets."""
    if timestamp_unix_ns <= 0 or now_unix_ns <= 0:
        raise PacketError("packet and local timestamps must be positive")
    if max_age_seconds <= 0.0 or max_future_skew_seconds < 0.0:
        raise ValueError("timestamp limits are invalid")
    age_seconds = (now_unix_ns - timestamp_unix_ns) / 1_000_000_000.0
    if age_seconds > max_age_seconds:
        raise PacketError(
            f"packet is stale ({age_seconds:.3f}s > {max_age_seconds:.3f}s)"
        )
    if age_seconds < -max_future_skew_seconds:
        raise PacketError(
            "packet timestamp is too far in the future "
            f"({-age_seconds:.3f}s > {max_future_skew_seconds:.3f}s)"
        )
    return age_seconds


def validate_session_packet_timestamp(
    sender_timestamp_ns: int,
    sender_origin_ns: int,
    receiver_monotonic_ns: int,
    receiver_origin_monotonic_ns: int,
    max_age_seconds: float,
    max_future_skew_seconds: float,
) -> float:
    """Validate packet delay without requiring synchronized host clocks.

    The first accepted packet in a sender session establishes both origins.
    Subsequent sender elapsed time is compared with receiver monotonic elapsed
    time.  This retains stale/future packet detection while tolerating a fixed
    wall-clock offset between Windows and Ubuntu.
    """
    if min(
        sender_timestamp_ns,
        sender_origin_ns,
        receiver_monotonic_ns,
        receiver_origin_monotonic_ns,
    ) <= 0:
        raise PacketError("packet session timestamps must be positive")
    if max_age_seconds <= 0.0 or max_future_skew_seconds < 0.0:
        raise ValueError("timestamp limits are invalid")
    sender_elapsed_ns = sender_timestamp_ns - sender_origin_ns
    receiver_elapsed_ns = (
        receiver_monotonic_ns - receiver_origin_monotonic_ns
    )
    if sender_elapsed_ns < 0:
        raise PacketError("packet timestamp moved backwards within session")
    if receiver_elapsed_ns < 0:
        raise PacketError("receiver monotonic timestamp moved backwards")
    age_seconds = (
        receiver_elapsed_ns - sender_elapsed_ns
    ) / 1_000_000_000.0
    if age_seconds > max_age_seconds:
        raise PacketError(
            f"packet is stale ({age_seconds:.3f}s > {max_age_seconds:.3f}s)"
        )
    if age_seconds < -max_future_skew_seconds:
        raise PacketError(
            "packet timestamp advanced too far within session "
            f"({-age_seconds:.3f}s > {max_future_skew_seconds:.3f}s)"
        )
    return age_seconds


class MultiJointUnwrapper:
    def __init__(
        self,
        period_pulses: int,
        max_step_pulses: float,
        joint_count: int = JOINT_COUNT,
    ):
        if period_pulses <= 0 or max_step_pulses <= 0:
            raise ValueError("unwrap period and maximum step must be positive")
        self.period = period_pulses
        self.max_step = max_step_pulses
        self.previous: list[float] | None = None
        self.joint_count = joint_count

    def reset(self) -> None:
        self.previous = None

    def update(self, raw_pulses: tuple[int, ...]) -> tuple[float, ...]:
        if len(raw_pulses) != self.joint_count:
            raise SafetyError("unexpected joint count during unwrap")
        if self.previous is None:
            self.previous = [float(value) for value in raw_pulses]
            return tuple(self.previous)

        result: list[float] = []
        for index, (raw, previous) in enumerate(zip(raw_pulses, self.previous)):
            turns = round((previous - raw) / self.period)
            candidate = float(raw + turns * self.period)
            step = candidate - previous
            if abs(step) > self.max_step:
                raise SafetyError(
                    f"leader joint {index + 1} jumped {step:.1f} pulses"
                )
            result.append(candidate)
        self.previous = result
        return tuple(result)


class LeaderSignalFilter:
    """Median spike rejection, optional low-pass, then output deadband."""

    def __init__(
        self,
        median_window: int = 1,
        low_pass_alpha: float = 1.0,
        deadband_pulses: float = 0.0,
        joint_count: int = JOINT_COUNT,
    ):
        if median_window < 1 or median_window > 31 or median_window % 2 == 0:
            raise ValueError("median_window must be an odd integer from 1 to 31")
        if not 0.0 < low_pass_alpha <= 1.0:
            raise ValueError("low_pass_alpha must be in (0, 1]")
        if deadband_pulses < 0.0 or not math.isfinite(deadband_pulses):
            raise ValueError("deadband_pulses must be finite and non-negative")
        self.median_window = median_window
        self.alpha = low_pass_alpha
        self.deadband = deadband_pulses
        self.joint_count = joint_count
        self.history = [
            deque(maxlen=median_window) for _ in range(self.joint_count)
        ]
        self.low_pass_state: list[float] | None = None
        self.output: list[float] | None = None

    def reset(self) -> None:
        for values in self.history:
            values.clear()
        self.low_pass_state = None
        self.output = None

    def update(self, positions: tuple[float, ...]) -> tuple[float, ...]:
        if len(positions) != self.joint_count:
            raise SafetyError("unexpected joint count during filtering")
        if any(not math.isfinite(value) for value in positions):
            raise SafetyError("leader filter input contains a non-finite value")

        medians: list[float] = []
        for history, value in zip(self.history, positions):
            history.append(float(value))
            medians.append(float(statistics.median(history)))

        if self.low_pass_state is None:
            self.low_pass_state = medians
        else:
            self.low_pass_state = [
                previous + self.alpha * (current - previous)
                for previous, current in zip(self.low_pass_state, medians)
            ]

        if self.output is None:
            self.output = list(self.low_pass_state)
        else:
            self.output = [
                previous if abs(current - previous) <= self.deadband else current
                for previous, current in zip(self.output, self.low_pass_state)
            ]
        return tuple(self.output)


@dataclass(frozen=True)
class MappingConfig:
    signs: tuple[float, ...]
    scale_rad_per_pulse: tuple[float, ...]
    lower_limits: tuple[float, ...]
    upper_limits: tuple[float, ...]

    def validate(self) -> None:
        arrays = (
            self.signs,
            self.scale_rad_per_pulse,
            self.lower_limits,
            self.upper_limits,
        )
        if any(len(values) != JOINT_COUNT for values in arrays):
            raise SafetyError("all mapping and limit arrays must contain seven values")
        if any(sign not in (-1.0, 1.0) for sign in self.signs):
            raise SafetyError("every sign must be +1 or -1")
        if any(
            not math.isfinite(scale) or scale <= 0.0
            for scale in self.scale_rad_per_pulse
        ):
            raise SafetyError("every scale_rad_per_pulse must be finite and positive")
        for index, (lower, upper) in enumerate(
            zip(self.lower_limits, self.upper_limits)
        ):
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise SafetyError(f"joint {index + 1} limits are invalid")


class OffsetAbsoluteMapper:
    """q_target = q0 + sign * scale * (leader - leader0)."""

    def __init__(self, config: MappingConfig):
        config.validate()
        self.config = config
        self.leader_zero: tuple[float, ...] | None = None
        self.follower_zero: tuple[float, ...] | None = None

    def set_reference(
        self,
        leader_position: tuple[float, ...],
        follower_position: tuple[float, ...],
    ) -> None:
        if len(leader_position) != JOINT_COUNT or len(follower_position) != JOINT_COUNT:
            raise SafetyError("reference positions must contain seven values")
        if any(not math.isfinite(value) for value in follower_position):
            raise SafetyError("follower reference contains a non-finite value")
        self.leader_zero = tuple(leader_position)
        self.follower_zero = tuple(follower_position)

    def map(self, leader_position: tuple[float, ...]) -> tuple[float, ...]:
        if self.leader_zero is None or self.follower_zero is None:
            raise SafetyError("offset reference has not been captured")
        target = tuple(
            q0 + sign * scale * (leader - p0)
            for q0, sign, scale, leader, p0 in zip(
                self.follower_zero,
                self.config.signs,
                self.config.scale_rad_per_pulse,
                leader_position,
                self.leader_zero,
            )
        )
        for index, (value, lower, upper) in enumerate(
            zip(target, self.config.lower_limits, self.config.upper_limits)
        ):
            if not math.isfinite(value):
                raise SafetyError(f"joint {index + 1} target is not finite")
            if value < lower or value > upper:
                raise SafetyError(
                    f"joint {index + 1} target {value:.4f} is outside "
                    f"[{lower:.4f}, {upper:.4f}]"
                )
        return target
