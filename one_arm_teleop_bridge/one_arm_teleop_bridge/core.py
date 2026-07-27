"""Pure-Python protocol, unwrap and offset-absolute mapping logic."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any


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


def parse_leader_packet(payload: bytes) -> LeaderFrame:
    try:
        raw = json.loads(payload.decode("ascii"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PacketError(f"packet is not valid ASCII JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise PacketError("packet root must be an object")
    if raw.get("protocol") != PROTOCOL or raw.get("version") != PROTOCOL_VERSION:
        raise PacketError("protocol name or version does not match")
    if raw.get("complete") is not True:
        raise PacketError("incomplete leader frames are not accepted")

    try:
        session_id = str(raw["session_id"])
        sequence = int(raw["sequence"])
        timestamp_unix_ns = int(raw["timestamp_unix_ns"])
        joint_names = tuple(str(value) for value in raw["joint_names"])
        joint_pulses = tuple(int(value) for value in raw["joint_pulses"])
        gripper = raw["gripper"]
        gripper_state = str(gripper["state"])
        gripper_raw = int(gripper["raw_pulse"])
        gripper_unwrapped = int(gripper["unwrapped_pulse"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PacketError(f"packet field is missing or invalid: {exc}") from exc

    if not session_id or sequence < 0 or timestamp_unix_ns <= 0:
        raise PacketError("session, sequence or timestamp is invalid")
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
    )


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
