"""Pure safety checks for one-arm SmolVLA actions.

The functions in this module are ROS-independent so they can be unit tested
without robot hardware.  Passing these checks is necessary but never replaces
the controller's joint limits, slew limiter, watchdog, or physical E-stop.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


JOINT_COUNT = 7
ACTION_DIM = 8


class PolicySafetyError(ValueError):
    """Raised when an observation or predicted action is unsafe to execute."""


@dataclass(frozen=True)
class PolicySafetyConfig:
    task_lower: tuple[float, ...]
    task_upper: tuple[float, ...]
    initial_lower: tuple[float, ...]
    initial_upper: tuple[float, ...]
    max_target_error_rad: float = 0.25
    small_envelope_overshoot_rad: float = 0.03
    gripper_open_threshold: float = 0.35
    gripper_close_threshold: float = 0.65

    def __post_init__(self) -> None:
        for name in ("task_lower", "task_upper", "initial_lower", "initial_upper"):
            values = getattr(self, name)
            if len(values) != JOINT_COUNT or not all(math.isfinite(value) for value in values):
                raise ValueError(f"{name} must contain seven finite values")
        for lower, upper in zip(self.task_lower, self.task_upper, strict=True):
            if lower >= upper:
                raise ValueError("each task lower limit must be below its upper limit")
        for lower, upper in zip(self.initial_lower, self.initial_upper, strict=True):
            if lower >= upper:
                raise ValueError("each initial lower limit must be below its upper limit")
        if not math.isfinite(self.max_target_error_rad) or self.max_target_error_rad <= 0:
            raise ValueError("max_target_error_rad must be positive")
        if self.small_envelope_overshoot_rad < 0:
            raise ValueError("small_envelope_overshoot_rad cannot be negative")
        if not 0 <= self.gripper_open_threshold < self.gripper_close_threshold <= 1:
            raise ValueError("gripper thresholds must satisfy 0 <= open < close <= 1")


def _finite_tuple(values: Sequence[float], expected: int, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != expected:
        raise PolicySafetyError(f"{name} must contain exactly {expected} values")
    if not all(math.isfinite(value) for value in result):
        raise PolicySafetyError(f"{name} contains NaN or infinity")
    return result


def validate_initial_pose(
    state: Sequence[float],
    config: PolicySafetyConfig,
    *,
    require_open_gripper: bool = True,
) -> tuple[float, ...]:
    """Validate the live 7+1 state before an autonomous rollout is armed."""

    checked = _finite_tuple(state, ACTION_DIM, "state")
    joints = checked[:JOINT_COUNT]
    outside = [
        index + 1
        for index, (value, lower, upper) in enumerate(
            zip(joints, config.initial_lower, config.initial_upper, strict=True)
        )
        if value < lower or value > upper
    ]
    if outside:
        raise PolicySafetyError(
            "initial pose is outside the demonstrated start envelope on joints "
            + ", ".join(map(str, outside))
        )
    if require_open_gripper and checked[-1] >= config.gripper_close_threshold:
        raise PolicySafetyError("initial gripper state is closed; the demonstrations start open")
    return checked


def guard_policy_action(
    predicted_action: Sequence[float],
    current_state: Sequence[float],
    previous_gripper_closed: bool,
    config: PolicySafetyConfig,
) -> tuple[tuple[float, ...], bool]:
    """Reject implausible targets and clamp only tiny task-envelope overshoot.

    The returned joint target remains an absolute target in radians.  A target
    that is far from the current measured pose is rejected rather than slowly
    chasing a potentially nonsensical model output.
    """

    action = _finite_tuple(predicted_action, ACTION_DIM, "predicted action")
    state = _finite_tuple(current_state, ACTION_DIM, "current state")
    guarded: list[float] = []
    for index, (target, current, lower, upper) in enumerate(
        zip(
            action[:JOINT_COUNT],
            state[:JOINT_COUNT],
            config.task_lower,
            config.task_upper,
            strict=True,
        )
    ):
        if target < lower - config.small_envelope_overshoot_rad or target > upper + config.small_envelope_overshoot_rad:
            raise PolicySafetyError(
                f"joint {index + 1} target {target:.4f} is outside the demonstrated task envelope"
            )
        target = min(max(target, lower), upper)
        if abs(target - current) > config.max_target_error_rad:
            raise PolicySafetyError(
                f"joint {index + 1} target error {target - current:+.4f} rad exceeds "
                f"{config.max_target_error_rad:.4f} rad"
            )
        guarded.append(target)

    gripper_value = action[-1]
    if gripper_value <= config.gripper_open_threshold:
        gripper_closed = False
    elif gripper_value >= config.gripper_close_threshold:
        gripper_closed = True
    else:
        gripper_closed = bool(previous_gripper_closed)
    return tuple(guarded), gripper_closed
