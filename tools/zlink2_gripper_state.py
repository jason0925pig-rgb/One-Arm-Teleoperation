#!/usr/bin/env python3
"""Calibrated, stateful interpretation of the ZLink2 leader gripper encoder.

The encoder wraps at a configured pulse period, so a raw value near 797 can
represent the calibrated open position 3297.  This module unwraps consecutive
samples and applies an OPEN/CLOSED hysteresis state machine.  It never sends a
serial or motion command.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = PROJECT_ROOT / "gripper_calibration.json"
VALID_STATES = {"UNKNOWN", "OPEN", "CLOSED"}


@dataclass(frozen=True)
class GripperCalibration:
    period_pulses: int
    closed_reference: int
    open_reference: int
    close_threshold: int
    open_threshold: int
    debounce_frames: int

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "GripperCalibration":
        try:
            encoder = raw["encoder"]
            references = raw["references"]
            state_machine = raw["state_machine"]
            calibration = cls(
                period_pulses=int(encoder["period_pulses"]),
                closed_reference=int(references["closed_unwrapped_pulse"]),
                open_reference=int(references["open_unwrapped_pulse"]),
                close_threshold=int(
                    state_machine["close_at_or_below_unwrapped_pulse"]
                ),
                open_threshold=int(
                    state_machine["open_at_or_above_unwrapped_pulse"]
                ),
                debounce_frames=int(state_machine["debounce_frames"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid gripper calibration structure: {exc}") from exc
        calibration.validate()
        return calibration

    def validate(self) -> None:
        if self.period_pulses <= 0:
            raise ValueError("gripper period_pulses must be positive")
        if self.closed_reference >= self.open_reference:
            raise ValueError(
                "gripper open reference must be above the closed reference "
                "in the unwrapped coordinate"
            )
        if not (
            self.closed_reference
            < self.close_threshold
            < self.open_threshold
            < self.open_reference
        ):
            raise ValueError(
                "expected closed_reference < close_threshold < "
                "open_threshold < open_reference"
            )
        if self.debounce_frames < 1:
            raise ValueError("gripper debounce_frames must be at least 1")


@dataclass(frozen=True)
class GripperObservation:
    raw_pulse: int
    unwrapped_pulse: int
    normalized: float
    state: str
    state_changed: bool


def load_calibration(
    path: Path = DEFAULT_CONFIG,
) -> tuple[GripperCalibration, dict[str, Any]]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"gripper calibration file does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"gripper calibration file is not valid JSON: {path}: {exc}"
        ) from exc
    if not isinstance(raw, dict):
        raise ValueError("gripper calibration root must be a JSON object")
    return GripperCalibration.from_dict(raw), raw


class GripperStateMachine:
    """Unwrap circular samples and emit a debounced binary gripper state."""

    def __init__(self, calibration: GripperCalibration):
        self.calibration = calibration
        self.previous_unwrapped: int | None = None
        self.state = "UNKNOWN"
        self.pending_state: str | None = None
        self.pending_frames = 0

    def _candidate_range(self, raw_pulse: int) -> range:
        low = self.calibration.closed_reference - self.calibration.period_pulses
        high = self.calibration.open_reference + self.calibration.period_pulses
        first = math.floor((low - raw_pulse) / self.calibration.period_pulses)
        last = math.ceil((high - raw_pulse) / self.calibration.period_pulses)
        return range(first, last + 1)

    def _unwrap(self, raw_pulse: int) -> int:
        period = self.calibration.period_pulses
        candidates = [
            raw_pulse + turns * period
            for turns in self._candidate_range(raw_pulse)
        ]
        if self.previous_unwrapped is not None:
            return min(
                candidates,
                key=lambda value: (abs(value - self.previous_unwrapped), value),
            )

        references = (
            self.calibration.closed_reference,
            self.calibration.open_reference,
        )
        return min(
            candidates,
            key=lambda value: (
                min(abs(value - reference) for reference in references),
                abs(value - sum(references) / 2.0),
            ),
        )

    def _threshold_state(self, unwrapped_pulse: int) -> str | None:
        if unwrapped_pulse <= self.calibration.close_threshold:
            return "CLOSED"
        if unwrapped_pulse >= self.calibration.open_threshold:
            return "OPEN"
        return None

    def update(self, raw_pulse: int) -> GripperObservation:
        if not 0 <= raw_pulse <= self.calibration.period_pulses:
            raise ValueError(
                f"raw gripper pulse {raw_pulse} is outside "
                f"0..{self.calibration.period_pulses}"
            )

        unwrapped = self._unwrap(raw_pulse)
        self.previous_unwrapped = unwrapped
        threshold_state = self._threshold_state(unwrapped)
        changed = False

        if self.state == "UNKNOWN":
            # A calibrated endpoint is unambiguous at startup.  Debouncing is
            # applied to later transitions, where accidental threshold crossing
            # is the concern.
            if threshold_state is not None:
                self.state = threshold_state
        elif threshold_state is None or threshold_state == self.state:
            self.pending_state = None
            self.pending_frames = 0
        else:
            if self.pending_state == threshold_state:
                self.pending_frames += 1
            else:
                self.pending_state = threshold_state
                self.pending_frames = 1
            if self.pending_frames >= self.calibration.debounce_frames:
                self.state = threshold_state
                self.pending_state = None
                self.pending_frames = 0
                changed = True

        span = (
            self.calibration.open_reference
            - self.calibration.closed_reference
        )
        normalized = (
            unwrapped - self.calibration.closed_reference
        ) / span
        normalized = min(1.0, max(0.0, normalized))
        return GripperObservation(
            raw_pulse=raw_pulse,
            unwrapped_pulse=unwrapped,
            normalized=normalized,
            state=self.state,
            state_changed=changed,
        )


def replay_csv(csv_path: Path, calibration: GripperCalibration) -> dict[str, Any]:
    machine = GripperStateMachine(calibration)
    frames = 0
    transitions: list[dict[str, Any]] = []
    final_state = "UNKNOWN"
    with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {"gripper_pulse", "elapsed_s"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(
                f"CSV needs columns {sorted(required)}; got {reader.fieldnames}"
            )
        for row in reader:
            pulse_text = row["gripper_pulse"].strip()
            if not pulse_text:
                continue
            observation = machine.update(int(pulse_text))
            if observation.state_changed:
                transitions.append(
                    {
                        "elapsed_s": float(row["elapsed_s"]),
                        "state": observation.state,
                        "raw_pulse": observation.raw_pulse,
                        "unwrapped_pulse": observation.unwrapped_pulse,
                    }
                )
            final_state = observation.state
            frames += 1
    return {
        "frames_with_gripper": frames,
        "transitions": transitions,
        "transition_count": len(transitions),
        "final_state": final_state,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay a ZLink2 recording through the gripper state machine"
    )
    parser.add_argument("csv", type=Path, help="recorded frames.csv")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()

    config_path = args.config.expanduser().resolve()
    csv_path = args.csv.expanduser().resolve()
    calibration, _ = load_calibration(config_path)
    result = replay_csv(csv_path, calibration)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
