#!/usr/bin/env python3
"""Read and identify eight ZLink2/ZServo encoder channels on Windows.

Normal operation is read-only: the program only sends PRAD position queries.
The optional ``--release-torque`` action sends PULK, but only after an
interactive safety confirmation.  This file contains no position command,
ID-write command, calibration command, or reset command.
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - depends on local installation
    raise SystemExit(
        "pyserial is required. Run with PlatformIO's Python:\n"
        r"  %USERPROFILE%\.platformio\penv\Scripts\python.exe "
        r"tools\zlink2_encoder_mapper.py"
    ) from exc


POSITION_PATTERN = re.compile(rb"#(?P<id>\d{3})P(?P<pulse>\d{4})!")
DEFAULT_IDS = tuple(range(8))
DEFAULT_LABELS = tuple([f"joint_{index}" for index in range(1, 8)] + ["gripper"])
POSITION_QUERY = "PRAD"
TORQUE_RELEASE = "PULK"


@dataclass(frozen=True)
class PositionSample:
    monotonic_time: float
    pulses: dict[int, int]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def pulse_to_degrees(pulse: int) -> float:
    """Provisional repository mapping: 500..2500 pulse equals 0..270 degrees."""
    return (pulse - 500) / 2000.0 * 270.0


def parse_ids(text: str) -> tuple[int, ...]:
    ids: set[int] = set()
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if "-" in item:
            start_text, end_text = item.split("-", 1)
            start, end = int(start_text), int(end_text)
            if start > end:
                raise argparse.ArgumentTypeError(f"descending ID range is invalid: {item}")
            ids.update(range(start, end + 1))
        else:
            ids.add(int(item))
    if not ids or min(ids) < 0 or max(ids) > 999:
        raise argparse.ArgumentTypeError("IDs must be in the range 0..999")
    return tuple(sorted(ids))


def parse_labels(text: str) -> tuple[str, ...]:
    labels = tuple(item.strip() for item in text.split(",") if item.strip())
    if not labels:
        raise argparse.ArgumentTypeError("at least one label is required")
    if len(set(labels)) != len(labels):
        raise argparse.ArgumentTypeError("labels must be unique")
    return labels


def build_command(servo_id: int, operation: str) -> bytes:
    if operation not in (POSITION_QUERY, TORQUE_RELEASE):
        raise ValueError(f"unsupported operation: {operation}")
    return f"#{servo_id:03d}{operation}!".encode("ascii")


def open_port(port_name: str, baudrate: int) -> serial.Serial:
    port = serial.Serial()
    port.port = port_name
    port.baudrate = baudrate
    port.timeout = 0.002
    port.write_timeout = 0.25
    port.dtr = False
    port.rts = False
    port.open()
    time.sleep(0.1)
    port.reset_input_buffer()
    port.reset_output_buffer()
    return port


def exchange(
    port: serial.Serial,
    command: bytes,
    total_timeout: float,
    quiet_time: float = 0.003,
) -> bytes:
    """Send one command and return bytes received after the line becomes quiet."""
    port.reset_input_buffer()
    port.write(command)
    port.flush()

    reply = bytearray()
    deadline = time.monotonic() + total_timeout
    last_byte_at: float | None = None
    while time.monotonic() < deadline:
        waiting = port.in_waiting
        chunk = port.read(waiting if waiting else 1)
        if chunk:
            reply.extend(chunk)
            last_byte_at = time.monotonic()
        elif last_byte_at is not None and time.monotonic() - last_byte_at >= quiet_time:
            break
    return bytes(reply)


def read_position(
    port: serial.Serial,
    servo_id: int,
    timeout: float,
    retries: int = 1,
) -> int | None:
    """Read one PRAD response and return as soon as a complete frame arrives.

    Waiting for a generic "quiet line" after every reply is unnecessarily slow
    on Windows because short serial timeouts may be rounded up by the driver.
    PRAD has a fixed, self-terminating ``#IDPdddd!`` response, so detecting that
    complete frame is sufficient and substantially improves eight-servo scan
    rate.
    """
    command = build_command(servo_id, POSITION_QUERY)
    for _ in range(retries + 1):
        port.reset_input_buffer()
        port.write(command)
        port.flush()

        reply = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            waiting = port.in_waiting
            chunk = port.read(waiting if waiting else 1)
            if not chunk:
                continue
            reply.extend(chunk)
            for match in POSITION_PATTERN.finditer(reply):
                if int(match.group("id")) == servo_id:
                    return int(match.group("pulse"))
    return None


def scan_positions(
    port: serial.Serial,
    ids: Iterable[int],
    timeout: float,
) -> PositionSample:
    pulses: dict[int, int] = {}
    for servo_id in ids:
        pulse = read_position(port, servo_id, timeout)
        if pulse is not None:
            pulses[servo_id] = pulse
    return PositionSample(time.monotonic(), pulses)


def median_baseline(
    port: serial.Serial,
    ids: Iterable[int],
    timeout: float,
    count: int = 3,
) -> dict[int, int]:
    collected: dict[int, list[int]] = {servo_id: [] for servo_id in ids}
    for _ in range(count):
        sample = scan_positions(port, ids, timeout)
        for servo_id, pulse in sample.pulses.items():
            collected[servo_id].append(pulse)
    return {
        servo_id: int(statistics.median(values))
        for servo_id, values in collected.items()
        if values
    }


def print_snapshot(sample: PositionSample, ids: Iterable[int]) -> None:
    print("ID    pulse    approx_deg")
    print("--    -----    ----------")
    for servo_id in ids:
        pulse = sample.pulses.get(servo_id)
        if pulse is None:
            print(f"{servo_id:03d}   -----    no reply")
        else:
            print(f"{servo_id:03d}   {pulse:5d}    {pulse_to_degrees(pulse):10.3f}")


def confirm_and_release_torque(
    port: serial.Serial,
    ids: Iterable[int],
    timeout: float,
) -> None:
    print()
    print("WARNING: torque release can make the arm fall or collapse under gravity.")
    print("Support every link, clear the workspace, and keep hands out of pinch points.")
    confirmation = input("Type RELEASE to send PULK to all requested IDs: ").strip()
    if confirmation != "RELEASE":
        raise SystemExit("Torque release cancelled; no PULK command was sent.")

    for servo_id in ids:
        reply = exchange(port, build_command(servo_id, TORQUE_RELEASE), timeout)
        text = reply.decode("ascii", errors="backslashreplace").strip()
        print(f"ID {servo_id:03d}: PULK sent; reply={text!r}")
        time.sleep(0.015)
    print("Torque-release requests completed. Do not force any joint that still resists.")


def watch_positions(
    port: serial.Serial,
    ids: tuple[int, ...],
    timeout: float,
    threshold: int,
    duration: float | None,
    clear_screen: bool,
) -> None:
    initial = scan_positions(port, ids, timeout)
    if not initial.pulses:
        raise RuntimeError("none of the requested servo IDs replied")

    history: deque[PositionSample] = deque()
    started = time.monotonic()
    while True:
        sample = scan_positions(port, ids, timeout)
        history.append(sample)
        while history and sample.monotonic_time - history[0].monotonic_time > 0.5:
            history.popleft()
        old = history[0].pulses if history else sample.pulses

        window_delta = {
            servo_id: abs(pulse - old.get(servo_id, pulse))
            for servo_id, pulse in sample.pulses.items()
        }
        moving_id = max(window_delta, key=window_delta.get) if window_delta else None
        moving_amount = window_delta.get(moving_id, 0) if moving_id is not None else 0
        if moving_amount < threshold:
            moving_id = None

        lines = [
            "ZLink2 encoder watch - query-only (PRAD); Ctrl+C to stop",
            f"Port: {port.port} @ {port.baudrate}   motion threshold: {threshold} pulses",
            "",
            "ID    pulse    approx_deg    delta_start    delta_0.5s    state",
            "--    -----    ----------    -----------    ----------    -----",
        ]
        for servo_id in ids:
            pulse = sample.pulses.get(servo_id)
            if pulse is None:
                lines.append(f"{servo_id:03d}   -----    no reply")
                continue
            delta_start = pulse - initial.pulses.get(servo_id, pulse)
            delta_window = pulse - old.get(servo_id, pulse)
            state = "<-- MOVING" if servo_id == moving_id else ""
            lines.append(
                f"{servo_id:03d}   {pulse:5d}    {pulse_to_degrees(pulse):10.3f}"
                f"    {delta_start:+11d}    {delta_window:+10d}    {state}"
            )
        if moving_id is None:
            lines.extend(["", "Detected movement: none"])
        else:
            lines.extend(["", f"Detected movement: ID {moving_id:03d}"])

        prefix = "\x1b[2J\x1b[H" if clear_screen else ""
        print(prefix + "\n".join(lines), flush=True)

        if duration is not None and time.monotonic() - started >= duration:
            return
        time.sleep(0.03)


def movement_candidate(
    port: serial.Serial,
    ids: tuple[int, ...],
    baseline: dict[int, int],
    timeout: float,
    dominance: float,
    confirm_seconds: float,
    min_total_travel: int,
    noise_floor: int,
) -> tuple[int, int, int, int] | None:
    """Observe all IDs for a fixed interval and select the most active one.

    Activity is cumulative travel: every absolute pulse change is added, so
    reversals at a mechanical limit still count. Small per-scan changes up to
    ``noise_floor`` are ignored to suppress encoder jitter.

    Returns ``(ID, signed displacement from baseline, current pulse,
    cumulative_travel)``.
    """
    print("Observation starts in 1 second. Keep all other joints as still as possible.")
    time.sleep(1.0)

    previous = PositionSample(time.monotonic(), dict(baseline))
    total_travel = {servo_id: 0 for servo_id in ids}
    last_pulses = dict(baseline)
    started = time.monotonic()

    while True:
        sample = scan_positions(port, ids, timeout)
        for servo_id, pulse in sample.pulses.items():
            last_pulses[servo_id] = pulse
            if servo_id in previous.pulses:
                raw_change = abs(pulse - previous.pulses[servo_id])
                total_travel[servo_id] += max(0, raw_change - noise_floor)
        previous = sample

        ranked = sorted(total_travel.items(), key=lambda item: item[1], reverse=True)
        candidate, candidate_travel = ranked[0]
        elapsed = time.monotonic() - started
        print(
            f"\rObserving {min(elapsed, confirm_seconds):4.1f}/{confirm_seconds:.1f}s | "
            f"current leader: ID {candidate:03d}, "
            f"total travel={candidate_travel} pulses   ",
            end="",
            flush=True,
        )
        if elapsed >= confirm_seconds:
            break
        time.sleep(0.02)

    print()
    ranked = sorted(total_travel.items(), key=lambda item: item[1], reverse=True)
    print("Activity ranking for this observation:")
    print("rank   ID    total_travel    approx_travel_deg")
    print("----   ---   ------------    -----------------")
    for rank, (servo_id, travel) in enumerate(ranked, start=1):
        print(f"{rank:>4}   {servo_id:03d}   {travel:12d}    {travel * 270 / 2000:17.2f}")

    candidate, candidate_travel = ranked[0]
    second_travel = ranked[1][1] if len(ranked) > 1 else 0
    if candidate_travel < min_total_travel:
        print(
            f"Rejected: largest travel {candidate_travel} is below the required "
            f"{min_total_travel} pulses."
        )
        return None
    if second_travel > 0 and candidate_travel < second_travel * dominance:
        print(
            f"Rejected: ID {candidate:03d} is not {dominance:.2f}x larger than "
            f"the second-place activity ({second_travel} pulses)."
        )
        return None

    pulse = last_pulses.get(candidate, baseline.get(candidate))
    if pulse is None:
        return None
    delta = pulse - baseline.get(candidate, pulse)
    print(
        f"Selected ID {candidate:03d}: {candidate_travel} cumulative pulses, "
        f"second place {second_travel}."
    )
    return candidate, delta, pulse, candidate_travel


def save_mapping(
    output_path: Path,
    port: serial.Serial,
    ids: tuple[int, ...],
    labels: tuple[str, ...],
    mapping: dict[str, dict[str, int | float]],
    min_total_travel: int,
    dominance: float,
    confirm_seconds: float,
    noise_floor: int,
) -> None:
    payload = {
        "timestamp_utc": utc_now(),
        "port": str(port.port),
        "baudrate": port.baudrate,
        "requested_ids": list(ids),
        "label_sequence": list(labels),
        "detection": {
            "minimum_total_travel_pulses": min_total_travel,
            "dominance_ratio": dominance,
            "observation_seconds": confirm_seconds,
            "per_scan_noise_floor_pulses": noise_floor,
        },
        "mapping": mapping,
        "notes": {
            "normal_commands": ["PRAD"],
            "pulse_to_degrees": "(pulse - 500) / 2000 * 270; provisional only",
        },
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def map_physical_joints(
    port: serial.Serial,
    ids: tuple[int, ...],
    labels: tuple[str, ...],
    timeout: float,
    dominance: float,
    confirm_seconds: float,
    min_total_travel: int,
    noise_floor: int,
    output_path: Path,
) -> None:
    if len(labels) != len(ids):
        raise ValueError(
            f"mapping needs one label per ID: received {len(labels)} labels for {len(ids)} IDs"
        )

    print()
    print("Physical joint mapping wizard")
    print("Move only the joint named in each step. Press Ctrl+C at any time to stop.")
    print("Default order is joint_1 from the base outward through joint_7, then gripper.")
    print("The program remains query-only and does not release torque unless requested.")
    print(
        f"Each ID decision uses a fixed {confirm_seconds:.1f}s observation. "
        "Forward and reverse travel are both accumulated."
    )

    mapping: dict[str, dict[str, int | float]] = {}
    assigned_ids: set[int] = set()
    label_index = 0
    while label_index < len(labels):
        label = labels[label_index]
        print()
        input(f"[{label_index + 1}/{len(labels)}] Hold still, then press Enter for {label}: ")
        baseline = median_baseline(port, ids, timeout)
        missing = [servo_id for servo_id in ids if servo_id not in baseline]
        if missing:
            print(f"Warning: no baseline reply from IDs {missing}")
        print(
            f"During the next {confirm_seconds:.1f} seconds, move ONLY {label} "
            "gently back and forth through its safe range."
        )

        result = movement_candidate(
            port,
            ids,
            baseline,
            timeout,
            dominance,
            confirm_seconds,
            min_total_travel,
            noise_floor,
        )
        if result is None:
            print("The observation did not contain one unambiguous moving ID.")
            choice = input("[r]etry, [s]kip, or [q]uit? ").strip().lower()
            if choice == "q":
                break
            if choice == "s":
                label_index += 1
            continue

        candidate, delta, pulse, total_travel = result
        if candidate in assigned_ids:
            owner = next(
                existing_label
                for existing_label, item in mapping.items()
                if item["servo_id"] == candidate
            )
            print(f"ID {candidate:03d} is already assigned to {owner}; please retry.")
            continue

        choice = input(
            f"Assign {label} -> ID {candidate:03d}? "
            "[Enter]=accept, [r]etry, [s]kip, [q]uit: "
        ).strip().lower()
        if choice == "q":
            break
        if choice == "r":
            continue
        if choice == "s":
            label_index += 1
            continue

        mapping[label] = {
            "servo_id": candidate,
            "detection_delta_pulses": delta,
            "cumulative_travel_pulses": total_travel,
            "observation_seconds": confirm_seconds,
            "pulse_when_identified": pulse,
            "approx_degrees_when_identified": round(pulse_to_degrees(pulse), 3),
        }
        assigned_ids.add(candidate)
        save_mapping(
            output_path,
            port,
            ids,
            labels,
            mapping,
            min_total_travel,
            dominance,
            confirm_seconds,
            noise_floor,
        )
        print(f"Saved: {label} -> ID {candidate:03d}")
        label_index += 1

    save_mapping(
        output_path,
        port,
        ids,
        labels,
        mapping,
        min_total_travel,
        dominance,
        confirm_seconds,
        noise_floor,
    )
    print()
    print(f"Mapping saved to: {output_path.resolve()}")
    if mapping:
        for label, item in mapping.items():
            print(f"  {label}: ID {int(item['servo_id']):03d}")
    else:
        print("No mappings were accepted.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read and identify ZLink2/ZServo encoder IDs without commanding motion",
    )
    parser.add_argument(
        "--mode",
        choices=("once", "watch", "map"),
        default="watch",
        help="once: one table; watch: live table; map: guided physical-joint mapping",
    )
    parser.add_argument("--port", default="COM10", help="serial port (default: COM10)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate")
    parser.add_argument("--ids", type=parse_ids, default=DEFAULT_IDS, help="IDs, e.g. 0-7")
    parser.add_argument(
        "--labels",
        type=parse_labels,
        default=DEFAULT_LABELS,
        help="comma-separated labels for map mode",
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=20,
        help="live watch threshold in pulses (20 is about 2.7 degrees)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.03,
        help="reply timeout per ID in seconds",
    )
    parser.add_argument(
        "--dominance",
        type=float,
        default=1.5,
        help="in map mode, first-place total travel must exceed second place by this ratio",
    )
    parser.add_argument(
        "--confirm-seconds",
        type=float,
        default=5.0,
        help="fixed observation duration for each ID decision",
    )
    parser.add_argument(
        "--map-travel-threshold",
        type=int,
        default=150,
        help="minimum cumulative travel required during map observation",
    )
    parser.add_argument(
        "--noise-floor",
        type=int,
        default=2,
        help="ignore per-scan changes up to this many pulses in map mode",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="optional watch duration in seconds; otherwise stop with Ctrl+C",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="do not redraw the terminal in watch mode",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("zlink2_joint_id_map.json"),
        help="map-mode JSON output path",
    )
    parser.add_argument(
        "--release-torque",
        action="store_true",
        help="after an interactive warning, send PULK before reading",
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.baud <= 0:
        raise SystemExit("--baud must be positive")
    if args.threshold <= 0:
        raise SystemExit("--threshold must be positive")
    if not 0.005 <= args.timeout <= 1.0:
        raise SystemExit("--timeout must be between 0.005 and 1.0 seconds")
    if args.dominance < 1.0:
        raise SystemExit("--dominance must be at least 1.0")
    if not 0.5 <= args.confirm_seconds <= 30.0:
        raise SystemExit("--confirm-seconds must be between 0.5 and 30 seconds")
    if args.map_travel_threshold <= 0:
        raise SystemExit("--map-travel-threshold must be positive")
    if not 0 <= args.noise_floor <= 20:
        raise SystemExit("--noise-floor must be between 0 and 20 pulses")
    if args.duration is not None and args.duration <= 0:
        raise SystemExit("--duration must be positive")


def main() -> int:
    args = build_parser().parse_args()
    validate_args(args)

    print("Default safety mode: encoder query only (PRAD); no motion commands.")
    print(f"Opening {args.port} at {args.baud} baud for IDs {list(args.ids)}")
    try:
        with open_port(args.port, args.baud) as port:
            if args.release_torque:
                confirm_and_release_torque(port, args.ids, args.timeout)

            initial = scan_positions(port, args.ids, args.timeout)
            confirmed = sorted(initial.pulses)
            print(f"Confirmed IDs: {confirmed if confirmed else 'none'}")
            if not confirmed:
                print(
                    "No servo replied. Check servo-bus power, DAT/GND wiring, baud rate, "
                    "and whether another program owns the serial port.",
                    file=sys.stderr,
                )
                return 3

            if args.mode == "once":
                print_snapshot(initial, args.ids)
            elif args.mode == "watch":
                watch_positions(
                    port,
                    args.ids,
                    args.timeout,
                    args.threshold,
                    args.duration,
                    not args.no_clear,
                )
            else:
                map_physical_joints(
                    port,
                    args.ids,
                    args.labels,
                    args.timeout,
                    args.dominance,
                    args.confirm_seconds,
                    args.map_travel_threshold,
                    args.noise_floor,
                    args.output,
                )
    except KeyboardInterrupt:
        print("\nStopped by user. The program sent no position command.")
        return 130
    except (SerialException, OSError, RuntimeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
