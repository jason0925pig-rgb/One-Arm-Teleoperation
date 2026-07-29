#!/usr/bin/env python3
"""Record an eight-channel ZLink2 leader arm on Windows.

Normal operation is query-only and sends only PRAD position requests. Each
program run creates one timestamped recording session containing:

* frames.csv    - raw pulse data and provisional relative joint values
* metadata.json - mapping snapshot, timing, quality counters, and stop reason

Raw pulses are always preserved so future calibration can be applied without
re-recording a demonstration.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import socket
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    from serial import SerialException
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - local dependency
    raise SystemExit(
        "pyserial is required. Use the supplied Windows launcher or PlatformIO Python."
    ) from exc

import zlink2_encoder_mapper as zlink
import zlink2_gripper_state as gripper_state


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MAPPING = PROJECT_ROOT / "zlink2_joint_id_map.json"
DEFAULT_GRIPPER_CONFIG = PROJECT_ROOT / "gripper_calibration.json"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "recordings"
FORMAT_NAME = "one_arm_leader_csv"
FORMAT_VERSION = 2
RAD_PER_PULSE_PROVISIONAL = math.radians(270.0 / 2000.0)
JOINT_LABEL_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
CH340_USB_IDS = {(0x1A86, 0x7523), (0x1A86, 0x5523)}
TELEOP_PROTOCOL = "one_arm_teleop"
TELEOP_PROTOCOL_VERSION = 1


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary.replace(path)


def safe_session_name(text: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_-]+", "_", text.strip()).strip("_")
    return value[:60] or "leader_session"


def parse_udp_target(value: str) -> tuple[str, int]:
    try:
        host, port_text = value.rsplit(":", 1)
        port = int(port_text)
    except (ValueError, AttributeError) as exc:
        raise argparse.ArgumentTypeError(
            "UDP target must use HOST:PORT, for example 192.168.50.2:5005"
        ) from exc
    if not host.strip() or not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("UDP target host/port is invalid")
    return host.strip(), port


class UdpTeleopSender:
    """Send complete leader frames; this class has no robot-control capability."""

    def __init__(self, target: tuple[str, int]):
        self.target = target
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.packets_sent = 0
        self.stop_packets_sent = 0
        self.last_sequence = -1
        self._lock = threading.Lock()

    def send(
        self,
        row: dict[str, Any],
        labels: tuple[str, ...],
        ids_by_label: dict[str, int],
        session_id: str,
        deadman_held: bool,
    ) -> None:
        joint_labels = [label for label in labels if label != "gripper"]
        packet = {
            "protocol": TELEOP_PROTOCOL,
            "version": TELEOP_PROTOCOL_VERSION,
            "message_type": "leader_frame",
            "session_id": session_id,
            "sequence": int(row["sequence"]),
            "timestamp_unix_ns": int(row["timestamp_unix_ns"]),
            "complete": True,
            "deadman_held": bool(deadman_held),
            "joint_names": joint_labels,
            "servo_ids": [ids_by_label[label] for label in joint_labels],
            "joint_pulses": [int(row[f"{label}_pulse"]) for label in joint_labels],
            "gripper": {
                "raw_pulse": int(row["gripper_pulse"]),
                "unwrapped_pulse": int(row["gripper_unwrapped_pulse"]),
                "normalized": float(row["gripper_normalized"]),
                "state": str(row["gripper_state"]),
                "state_changed": bool(int(row["gripper_state_changed"])),
            },
        }
        payload = json.dumps(
            packet,
            ensure_ascii=True,
            separators=(",", ":"),
        ).encode("ascii")
        with self._lock:
            self.socket.sendto(payload, self.target)
            self.packets_sent += 1
            self.last_sequence = max(self.last_sequence, int(row["sequence"]))

    def send_stop(
        self,
        session_id: str,
        reason: str,
        repeat: int = 5,
        interval_seconds: float = 0.02,
    ) -> None:
        """Send redundant fail-safe STOP datagrams; STOP is idempotent."""
        safe_reason = reason.strip()[:160] or "remote_stop"
        for index in range(max(1, repeat)):
            with self._lock:
                self.last_sequence += 1
                packet = {
                    "protocol": TELEOP_PROTOCOL,
                    "version": TELEOP_PROTOCOL_VERSION,
                    "message_type": "stop",
                    "session_id": session_id,
                    "sequence": self.last_sequence,
                    "timestamp_unix_ns": time.time_ns(),
                    "reason": safe_reason,
                }
                payload = json.dumps(
                    packet,
                    ensure_ascii=True,
                    separators=(",", ":"),
                ).encode("ascii")
                self.socket.sendto(payload, self.target)
                self.packets_sent += 1
                self.stop_packets_sent += 1
            if index + 1 < repeat:
                time.sleep(interval_seconds)

    def close(self) -> None:
        with self._lock:
            self.socket.close()


class WindowsSafetyKeyboard:
    """Poll Esc/Space independently so a slow serial scan cannot delay STOP."""

    VK_ESCAPE = 0x1B
    VK_SPACE = 0x20

    def __init__(
        self,
        sender: UdpTeleopSender | None,
        session_id: str,
        deadman_required: bool,
        stop_repeat: int,
        stop_interval: float,
    ):
        self.sender = sender
        self.session_id = session_id
        self.deadman_required = deadman_required
        self.stop_repeat = stop_repeat
        self.stop_interval = stop_interval
        self.stop_requested = threading.Event()
        self.stop_reason: str | None = None
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None
        self._space_was_held = False
        self._user32: Any | None = None
        if sys.platform == "win32":
            import ctypes

            self._user32 = ctypes.windll.user32
        elif deadman_required:
            raise RuntimeError("--deadman is supported only on Windows")

    def _key_down(self, virtual_key: int) -> bool:
        return bool(
            self._user32 is not None
            and self._user32.GetAsyncKeyState(virtual_key) & 0x8000
        )

    def deadman_held(self) -> bool:
        return self._key_down(self.VK_SPACE) if self.deadman_required else False

    def start(self) -> None:
        if self._user32 is None:
            return
        self._space_was_held = self.deadman_held()
        self._thread = threading.Thread(
            target=self._run,
            name="teleop-safety-keyboard",
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._shutdown.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)

    def request_stop(self, reason: str) -> None:
        if self.stop_requested.is_set():
            return
        self.stop_reason = reason
        self.stop_requested.set()
        if self.sender is not None:
            self.sender.send_stop(
                self.session_id,
                reason,
                repeat=self.stop_repeat,
                interval_seconds=self.stop_interval,
            )

    def _run(self) -> None:
        escape_was_down = False
        while not self._shutdown.wait(0.01):
            escape_down = self._key_down(self.VK_ESCAPE)
            if escape_down and not escape_was_down:
                self.request_stop("escape_key")
                return
            escape_was_down = escape_down

            if self.deadman_required:
                space_held = self.deadman_held()
                if self._space_was_held and not space_held:
                    self.request_stop("deadman_released")
                    return
                self._space_was_held = space_held


def unique_session_directory(root: Path, session_name: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = root / f"{timestamp}_{safe_session_name(session_name)}"
    candidate = base
    suffix = 1
    while candidate.exists():
        candidate = root / f"{base.name}_{suffix:02d}"
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def load_mapping(path: Path) -> tuple[tuple[str, ...], dict[str, int], dict[str, Any]]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"mapping file does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"mapping file is not valid JSON: {path}: {exc}") from exc

    sequence = raw.get("label_sequence")
    mapping = raw.get("mapping")
    if not isinstance(sequence, list) or not isinstance(mapping, dict):
        raise ValueError("mapping JSON needs label_sequence and mapping")

    labels = tuple(str(label) for label in sequence)
    if len(labels) != 8 or len(set(labels)) != 8:
        raise ValueError("exactly eight unique labels are required")
    if any(not JOINT_LABEL_PATTERN.fullmatch(label) for label in labels):
        raise ValueError("labels may contain only letters, digits, and underscores")
    if labels[-1] != "gripper":
        raise ValueError("the final label must be gripper")

    ids_by_label: dict[str, int] = {}
    for label in labels:
        entry = mapping.get(label)
        if not isinstance(entry, dict) or "servo_id" not in entry:
            raise ValueError(f"missing servo_id for {label}")
        servo_id = int(entry["servo_id"])
        if not 0 <= servo_id <= 999:
            raise ValueError(f"invalid servo ID for {label}: {servo_id}")
        ids_by_label[label] = servo_id

    if len(set(ids_by_label.values())) != len(ids_by_label):
        raise ValueError("each physical label must map to a unique servo ID")
    return labels, ids_by_label, raw


def available_port_rows() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for port in list_ports.comports():
        rows.append(
            {
                "device": port.device,
                "description": port.description,
                "hwid": port.hwid,
                "vid": port.vid,
                "pid": port.pid,
            }
        )
    return rows


def resolve_port(requested: str) -> str:
    if requested.lower() != "auto":
        return requested

    ports = list(list_ports.comports())
    candidates = [
        port
        for port in ports
        if (port.vid, port.pid) in CH340_USB_IDS
        or "CH340" in (port.description or "").upper()
        or "USB-SERIAL" in (port.description or "").upper()
    ]
    if len(candidates) == 1:
        return candidates[0].device
    if len(candidates) > 1:
        details = ", ".join(
            f"{port.device} ({port.description})" for port in candidates
        )
        raise ValueError(f"multiple CH340-like ports found; use --port explicitly: {details}")

    details = ", ".join(
        f"{port.device} ({port.description})" for port in ports
    ) or "none"
    raise ValueError(
        "no CH340/ZLink2 serial adapter is currently present; "
        f"available ports: {details}"
    )


def csv_fieldnames(labels: tuple[str, ...]) -> list[str]:
    fields = [
        "sequence",
        "timestamp_unix_ns",
        "monotonic_ns",
        "elapsed_s",
        "scan_duration_ms",
        "complete",
        "reply_count",
    ]
    for label in labels:
        fields.extend((f"{label}_pulse", f"{label}_delta_pulse"))
        if label == "gripper":
            fields.extend(
                (
                    "gripper_unwrapped_pulse",
                    "gripper_normalized",
                    "gripper_state",
                    "gripper_state_changed",
                )
            )
        else:
            fields.extend(
                (
                    f"{label}_position_deg_provisional",
                    f"{label}_offset_rad_provisional",
                )
            )
    return fields


def build_frame_row(
    sequence: int,
    labels: tuple[str, ...],
    ids_by_label: dict[str, int],
    pulses: dict[int, int],
    baseline: dict[int, int],
    session_started_monotonic: float,
    sample_monotonic: float,
    scan_duration_ms: float,
    gripper_machine: gripper_state.GripperStateMachine,
) -> dict[str, Any]:
    row: dict[str, Any] = {
        "sequence": sequence,
        "timestamp_unix_ns": time.time_ns(),
        "monotonic_ns": int(sample_monotonic * 1_000_000_000),
        "elapsed_s": f"{sample_monotonic - session_started_monotonic:.9f}",
        "scan_duration_ms": f"{scan_duration_ms:.3f}",
        "complete": int(len(pulses) == len(ids_by_label)),
        "reply_count": len(pulses),
    }
    for label in labels:
        servo_id = ids_by_label[label]
        pulse = pulses.get(servo_id)
        start_pulse = baseline.get(servo_id)
        if pulse is None or start_pulse is None:
            row[f"{label}_pulse"] = ""
            row[f"{label}_delta_pulse"] = ""
            if label != "gripper":
                row[f"{label}_position_deg_provisional"] = ""
                row[f"{label}_offset_rad_provisional"] = ""
            continue

        delta = pulse - start_pulse
        row[f"{label}_pulse"] = pulse
        row[f"{label}_delta_pulse"] = delta
        if label == "gripper":
            observation = gripper_machine.update(pulse)
            row["gripper_unwrapped_pulse"] = observation.unwrapped_pulse
            row["gripper_normalized"] = f"{observation.normalized:.6f}"
            row["gripper_state"] = observation.state
            row["gripper_state_changed"] = int(observation.state_changed)
        else:
            row[f"{label}_position_deg_provisional"] = (
                f"{zlink.pulse_to_degrees(pulse):.6f}"
            )
            row[f"{label}_offset_rad_provisional"] = (
                f"{delta * RAD_PER_PULSE_PROVISIONAL:.9f}"
            )
    return row


def initial_metadata(
    args: argparse.Namespace,
    session_dir: Path,
    mapping_path: Path,
    mapping_raw: dict[str, Any],
    gripper_config_path: Path,
    gripper_config_raw: dict[str, Any],
    gripper_calibration: gripper_state.GripperCalibration,
    labels: tuple[str, ...],
    ids_by_label: dict[str, int],
    port_name: str,
    baseline: dict[int, int],
) -> dict[str, Any]:
    baseline_by_label = {
        label: baseline.get(servo_id)
        for label, servo_id in ids_by_label.items()
    }
    return {
        "format": FORMAT_NAME,
        "format_version": FORMAT_VERSION,
        "status": "recording",
        "session_id": session_dir.name,
        "session_name": args.session_name,
        "task": args.task,
        "operator": args.operator,
        "notes": args.notes,
        "created_utc": utc_now(),
        "serial": {
            "port": port_name,
            "baudrate": args.baud,
            "query_timeout_seconds": args.timeout,
            "allowed_normal_operation_commands": ["PRAD"],
            "torque_release_requested": bool(args.release_torque),
        },
        "sampling": {
            "requested_rate_hz": args.rate_hz,
            "baseline_samples": args.baseline_samples,
            "max_consecutive_incomplete_frames": args.max_consecutive_incomplete,
        },
        "network_stream": {
            "enabled": args.udp_target is not None,
            "transport": "udp",
            "protocol": TELEOP_PROTOCOL,
            "protocol_version": TELEOP_PROTOCOL_VERSION,
            "target": (
                f"{args.udp_target[0]}:{args.udp_target[1]}"
                if args.udp_target is not None
                else None
            ),
            "complete_frames_only": True,
            "deadman_enabled": bool(args.deadman),
            "stop_repeat_count": int(args.stop_repeat),
            "robot_motion_authorized_by_sender": False,
            "packets_sent": 0,
            "stop_packets_sent": 0,
        },
        "mapping": {
            "source_path": str(mapping_path),
            "sha256": sha256_file(mapping_path),
            "labels": list(labels),
            "ids_by_label": ids_by_label,
            "snapshot": mapping_raw,
        },
        "gripper_calibration": {
            "source_path": str(gripper_config_path),
            "sha256": sha256_file(gripper_config_path),
            "snapshot": gripper_config_raw,
            "active_parameters": {
                "period_pulses": gripper_calibration.period_pulses,
                "closed_reference": gripper_calibration.closed_reference,
                "open_reference": gripper_calibration.open_reference,
                "close_threshold": gripper_calibration.close_threshold,
                "open_threshold": gripper_calibration.open_threshold,
                "debounce_frames": gripper_calibration.debounce_frames,
            },
            "output_fields": [
                "gripper_unwrapped_pulse",
                "gripper_normalized",
                "gripper_state",
                "gripper_state_changed",
            ],
        },
        "start_pulses_by_label": baseline_by_label,
        "provisional_conversion": {
            "pulse_range": [500, 2500],
            "angle_range_degrees": 270.0,
            "radians_per_pulse": RAD_PER_PULSE_PROVISIONAL,
            "warning": (
                "Joint conversion is not calibrated; raw joint pulses are "
                "authoritative. Gripper state uses the separate calibration snapshot."
            ),
        },
        "files": {
            "frames_csv": "frames.csv",
            "metadata_json": "metadata.json",
        },
        "quality": {
            "frames": 0,
            "complete_frames": 0,
            "incomplete_frames": 0,
            "missing_replies_total": 0,
            "deadline_misses": 0,
        },
    }


def record_session(
    args: argparse.Namespace,
    port: Any,
    labels: tuple[str, ...],
    ids_by_label: dict[str, int],
    mapping_path: Path,
    mapping_raw: dict[str, Any],
    gripper_config_path: Path,
    gripper_config_raw: dict[str, Any],
    gripper_calibration: gripper_state.GripperCalibration,
) -> Path:
    ids = tuple(ids_by_label[label] for label in labels)

    if not args.no_wait:
        print()
        print("Put the leader arm in the desired starting pose.")
        print("This starting pose becomes the offset origin for this session.")
        input("Press Enter to capture the baseline and start recording: ")

    baseline = zlink.median_baseline(
        port,
        ids,
        args.timeout,
        count=args.baseline_samples,
    )
    missing_baseline = [servo_id for servo_id in ids if servo_id not in baseline]
    if missing_baseline:
        raise RuntimeError(
            f"cannot start: no baseline reply from servo IDs {missing_baseline}"
        )

    session_dir = unique_session_directory(args.output_root, args.session_name)
    csv_path = session_dir / "frames.csv"
    metadata_path = session_dir / "metadata.json"
    metadata = initial_metadata(
        args,
        session_dir,
        mapping_path,
        mapping_raw,
        gripper_config_path,
        gripper_config_raw,
        gripper_calibration,
        labels,
        ids_by_label,
        str(port.port),
        baseline,
    )
    write_json_atomic(metadata_path, metadata)
    gripper_machine = gripper_state.GripperStateMachine(gripper_calibration)
    udp_sender = UdpTeleopSender(args.udp_target) if args.udp_target else None
    safety_keyboard = WindowsSafetyKeyboard(
        udp_sender,
        session_dir.name,
        args.deadman,
        args.stop_repeat,
        args.stop_interval,
    )

    requested_period = 1.0 / args.rate_hz
    sequence = 0
    complete_frames = 0
    incomplete_frames = 0
    missing_replies_total = 0
    deadline_misses = 0
    consecutive_incomplete = 0
    stop_reason = "duration_complete"
    error_text: str | None = None
    session_started = time.monotonic()
    next_deadline = session_started
    last_status = session_started

    print()
    print(f"Recording session: {session_dir}")
    print("Safety mode: PRAD query only; no position command is present.")
    if args.deadman:
        print("Deadman enabled: hold Space for live frames; releasing it sends STOP.")
    print("Press Esc or Ctrl+C to send STOP and finalize metadata.")
    safety_keyboard.start()

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=csv_fieldnames(labels))
            writer.writeheader()

            while True:
                if safety_keyboard.stop_requested.is_set():
                    stop_reason = safety_keyboard.stop_reason or "keyboard_stop"
                    break
                scan_started = time.monotonic()
                sample = zlink.scan_positions(port, ids, args.timeout)
                scan_duration_ms = (time.monotonic() - scan_started) * 1000.0
                row = build_frame_row(
                    sequence,
                    labels,
                    ids_by_label,
                    sample.pulses,
                    baseline,
                    session_started,
                    sample.monotonic_time,
                    scan_duration_ms,
                    gripper_machine,
                )
                writer.writerow(row)
                if udp_sender is not None and int(row["complete"]) == 1:
                    udp_sender.send(
                        row,
                        labels,
                        ids_by_label,
                        session_dir.name,
                        safety_keyboard.deadman_held(),
                    )

                reply_count = len(sample.pulses)
                missing_count = len(ids) - reply_count
                missing_replies_total += missing_count
                if missing_count == 0:
                    complete_frames += 1
                    consecutive_incomplete = 0
                else:
                    incomplete_frames += 1
                    consecutive_incomplete += 1

                sequence += 1
                if sequence % args.flush_every == 0:
                    csv_file.flush()

                now = time.monotonic()
                elapsed = now - session_started
                if now - last_status >= 1.0:
                    actual_rate = sequence / elapsed if elapsed > 0 else 0.0
                    pulse_summary = " ".join(
                        f"{label}={sample.pulses.get(ids_by_label[label], '-')}"
                        for label in labels
                    )
                    print(
                        f"frames={sequence} elapsed={elapsed:.1f}s "
                        f"rate={actual_rate:.2f}Hz complete={complete_frames}/{sequence} "
                        f"| {pulse_summary} "
                        f"gripper_state={row.get('gripper_state', '-')}",
                        flush=True,
                    )
                    last_status = now

                if (
                    args.max_consecutive_incomplete > 0
                    and consecutive_incomplete >= args.max_consecutive_incomplete
                ):
                    raise RuntimeError(
                        f"{consecutive_incomplete} consecutive incomplete frames; "
                        "recording stopped to protect data quality"
                    )
                if args.duration is not None and elapsed >= args.duration:
                    break
                if safety_keyboard.stop_requested.is_set():
                    stop_reason = safety_keyboard.stop_reason or "keyboard_stop"
                    break

                next_deadline += requested_period
                remaining = next_deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    deadline_misses += 1
                    next_deadline = time.monotonic()
    except KeyboardInterrupt:
        stop_reason = "user_stopped"
        print("\nStop requested; finalizing the session.")
    except Exception as exc:
        stop_reason = "error"
        error_text = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        safety_keyboard.close()
        stop_send_error: str | None = None
        if udp_sender is not None:
            try:
                udp_sender.send_stop(
                    session_dir.name,
                    stop_reason,
                    repeat=args.stop_repeat,
                    interval_seconds=args.stop_interval,
                )
            except OSError as exc:
                stop_send_error = f"{type(exc).__name__}: {exc}"
                print(f"WARNING: final UDP STOP send failed: {exc}", file=sys.stderr)
        ended = time.monotonic()
        duration_seconds = max(0.0, ended - session_started)
        actual_rate = sequence / duration_seconds if duration_seconds > 0 else 0.0
        metadata["status"] = stop_reason
        metadata["ended_utc"] = utc_now()
        metadata["duration_seconds"] = duration_seconds
        metadata["quality"] = {
            "frames": sequence,
            "complete_frames": complete_frames,
            "incomplete_frames": incomplete_frames,
            "complete_fraction": (
                complete_frames / sequence if sequence else 0.0
            ),
            "missing_replies_total": missing_replies_total,
            "deadline_misses": deadline_misses,
            "actual_average_rate_hz": actual_rate,
            "requested_rate_achieved": actual_rate >= args.rate_hz * 0.8,
        }
        metadata["network_stream"]["packets_sent"] = (
            udp_sender.packets_sent if udp_sender is not None else 0
        )
        metadata["network_stream"]["stop_packets_sent"] = (
            udp_sender.stop_packets_sent if udp_sender is not None else 0
        )
        if stop_send_error is not None:
            metadata["network_stream"]["stop_send_error"] = stop_send_error
        if udp_sender is not None:
            udp_sender.close()
        if error_text is not None:
            metadata["error"] = error_text
        write_json_atomic(metadata_path, metadata)

    print(f"Saved frames: {csv_path}")
    print(f"Saved metadata: {metadata_path}")
    print(
        f"Summary: {sequence} frames, {complete_frames} complete, "
        f"{metadata['quality']['actual_average_rate_hz']:.2f} Hz average"
    )
    return session_dir


def print_validation(
    mapping_path: Path,
    labels: tuple[str, ...],
    ids_by_label: dict[str, int],
    gripper_config_path: Path,
    gripper_calibration: gripper_state.GripperCalibration,
) -> None:
    print(f"Mapping JSON: {mapping_path}")
    print("Physical mapping:")
    for label in labels:
        print(f"  {label:>8} -> ID {ids_by_label[label]:03d}")
    print(f"Gripper calibration JSON: {gripper_config_path}")
    print(
        "Gripper state machine: "
        f"CLOSED <= {gripper_calibration.close_threshold}, "
        f"OPEN >= {gripper_calibration.open_threshold}, "
        f"debounce={gripper_calibration.debounce_frames} frames"
    )
    print("Detected serial ports:")
    rows = available_port_rows()
    if not rows:
        print("  none")
    for row in rows:
        vid_pid = (
            f"{row['vid']:04X}:{row['pid']:04X}"
            if row["vid"] is not None and row["pid"] is not None
            else "n/a"
        )
        print(
            f"  {row['device']}: {row['description']} "
            f"(USB VID:PID {vid_pid})"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record eight-channel ZLink2 leader-arm data on Windows",
    )
    parser.add_argument(
        "--port",
        default="auto",
        help="serial port or auto to find a CH340 adapter (default: auto)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument(
        "--gripper-config",
        type=Path,
        default=DEFAULT_GRIPPER_CONFIG,
        help="calibrated gripper state-machine JSON",
    )
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--udp-target",
        type=parse_udp_target,
        help=(
            "optional Ubuntu bridge destination HOST:PORT; sends read-only "
            "leader frames and does not authorize robot motion"
        ),
    )
    parser.add_argument(
        "--deadman",
        action="store_true",
        help=(
            "mark live frames active only while Space is held; releasing Space "
            "sends repeated STOP packets and ends the session"
        ),
    )
    parser.add_argument(
        "--stop-repeat",
        type=int,
        default=5,
        help="number of redundant UDP STOP datagrams (default: 5)",
    )
    parser.add_argument(
        "--stop-interval",
        type=float,
        default=0.02,
        help="seconds between redundant STOP datagrams (default: 0.02)",
    )
    parser.add_argument("--session-name", default="leader_session")
    parser.add_argument("--task", default="")
    parser.add_argument("--operator", default="")
    parser.add_argument("--notes", default="")
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=10.0,
        help="requested scan rate; actual rate is measured and saved",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="optional recording duration in seconds; otherwise stop with Ctrl+C",
    )
    parser.add_argument("--timeout", type=float, default=0.03)
    parser.add_argument("--baseline-samples", type=int, default=5)
    parser.add_argument("--flush-every", type=int, default=10)
    parser.add_argument(
        "--max-consecutive-incomplete",
        type=int,
        default=20,
        help="stop after this many incomplete frames; 0 disables the guard",
    )
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help="start immediately without waiting for Enter",
    )
    parser.add_argument(
        "--release-torque",
        action="store_true",
        help="interactively send PULK before recording; support the arm first",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate mapping and list serial ports without opening hardware",
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.baud <= 0:
        raise SystemExit("--baud must be positive")
    if not 0.5 <= args.rate_hz <= 100.0:
        raise SystemExit("--rate-hz must be between 0.5 and 100")
    if args.duration is not None and args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if not 0.005 <= args.timeout <= 1.0:
        raise SystemExit("--timeout must be between 0.005 and 1.0 seconds")
    if not 1 <= args.baseline_samples <= 101:
        raise SystemExit("--baseline-samples must be between 1 and 101")
    if args.flush_every <= 0:
        raise SystemExit("--flush-every must be positive")
    if args.max_consecutive_incomplete < 0:
        raise SystemExit("--max-consecutive-incomplete cannot be negative")
    if not 1 <= args.stop_repeat <= 20:
        raise SystemExit("--stop-repeat must be between 1 and 20")
    if not 0.0 <= args.stop_interval <= 0.20:
        raise SystemExit("--stop-interval must be between 0 and 0.20 seconds")
    if args.deadman and args.udp_target is None:
        raise SystemExit("--deadman requires --udp-target")


def main() -> int:
    args = build_parser().parse_args()
    validate_args(args)
    mapping_path = args.mapping.expanduser().resolve()
    gripper_config_path = args.gripper_config.expanduser().resolve()
    args.output_root = args.output_root.expanduser().resolve()

    try:
        labels, ids_by_label, mapping_raw = load_mapping(mapping_path)
        gripper_calibration, gripper_config_raw = gripper_state.load_calibration(
            gripper_config_path
        )
        if args.validate_only:
            print_validation(
                mapping_path,
                labels,
                ids_by_label,
                gripper_config_path,
                gripper_calibration,
            )
            return 0

        port_name = resolve_port(args.port)
        print("Normal safety mode: PRAD query only; no motion commands.")
        print(f"Opening {port_name} at {args.baud} baud.")
        with zlink.open_port(port_name, args.baud) as port:
            if args.release_torque:
                zlink.confirm_and_release_torque(
                    port,
                    ids_by_label.values(),
                    args.timeout,
                )

            probe = zlink.scan_positions(
                port,
                tuple(ids_by_label[label] for label in labels),
                args.timeout,
            )
            missing = [
                ids_by_label[label]
                for label in labels
                if ids_by_label[label] not in probe.pulses
            ]
            if missing:
                raise RuntimeError(
                    f"startup check failed; no reply from servo IDs {missing}"
                )
            print(f"Startup check passed: {len(probe.pulses)}/8 IDs replied.")
            record_session(
                args,
                port,
                labels,
                ids_by_label,
                mapping_path,
                mapping_raw,
                gripper_config_path,
                gripper_config_raw,
                gripper_calibration,
            )
    except (SerialException, OSError, RuntimeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
