#!/usr/bin/env python3
"""Safely probe Zhonglin-style bus servos through a ZLink2 adapter.

This utility intentionally exposes only two query commands:

* PVER - read the servo firmware/version response
* PRAD - read the current position pulse

It contains no position, torque, calibration, ID-write, or reset command.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - depends on the local environment
    raise SystemExit(
        "pyserial is required. Run this script with PlatformIO's Python, for example:\n"
        r"  %USERPROFILE%\.platformio\penv\Scripts\python.exe tools\zlink2_safe_probe.py"
    ) from exc


VERSION_QUERY = "PVER"
POSITION_QUERY = "PRAD"
ALLOWED_QUERIES = (VERSION_QUERY, POSITION_QUERY)
POSITION_PATTERN = re.compile(r"#(?P<id>\d{3})P(?P<pulse>\d{4})")


@dataclass
class QueryResult:
    command: str
    attempts: int
    raw_hex: str
    raw_text: str
    non_echo_reply: bool


@dataclass
class ServoResult:
    servo_id: int
    confirmed: bool
    position_pulse: int | None
    estimated_degrees: float | None
    version: QueryResult
    position: QueryResult


def parse_id_spec(value: str) -> list[int]:
    """Parse values such as ``0-7`` or ``0,2,5-8``."""
    ids: set[int] = set()
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        if "-" in item:
            start_text, end_text = item.split("-", 1)
            start, end = int(start_text), int(end_text)
            if start > end:
                raise argparse.ArgumentTypeError(f"invalid descending ID range: {item}")
            ids.update(range(start, end + 1))
        else:
            ids.add(int(item))

    if not ids:
        raise argparse.ArgumentTypeError("at least one servo ID is required")
    if min(ids) < 0 or max(ids) > 999:
        raise argparse.ArgumentTypeError("servo IDs must be between 0 and 999")
    return sorted(ids)


def build_query(servo_id: int, query: str) -> bytes:
    if query not in ALLOWED_QUERIES:
        raise ValueError(f"unsafe or unsupported query: {query}")
    return f"#{servo_id:03d}{query}!".encode("ascii")


def read_until_quiet(
    port: serial.Serial,
    total_timeout: float,
    quiet_time: float = 0.025,
) -> bytes:
    deadline = time.monotonic() + total_timeout
    last_data_at: float | None = None
    data = bytearray()

    while time.monotonic() < deadline:
        waiting = port.in_waiting
        chunk = port.read(waiting if waiting else 1)
        if chunk:
            data.extend(chunk)
            last_data_at = time.monotonic()
        elif last_data_at is not None and time.monotonic() - last_data_at >= quiet_time:
            break

    return bytes(data)


def query_once(
    port: serial.Serial,
    servo_id: int,
    query: str,
    timeout: float,
    retries: int,
) -> QueryResult:
    command = build_query(servo_id, query)
    best_reply = b""
    attempts = 0

    for attempt in range(1, retries + 2):
        attempts = attempt
        port.reset_input_buffer()
        port.write(command)
        port.flush()
        reply = read_until_quiet(port, timeout)
        if len(reply) > len(best_reply):
            best_reply = reply
        if reply and reply.strip() != command:
            break
        time.sleep(0.02)

    raw_text = best_reply.decode("ascii", errors="backslashreplace")
    return QueryResult(
        command=command.decode("ascii"),
        attempts=attempts,
        raw_hex=best_reply.hex(" "),
        raw_text=raw_text,
        non_echo_reply=bool(best_reply and best_reply.strip() != command),
    )


def pulse_to_degrees(pulse: int) -> float:
    """Match the repository's provisional 500-2500 pulse / 270-degree mapping."""
    return round((pulse - 500) / 2000 * 270, 3)


def find_position(reply: bytes, servo_id: int) -> int | None:
    text = reply.decode("ascii", errors="ignore")
    for match in POSITION_PATTERN.finditer(text):
        if int(match.group("id")) == servo_id:
            return int(match.group("pulse"))
    return None


def probe_servo(
    port: serial.Serial,
    servo_id: int,
    timeout: float,
    retries: int,
    gap: float,
) -> ServoResult:
    version = query_once(port, servo_id, VERSION_QUERY, timeout, retries)
    time.sleep(gap)
    position = query_once(port, servo_id, POSITION_QUERY, timeout, retries)
    pulse = find_position(bytes.fromhex(position.raw_hex), servo_id) if position.raw_hex else None
    confirmed = pulse is not None or version.non_echo_reply or position.non_echo_reply
    return ServoResult(
        servo_id=servo_id,
        confirmed=confirmed,
        position_pulse=pulse,
        estimated_degrees=pulse_to_degrees(pulse) if pulse is not None else None,
        version=version,
        position=position,
    )


def open_serial_port(port_name: str, baudrate: int) -> serial.Serial:
    port = serial.Serial()
    port.port = port_name
    port.baudrate = baudrate
    port.timeout = 0.01
    port.write_timeout = 0.25
    # Set inactive modem-line states before opening. ZLink2 should not need either
    # signal for its one-wire servo bus.
    port.dtr = False
    port.rts = False
    port.open()
    time.sleep(0.1)
    port.reset_input_buffer()
    port.reset_output_buffer()
    return port


def result_payload(
    port_name: str,
    baudrate: int,
    ids: Iterable[int],
    results: list[ServoResult],
) -> dict[str, object]:
    return {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "port": port_name,
        "baudrate": baudrate,
        "requested_ids": list(ids),
        "safety": {
            "allowed_queries": list(ALLOWED_QUERIES),
            "state_changing_commands_present": False,
        },
        "confirmed_ids": [result.servo_id for result in results if result.confirmed],
        "results": [asdict(result) for result in results],
    }


def print_result(result: ServoResult) -> None:
    status = "REPLIED" if result.confirmed else "no reply"
    position = (
        f"pulse={result.position_pulse}, approx={result.estimated_degrees:.3f} deg"
        if result.position_pulse is not None
        else "position=unknown"
    )
    print(f"ID {result.servo_id:03d}: {status}; {position}")
    if result.version.raw_text:
        print(f"  PVER raw: {result.version.raw_text!r}")
    if result.position.raw_text:
        print(f"  PRAD raw: {result.position.raw_text!r}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only ZLink2 servo discovery for Windows",
    )
    parser.add_argument("--port", default="COM10", help="serial port (default: COM10)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate (default: 115200)")
    parser.add_argument(
        "--ids",
        type=parse_id_spec,
        default=parse_id_spec("0-7"),
        help="servo IDs, for example 0-7 or 0,2,5-8 (default: 0-7)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.15,
        help="reply timeout in seconds per attempt (default: 0.15)",
    )
    parser.add_argument("--retries", type=int, default=1, help="extra attempts per query (default: 1)")
    parser.add_argument(
        "--gap-ms",
        type=float,
        default=30.0,
        help="quiet gap between version and position queries (default: 30 ms)",
    )
    parser.add_argument("--json", type=Path, help="optional JSON result path")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the exact allowlisted commands without opening the serial port",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.timeout <= 0 or args.timeout > 2:
        raise SystemExit("--timeout must be greater than 0 and no more than 2 seconds")
    if args.retries < 0 or args.retries > 5:
        raise SystemExit("--retries must be between 0 and 5")
    if args.gap_ms < 0 or args.gap_ms > 1000:
        raise SystemExit("--gap-ms must be between 0 and 1000")

    print("Safety mode: query-only; allowed commands are PVER and PRAD.")
    print(f"Target: {args.port} at {args.baud} baud; IDs: {args.ids}")

    if args.dry_run:
        for servo_id in args.ids:
            print(build_query(servo_id, VERSION_QUERY).decode("ascii"))
            print(build_query(servo_id, POSITION_QUERY).decode("ascii"))
        return 0

    try:
        with open_serial_port(args.port, args.baud) as port:
            results = []
            for servo_id in args.ids:
                result = probe_servo(
                    port,
                    servo_id,
                    args.timeout,
                    args.retries,
                    args.gap_ms / 1000,
                )
                results.append(result)
                print_result(result)
    except (SerialException, OSError) as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        print("Close any serial monitor using the port, verify power/wiring, and retry.", file=sys.stderr)
        return 2

    payload = result_payload(args.port, args.baud, args.ids, results)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Saved JSON result: {args.json.resolve()}")

    confirmed = payload["confirmed_ids"]
    print(f"Confirmed servo IDs: {confirmed if confirmed else 'none'}")
    if not confirmed:
        print(
            "No replies does not prove a hardware failure; possible causes include a different "
            "baud rate/protocol, unpowered servo bus, reversed DAT wiring, or different IDs."
        )
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
