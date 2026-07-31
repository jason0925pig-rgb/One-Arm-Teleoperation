#!/usr/bin/env python3
"""Record one synchronized ROS 2 teleoperation episode without publishing commands."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "datasets"
DEFAULT_TOPICS = (
    "/teleop/leader_pulses",
    "/teleop/leader_filtered_pulses",
    "/teleop/target_preview",
    "/teleop/bridge_status",
    "/teleop/enabled",
    "/teleop/stop_request",
    "/right_arm/teleop_joint_command",
    "/right_arm/executed_joint_command",
    "/right_arm/joint_states",
    "/right_arm/safety_status",
    "/right_arm/motion_enabled",
    "/right_arm/powered_on",
    "/right_arm/robot_enabled",
    "/right_arm/gripper_command",
    "/right_arm/executed_gripper_command",
    "/right_arm/gripper_state",
    "/right_arm/gripper_status",
    "/right_arm/gripper_feedback_valid",
    "/right_arm/gripper_contact",
)
DEFAULT_HEAD_TOPIC = "/camera_head/color/image_raw/compressed"
DEFAULT_WRIST_TOPIC = "/camera_wrist/color/image_raw/compressed"
DEFAULT_CAMERA_TOPICS = (
    "/camera_head/color/camera_info",
    "/camera_wrist/color/camera_info",
    "/diagnostics",
)
LEROBOT_REQUIRED_TOPICS = (
    "/right_arm/executed_joint_command",
    "/right_arm/joint_states",
    "/right_arm/gripper_command",
    "/right_arm/executed_gripper_command",
    "/right_arm/gripper_feedback_valid",
    "/right_arm/gripper_contact",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", value.strip()).strip("_")
    return cleaned[:80] or "episode"


def run_text(command: list[str]) -> str:
    result = subprocess.run(
        command,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return result.stdout.strip()


def write_metadata(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary.replace(path)


def available_topics() -> set[str]:
    result = subprocess.run(
        ["ros2", "topic", "list"],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "ros2 topic list failed")
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def topic_publisher_count(topic: str) -> int:
    result = subprocess.run(
        ["ros2", "topic", "info", topic],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        return 0
    match = re.search(r"Publisher count:\s*(\d+)", result.stdout)
    return int(match.group(1)) if match else 0


def unique_episode_directory(root: Path, name: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = root / f"{timestamp}_{safe_name(name)}"
    candidate = base
    suffix = 1
    while candidate.exists():
        candidate = root / f"{base.name}_{suffix:02d}"
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record passive ROS2 teleoperation topics and episode metadata. "
            "This tool never publishes a robot command."
        )
    )
    parser.add_argument("--name", required=True, help="short episode name")
    parser.add_argument("--task", default="", help="task description")
    parser.add_argument("--operator", default="")
    parser.add_argument("--notes", default="")
    parser.add_argument(
        "--outcome",
        choices=("unknown", "success", "failure"),
        default="unknown",
    )
    parser.add_argument("--duration", type=float)
    parser.add_argument(
        "--profile",
        choices=("lerobot", "debug"),
        default="lerobot",
        help=(
            "lerobot requires state, executed action, gripper and both cameras; "
            "debug permits incomplete topic sets"
        ),
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="target LeRobot export FPS stored in episode metadata",
    )
    parser.add_argument("--head-topic", default=DEFAULT_HEAD_TOPIC)
    parser.add_argument("--wrist-topic", default=DEFAULT_WRIST_TOPIC)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--episode-path-file",
        type=Path,
        help=(
            "atomically write the created episode directory here so a "
            "supervisor can finalize it"
        ),
    )
    parser.add_argument(
        "--extra-topic",
        action="append",
        default=[],
        help="additional topic such as an RGB/depth camera topic; repeat as needed",
    )
    parser.add_argument(
        "--require-topic",
        action="append",
        default=[],
        help="fail before recording if this topic is absent; repeat as needed",
    )
    parser.add_argument(
        "--storage",
        choices=("mcap", "sqlite3"),
        default="sqlite3",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="check ROS/topic availability without creating an episode",
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.duration is not None and args.duration <= 0.0:
        raise SystemExit("--duration must be positive")
    if args.fps <= 0:
        raise SystemExit("--fps must be positive")
    if args.profile == "lerobot":
        if not args.task:
            raise SystemExit("--task is required for the lerobot profile")
        if args.task != args.task.strip():
            raise SystemExit(
                "--task must not contain leading or trailing whitespace"
            )
    for topic in (
        args.head_topic,
        args.wrist_topic,
        *args.extra_topic,
        *args.require_topic,
    ):
        if not topic.startswith("/") or any(character.isspace() for character in topic):
            raise SystemExit(f"invalid ROS topic name: {topic!r}")


def main() -> int:
    args = build_parser().parse_args()
    validate_args(args)

    try:
        visible_topics = available_topics()
    except (FileNotFoundError, RuntimeError) as exc:
        print(f"ERROR: ROS2 topic discovery failed: {exc}", file=sys.stderr)
        return 2

    required_topics = set(args.require_topic)
    if args.profile == "lerobot":
        required_topics.update(LEROBOT_REQUIRED_TOPICS)
        required_topics.update((args.head_topic, args.wrist_topic))
    missing_required = sorted(required_topics - visible_topics)
    if missing_required:
        print(
            "ERROR: required topics are absent: " + ", ".join(missing_required),
            file=sys.stderr,
        )
        return 3

    missing_publishers = sorted(
        topic
        for topic in required_topics
        if topic_publisher_count(topic) < 1
    )
    if missing_publishers:
        print(
            "ERROR: required topics have no active publisher: "
            + ", ".join(missing_publishers),
            file=sys.stderr,
        )
        return 3

    camera_topics = (args.head_topic, args.wrist_topic)
    topics = tuple(
        dict.fromkeys(
            (
                *DEFAULT_TOPICS,
                *camera_topics,
                *DEFAULT_CAMERA_TOPICS,
                *args.extra_topic,
            )
        )
    )
    print("Passive episode topics:")
    for topic in topics:
        state = "visible" if topic in visible_topics else "waiting"
        print(f"  {topic}: {state}")
    if args.preflight_only:
        print("Preflight complete; no directory or rosbag was created.")
        return 0

    episode_dir = unique_episode_directory(
        args.output_root.expanduser().resolve(),
        args.name,
    )
    bag_dir = episode_dir / "rosbag"
    metadata_path = episode_dir / "episode_metadata.json"
    command = [
        "ros2",
        "bag",
        "record",
        "--storage",
        args.storage,
        "--output",
        str(bag_dir),
        *topics,
    ]
    metadata: dict[str, Any] = {
        "format": "one_arm_teleoperation_episode",
        "format_version": 2,
        "status": "recording",
        "outcome": args.outcome,
        "name": args.name,
        "task": args.task,
        "task_sha256": hashlib.sha256(
            args.task.encode("utf-8")
        ).hexdigest(),
        "operator": args.operator,
        "notes": args.notes,
        "host": socket.gethostname(),
        "started_utc": utc_now(),
        "ended_utc": None,
        "duration_seconds": None,
        "stop_reason": None,
        "git_commit": run_text(
            ["git", "-C", str(PROJECT_ROOT), "rev-parse", "HEAD"]
        ),
        "ros_distro": run_text(["printenv", "ROS_DISTRO"]),
        "storage": args.storage,
        "recording_profile": args.profile,
        "target_lerobot_fps": args.fps,
        "camera_topics": {
            "observation.images.head": args.head_topic,
            "observation.images.wrist_right": args.wrist_topic,
        },
        "joint_state_topic": "/right_arm/joint_states",
        "executed_action_topic": "/right_arm/executed_joint_command",
        "gripper_requested_action_topic": "/right_arm/gripper_command",
        "gripper_action_topic": "/right_arm/executed_gripper_command",
        "gripper_semantics": {
            "normalized_open": 0.0,
            "normalized_closed": 1.0,
            "observation_source": (
                "commanded_state_estimate_until_valid_position_feedback_exists"
            ),
        },
        "topics": list(topics),
        "visible_topics_at_start": sorted(visible_topics),
        "ros2_topic_types_at_start": run_text(["ros2", "topic", "list", "-t"]),
        "rosbag_command": command,
        "robot_motion_authorized_by_recorder": False,
    }
    write_metadata(metadata_path, metadata)

    print(f"Episode directory: {episode_dir}")
    print("Recorder is passive. Press Ctrl+C to finish and finalize metadata.")
    started = time.monotonic()
    stop_reason = "duration_complete" if args.duration is not None else "rosbag_exit"
    try:
        process = subprocess.Popen(command)
    except OSError as exc:
        metadata["status"] = "error"
        metadata["ended_utc"] = utc_now()
        metadata["duration_seconds"] = time.monotonic() - started
        metadata["stop_reason"] = "rosbag_start_failed"
        metadata["error"] = f"{type(exc).__name__}: {exc}"
        write_metadata(metadata_path, metadata)
        print(f"ERROR: failed to start rosbag: {exc}", file=sys.stderr)
        return 4
    if args.episode_path_file is not None:
        state_path = args.episode_path_file.expanduser().resolve()
        state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = state_path.with_suffix(state_path.suffix + ".tmp")
        temporary.write_text(str(episode_dir) + "\n", encoding="utf-8")
        temporary.replace(state_path)
    try:
        while process.poll() is None:
            if args.duration is not None and time.monotonic() - started >= args.duration:
                process.send_signal(signal.SIGINT)
                stop_reason = "duration_complete"
                break
            time.sleep(0.1)
        process.wait(timeout=15.0)
    except KeyboardInterrupt:
        stop_reason = "operator_ctrl_c"
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5.0)
            stop_reason = "rosbag_forced_terminate"
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5.0)
        stop_reason = "rosbag_shutdown_timeout"

    duration = time.monotonic() - started
    metadata["status"] = "complete" if process.returncode == 0 else "error"
    metadata["ended_utc"] = utc_now()
    metadata["duration_seconds"] = duration
    metadata["stop_reason"] = stop_reason
    metadata["rosbag_return_code"] = process.returncode
    write_metadata(metadata_path, metadata)
    print(f"Saved metadata: {metadata_path}")
    print(f"ROS bag return code: {process.returncode}")
    return 0 if process.returncode == 0 else 4


if __name__ == "__main__":
    raise SystemExit(main())
