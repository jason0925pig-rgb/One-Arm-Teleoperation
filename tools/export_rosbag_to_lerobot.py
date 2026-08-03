#!/usr/bin/env python3
"""Export one passive ROS 2 episode to an official LeRobot Dataset v3."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import sys
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np


JOINT_NAMES = tuple(f"right_joint{index}" for index in range(1, 8))
STATE_NAMES = (*JOINT_NAMES, "right_gripper_closed")
DEFAULT_STATE_TOPIC = "/right_arm/joint_states"
DEFAULT_ACTION_TOPIC = "/right_arm/executed_joint_command"
DEFAULT_GRIPPER_ACTION_TOPIC = "/right_arm/executed_gripper_command"
DEFAULT_GRIPPER_FEEDBACK_VALID_TOPIC = (
    "/right_arm/gripper_feedback_valid"
)
DEFAULT_GRIPPER_CONTACT_TOPIC = "/right_arm/gripper_contact"
DEFAULT_HEAD_TOPIC = "/camera_head/color/image_raw/compressed"
DEFAULT_WRIST_TOPIC = "/camera_wrist/color/image_raw/compressed"


@dataclass
class TimedStream:
    topic: str
    type_name: str = ""
    times_ns: list[int] = field(default_factory=list)
    values: list[Any] = field(default_factory=list)

    def append(self, timestamp_ns: int, value: Any) -> None:
        self.times_ns.append(int(timestamp_ns))
        self.values.append(value)

    def sort(self) -> None:
        if len(self.times_ns) < 2:
            return
        order = sorted(range(len(self.times_ns)), key=self.times_ns.__getitem__)
        self.times_ns = [self.times_ns[index] for index in order]
        self.values = [self.values[index] for index in order]


@dataclass(frozen=True)
class SelectedSample:
    index: int
    skew_ns: int


@dataclass(frozen=True)
class SqliteMessageRef:
    """A lightweight pointer to one serialized message in a rosbag DB."""

    database_index: int
    message_id: int
    type_name: str


class SqliteMessageStore:
    """Index a sqlite3 rosbag without retaining image payloads in RAM."""

    def __init__(self, bag_dir: Path, cache_size: int = 16) -> None:
        database_paths = sorted(bag_dir.glob("*.db3"))
        if not database_paths:
            raise FileNotFoundError(
                f"no sqlite3 rosbag database (*.db3) found in {bag_dir}"
            )
        self._connections: list[sqlite3.Connection] = []
        for path in database_paths:
            connection = sqlite3.connect(
                f"{path.resolve().as_uri()}?mode=ro",
                uri=True,
            )
            connection.execute("PRAGMA query_only=ON")
            self._connections.append(connection)
        self._cache_size = max(int(cache_size), 0)
        self._cache: OrderedDict[SqliteMessageRef, Any] = OrderedDict()
        self._message_types: dict[str, Any] = {}

    def close(self) -> None:
        self._cache.clear()
        for connection in self._connections:
            connection.close()
        self._connections.clear()

    def __enter__(self) -> "SqliteMessageStore":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def _message_type(self, type_name: str) -> Any:
        try:
            return self._message_types[type_name]
        except KeyError:
            try:
                from rosidl_runtime_py.utilities import get_message
            except ImportError as exc:
                raise RuntimeError(
                    "run this exporter in the sourced Ubuntu ROS 2 environment"
                ) from exc
            message_type = get_message(type_name)
            self._message_types[type_name] = message_type
            return message_type

    def _deserialize(self, serialized: bytes, type_name: str) -> Any:
        try:
            from rclpy.serialization import deserialize_message
        except ImportError as exc:
            raise RuntimeError(
                "run this exporter in the sourced Ubuntu ROS 2 environment"
            ) from exc
        return deserialize_message(serialized, self._message_type(type_name))

    def index(
        self,
        requested_topics: set[str],
    ) -> dict[str, TimedStream]:
        streams = {topic: TimedStream(topic=topic) for topic in requested_topics}
        for database_index, connection in enumerate(self._connections):
            topic_rows = connection.execute(
                "SELECT id, name, type FROM topics"
            ).fetchall()
            selected_topics = {
                int(topic_id): (str(name), str(type_name))
                for topic_id, name, type_name in topic_rows
                if str(name) in requested_topics
            }
            for topic_id, (topic, type_name) in selected_topics.items():
                stream = streams[topic]
                if stream.type_name and stream.type_name != type_name:
                    raise ValueError(
                        f"{topic} changes type between bag files: "
                        f"{stream.type_name!r} vs {type_name!r}"
                    )
                stream.type_name = type_name
                cursor = connection.execute(
                    "SELECT id, timestamp, data FROM messages "
                    "WHERE topic_id = ? ORDER BY timestamp",
                    (topic_id,),
                )
                for message_id, bag_timestamp_ns, serialized in cursor:
                    message = self._deserialize(serialized, type_name)
                    stream.append(
                        _header_timestamp_ns(message, int(bag_timestamp_ns)),
                        SqliteMessageRef(
                            database_index=database_index,
                            message_id=int(message_id),
                            type_name=type_name,
                        ),
                    )
        for stream in streams.values():
            stream.sort()
        return streams

    def load(self, reference: SqliteMessageRef) -> Any:
        cached = self._cache.get(reference)
        if cached is not None:
            self._cache.move_to_end(reference)
            return cached
        try:
            row = self._connections[reference.database_index].execute(
                "SELECT data FROM messages WHERE id = ?",
                (reference.message_id,),
            ).fetchone()
        except IndexError as exc:
            raise ValueError("invalid rosbag database reference") from exc
        if row is None:
            raise ValueError(
                f"rosbag message id {reference.message_id} is missing"
            )
        message = self._deserialize(row[0], reference.type_name)
        if self._cache_size:
            self._cache[reference] = message
            self._cache.move_to_end(reference)
            while len(self._cache) > self._cache_size:
                self._cache.popitem(last=False)
        return message


def _header_timestamp_ns(message: Any, fallback_ns: int) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return int(fallback_ns)
    seconds = int(getattr(stamp, "sec", 0))
    nanoseconds = int(getattr(stamp, "nanosec", 0))
    timestamp = seconds * 1_000_000_000 + nanoseconds
    return timestamp if timestamp > 0 else int(fallback_ns)


def nearest_sample(
    times_ns: Sequence[int],
    target_ns: int,
    maximum_skew_ns: int,
) -> SelectedSample | None:
    if not times_ns:
        return None
    insertion = bisect.bisect_left(times_ns, target_ns)
    candidates: list[int] = []
    if insertion < len(times_ns):
        candidates.append(insertion)
    if insertion > 0:
        candidates.append(insertion - 1)
    index = min(candidates, key=lambda item: abs(times_ns[item] - target_ns))
    skew = abs(times_ns[index] - target_ns)
    if skew > maximum_skew_ns:
        return None
    return SelectedSample(index=index, skew_ns=skew)


def previous_sample(
    times_ns: Sequence[int],
    target_ns: int,
    maximum_age_ns: int,
) -> SelectedSample | None:
    index = bisect.bisect_right(times_ns, target_ns) - 1
    if index < 0:
        return None
    age = target_ns - times_ns[index]
    if age > maximum_age_ns:
        return None
    return SelectedSample(index=index, skew_ns=age)


def uniform_grid_ns(start_ns: int, end_ns: int, fps: int) -> list[int]:
    if fps <= 0 or end_ns < start_ns:
        return []
    duration_ns = end_ns - start_ns
    count = math.floor(duration_ns * fps / 1_000_000_000) + 1
    return [
        start_ns + round(index * 1_000_000_000 / fps)
        for index in range(count)
    ]


def _joint_positions(message: Any) -> np.ndarray:
    positions = tuple(float(value) for value in message.position)
    names = tuple(str(name) for name in message.name)
    if names:
        lookup = dict(zip(names, positions, strict=True))
        try:
            ordered = [lookup[name] for name in JOINT_NAMES]
        except KeyError as exc:
            raise ValueError(
                f"JointState is missing required joint {exc.args[0]!r}"
            ) from exc
    else:
        if len(positions) != len(JOINT_NAMES):
            raise ValueError("unnamed JointState must contain exactly 7 values")
        ordered = list(positions)
    result = np.asarray(ordered, dtype=np.float32)
    if not np.isfinite(result).all():
        raise ValueError("JointState contains NaN or infinity")
    return result


def _closed_value(message: Any) -> np.float32:
    # The bridge Bool means requested_open. LeRobot convention here is
    # normalized 0=open and 1=closed.
    return np.float32(0.0 if bool(message.data) else 1.0)


def _bool_float(message: Any) -> np.float32:
    return np.float32(1.0 if bool(message.data) else 0.0)


def _decode_image(message: Any, type_name: str) -> np.ndarray:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV is required for image decoding; install python3-opencv"
        ) from exc

    if type_name == "sensor_msgs/msg/CompressedImage":
        encoded = np.frombuffer(bytes(message.data), dtype=np.uint8)
        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("JPEG image could not be decoded")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    if type_name != "sensor_msgs/msg/Image":
        raise ValueError(
            f"unsupported camera message type {type_name!r}; "
            "expected sensor_msgs/msg/Image or CompressedImage"
        )
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    if height <= 0 or width <= 0 or step <= 0:
        raise ValueError("raw image has invalid dimensions")
    rows = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
        height, step
    )
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }
    if encoding not in channels_by_encoding:
        raise ValueError(f"unsupported raw image encoding {encoding!r}")
    channels = channels_by_encoding[encoding]
    required = width * channels
    if step < required:
        raise ValueError("raw image step is shorter than one pixel row")
    image = rows[:, :required].reshape(height, width, channels)
    if encoding == "bgr8":
        image = image[:, :, ::-1]
    elif encoding == "rgba8":
        image = image[:, :, :3]
    elif encoding == "bgra8":
        image = image[:, :, [2, 1, 0]]
    elif encoding == "mono8":
        image = np.repeat(image, 3, axis=2)
    return np.ascontiguousarray(image, dtype=np.uint8)


def _read_episode_metadata(episode_dir: Path) -> dict[str, Any]:
    path = episode_dir / "episode_metadata.json"
    if not path.is_file():
        raise FileNotFoundError(f"episode metadata is missing: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("episode metadata root must be an object")
    return payload


def _read_rosbag(
    bag_dir: Path,
    storage_id: str,
    requested_topics: set[str],
) -> tuple[dict[str, TimedStream], SqliteMessageStore]:
    if storage_id != "sqlite3":
        raise ValueError(
            f"memory-bounded export currently requires sqlite3, got "
            f"{storage_id!r}"
        )
    store = SqliteMessageStore(bag_dir)
    try:
        return store.index(requested_topics), store
    except Exception:
        store.close()
        raise


def _validate_stream_type(
    stream: TimedStream,
    expected: set[str],
) -> None:
    if stream.type_name not in expected:
        raise ValueError(
            f"{stream.topic} has type {stream.type_name!r}, expected "
            + " or ".join(sorted(expected))
        )
    if not stream.times_ns:
        if stream.topic == DEFAULT_ACTION_TOPIC:
            raise SystemExit(
                "LeRobot export refused: "
                f"{stream.topic} contains no executed robot actions. "
                "The raw ROS bag is preserved, but this episode is not valid "
                "training data. Check that FULL_TELEOP_READY remained active "
                "and that the robot actually moved; target_preview must not "
                "be substituted for executed actions."
            )
        raise SystemExit(f"{stream.topic} contains no messages")


def _summarize_skews(values_ns: Sequence[int]) -> dict[str, float]:
    milliseconds = np.asarray(values_ns, dtype=np.float64) / 1_000_000.0
    return {
        "mean_ms": float(milliseconds.mean()),
        "p95_ms": float(np.quantile(milliseconds, 0.95)),
        "max_ms": float(milliseconds.max()),
    }


def _feature_schema(
    head_shape: tuple[int, int, int],
    wrist_shape: tuple[int, int, int],
) -> dict[str, dict[str, Any]]:
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (8,),
            "names": list(STATE_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": (8,),
            "names": list(STATE_NAMES),
        },
        "observation.gripper_contact": {
            "dtype": "float32",
            "shape": (1,),
            "names": ["right_gripper_contact"],
        },
        "observation.gripper_feedback_valid": {
            "dtype": "float32",
            "shape": (1,),
            "names": ["right_gripper_position_feedback_valid"],
        },
        "observation.images.head": {
            "dtype": "video",
            "shape": head_shape,
            "names": ["height", "width", "channels"],
            "info": {"is_depth_map": False},
        },
        "observation.images.wrist_right": {
            "dtype": "video",
            "shape": wrist_shape,
            "names": ["height", "width", "channels"],
            "info": {"is_depth_map": False},
        },
    }


def _schema_matches(
    current: dict[str, dict[str, Any]],
    expected: dict[str, dict[str, Any]],
) -> bool:
    # LeRobot adds bookkeeping features such as timestamp, frame_index,
    # episode_index, index and task_index when a dataset is created.  Those
    # generated fields are absent from our user feature declaration and must
    # not make every subsequent episode fail schema validation.
    if not set(expected).issubset(current):
        return False
    for key, value in expected.items():
        other = current[key]
        if value.get("dtype") != other.get("dtype"):
            return False
        if tuple(value.get("shape", ())) != tuple(other.get("shape", ())):
            return False
        if value.get("names") != other.get("names"):
            return False
    return True


def _write_report(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary.replace(path)


def _completed_export_index(
    report_path: Path,
    dataset_root: Path,
    repo_id: str,
) -> int | None:
    """Return an existing completed export index for idempotent retries."""
    if not report_path.is_file():
        return None
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
        index = int(report["dataset_episode_index"])
        reported_root = Path(str(report["dataset_root"])).resolve()
        reported_repo = str(report["repo_id"])
        info = json.loads(
            (dataset_root / "meta" / "info.json").read_text(encoding="utf-8")
        )
        total_episodes = int(info["total_episodes"])
    except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError):
        return None
    if (
        report.get("status") == "complete"
        and reported_root == dataset_root
        and reported_repo == repo_id
        and 0 <= index < total_episodes
    ):
        return index
    return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate, synchronize and export one raw ROS2 episode to "
            "LeRobot Dataset v3. This program never publishes ROS commands."
        )
    )
    parser.add_argument("--episode-dir", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--fps", type=int)
    parser.add_argument("--task")
    parser.add_argument("--allow-non-success", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--action-topic", default=DEFAULT_ACTION_TOPIC)
    parser.add_argument(
        "--gripper-action-topic", default=DEFAULT_GRIPPER_ACTION_TOPIC
    )
    parser.add_argument(
        "--gripper-feedback-valid-topic",
        default=DEFAULT_GRIPPER_FEEDBACK_VALID_TOPIC,
    )
    parser.add_argument(
        "--gripper-contact-topic", default=DEFAULT_GRIPPER_CONTACT_TOPIC
    )
    parser.add_argument("--head-topic", default=DEFAULT_HEAD_TOPIC)
    parser.add_argument("--wrist-topic", default=DEFAULT_WRIST_TOPIC)
    parser.add_argument("--max-state-skew-ms", type=float, default=75.0)
    parser.add_argument("--max-action-skew-ms", type=float, default=60.0)
    parser.add_argument("--max-camera-skew-ms", type=float, default=50.0)
    parser.add_argument("--max-discrete-age-ms", type=float, default=5000.0)
    parser.add_argument(
        "--max-camera-duplicate-ratio", type=float, default=0.05
    )
    parser.add_argument("--minimum-frames", type=int, default=30)
    # Offline export must apply backpressure. LeRobot's asynchronous image
    # writer uses an unbounded queue, so a fast rosbag decoder can otherwise
    # retain thousands of decoded 720p frames and be killed by the OOM killer.
    parser.add_argument("--image-writer-threads", type=int, default=0)
    return parser


def _positive_milliseconds(args: argparse.Namespace) -> None:
    for name in (
        "max_state_skew_ms",
        "max_action_skew_ms",
        "max_camera_skew_ms",
        "max_discrete_age_ms",
    ):
        if getattr(args, name) <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive")
    if not 0.0 <= args.max_camera_duplicate_ratio <= 1.0:
        raise SystemExit("--max-camera-duplicate-ratio must be within 0..1")
    if args.minimum_frames <= 0 or args.image_writer_threads < 0:
        raise SystemExit("minimum frames must be positive and threads nonnegative")


def main() -> int:
    args = build_parser().parse_args()
    _positive_milliseconds(args)
    episode_dir = args.episode_dir.expanduser().resolve()
    metadata = _read_episode_metadata(episode_dir)
    if metadata.get("status") != "complete":
        raise SystemExit("episode metadata status is not complete")
    outcome = str(metadata.get("outcome", "unknown"))
    if outcome != "success" and not args.allow_non_success:
        raise SystemExit(
            f"episode outcome is {outcome!r}; mark it success or use "
            "--allow-non-success for diagnostic exports"
        )
    task = args.task if args.task is not None else str(metadata.get("task", ""))
    if not task:
        raise SystemExit("episode has no language task")
    if task != task.strip():
        raise SystemExit("language task has leading or trailing whitespace")
    fps = args.fps or int(metadata.get("target_lerobot_fps", 30))
    if fps <= 0:
        raise SystemExit("FPS must be positive")
    dataset_root = args.dataset_root.expanduser().resolve()
    report_path = episode_dir / "lerobot_export_report.json"
    if not args.dry_run:
        completed_index = _completed_export_index(
            report_path,
            dataset_root,
            args.repo_id,
        )
        if completed_index is not None:
            print(
                f"Episode is already exported as dataset episode "
                f"{completed_index}; no duplicate was written."
            )
            print(f"Dataset: {dataset_root}")
            print(f"Quality report: {report_path}")
            return 0

    bag_dir = episode_dir / "rosbag"
    if not bag_dir.is_dir():
        raise SystemExit(f"ROS bag directory is missing: {bag_dir}")
    topics = {
        args.state_topic,
        args.action_topic,
        args.gripper_action_topic,
        args.gripper_feedback_valid_topic,
        args.gripper_contact_topic,
        args.head_topic,
        args.wrist_topic,
    }
    streams, message_store = _read_rosbag(
        bag_dir,
        str(metadata.get("storage", "sqlite3")),
        topics,
    )
    _validate_stream_type(
        streams[args.state_topic], {"sensor_msgs/msg/JointState"}
    )
    _validate_stream_type(
        streams[args.action_topic], {"sensor_msgs/msg/JointState"}
    )
    for topic in (
        args.gripper_action_topic,
        args.gripper_feedback_valid_topic,
        args.gripper_contact_topic,
    ):
        _validate_stream_type(streams[topic], {"std_msgs/msg/Bool"})
    camera_types = {
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/CompressedImage",
    }
    _validate_stream_type(streams[args.head_topic], camera_types)
    _validate_stream_type(streams[args.wrist_topic], camera_types)

    required_streams = list(streams.values())
    start_ns = max(stream.times_ns[0] for stream in required_streams)
    end_ns = min(stream.times_ns[-1] for stream in required_streams)
    grid = uniform_grid_ns(start_ns, end_ns, fps)
    if len(grid) < args.minimum_frames:
        raise SystemExit(
            f"only {len(grid)} synchronized frames are available; "
            f"minimum is {args.minimum_frames}"
        )

    state_limit = round(args.max_state_skew_ms * 1_000_000)
    action_limit = round(args.max_action_skew_ms * 1_000_000)
    camera_limit = round(args.max_camera_skew_ms * 1_000_000)
    discrete_limit = round(args.max_discrete_age_ms * 1_000_000)
    selectors: dict[str, list[SelectedSample]] = {
        topic: [] for topic in topics
    }
    selector_functions: dict[
        str, tuple[Callable[..., SelectedSample | None], int]
    ] = {
        args.state_topic: (nearest_sample, state_limit),
        # The action stored at frame t must already have been accepted by the
        # SDK. Never borrow a command carrying a later timestamp.
        args.action_topic: (previous_sample, action_limit),
        args.gripper_action_topic: (previous_sample, discrete_limit),
        args.gripper_feedback_valid_topic: (
            previous_sample,
            discrete_limit,
        ),
        args.gripper_contact_topic: (previous_sample, discrete_limit),
        args.head_topic: (nearest_sample, camera_limit),
        args.wrist_topic: (nearest_sample, camera_limit),
    }
    for frame_index, target_ns in enumerate(grid):
        for topic, (function, limit) in selector_functions.items():
            selected = function(streams[topic].times_ns, target_ns, limit)
            if selected is None:
                raise SystemExit(
                    f"frame {frame_index} at {target_ns} has no fresh sample "
                    f"for {topic}"
                )
            selectors[topic].append(selected)

    duplicate_ratios: dict[str, float] = {}
    for topic in (args.head_topic, args.wrist_topic):
        indices = [item.index for item in selectors[topic]]
        duplicates = sum(
            current == previous
            for previous, current in zip(indices, indices[1:], strict=False)
        )
        ratio = duplicates / max(len(indices) - 1, 1)
        duplicate_ratios[topic] = ratio
        if ratio > args.max_camera_duplicate_ratio:
            raise SystemExit(
                f"{topic} duplicate-frame ratio {ratio:.3%} exceeds "
                f"{args.max_camera_duplicate_ratio:.3%}"
            )

    first_head = _decode_image(
        message_store.load(streams[args.head_topic].values[
            selectors[args.head_topic][0].index
        ]),
        streams[args.head_topic].type_name,
    )
    first_wrist = _decode_image(
        message_store.load(streams[args.wrist_topic].values[
            selectors[args.wrist_topic][0].index
        ]),
        streams[args.wrist_topic].type_name,
    )
    if first_head.ndim != 3 or first_head.shape[2] != 3:
        raise SystemExit(f"invalid head RGB shape: {first_head.shape}")
    if first_wrist.ndim != 3 or first_wrist.shape[2] != 3:
        raise SystemExit(f"invalid wrist RGB shape: {first_wrist.shape}")
    features = _feature_schema(
        tuple(int(item) for item in first_head.shape),
        tuple(int(item) for item in first_wrist.shape),
    )
    quality = {
        "format": "one_arm_lerobot_export_report",
        "format_version": 1,
        "episode_directory": str(episode_dir),
        "dataset_root": str(args.dataset_root.expanduser().resolve()),
        "repo_id": args.repo_id,
        "task": task,
        "outcome": outcome,
        "fps": fps,
        "frames": len(grid),
        "duration_seconds": (grid[-1] - grid[0]) / 1_000_000_000,
        "timestamp_source": (
            "ROS message header when nonzero, otherwise rosbag receive timestamp"
        ),
        "gripper_convention": "0.0=open, 1.0=closed",
        "gripper_observation_source": (
            "last requested binary state; position feedback validity is stored "
            "separately because CTAG2F120 readback is not yet trustworthy"
        ),
        "camera_duplicate_ratio": duplicate_ratios,
        "sample_skew": {
            topic: _summarize_skews(
                [item.skew_ns for item in selected]
            )
            for topic, selected in selectors.items()
        },
        "status": "validated" if args.dry_run else "exporting",
    }
    if args.dry_run:
        _write_report(report_path, quality)
        message_store.close()
        print(
            f"Validated {len(grid)} frames at {fps} FPS; no dataset was written."
        )
        print(f"Quality report: {report_path}")
        return 0

    try:
        from lerobot.datasets import LeRobotDataset
    except ImportError as exc:
        raise SystemExit(
            "LeRobot is not installed. Install lerobot>=0.4.0 in the "
            "Ubuntu export environment."
        ) from exc

    info_path = dataset_root / "meta" / "info.json"
    if info_path.exists():
        dataset = LeRobotDataset.resume(
            repo_id=args.repo_id,
            root=dataset_root,
            image_writer_threads=args.image_writer_threads,
        )
        if int(dataset.fps) != fps:
            raise SystemExit(
                f"dataset FPS is {dataset.fps}, episode requests {fps}"
            )
        if not _schema_matches(dataset.features, features):
            raise SystemExit("episode feature schema does not match dataset")
    else:
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=fps,
            features=features,
            root=dataset_root,
            robot_type="armstrong_right_arm_ctag2f120",
            use_videos=True,
            image_writer_threads=args.image_writer_threads,
        )

    decode_cache: dict[str, tuple[int, np.ndarray] | None] = {
        args.head_topic: None,
        args.wrist_topic: None,
    }

    def camera_frame(topic: str, selected: SelectedSample) -> np.ndarray:
        cached = decode_cache[topic]
        if cached is not None and cached[0] == selected.index:
            return cached[1]
        image = _decode_image(
            message_store.load(streams[topic].values[selected.index]),
            streams[topic].type_name,
        )
        decode_cache[topic] = (selected.index, image)
        return image

    try:
        for frame_index in range(len(grid)):
            state_message = message_store.load(
                streams[args.state_topic].values[
                    selectors[args.state_topic][frame_index].index
                ]
            )
            action_message = message_store.load(
                streams[args.action_topic].values[
                    selectors[args.action_topic][frame_index].index
                ]
            )
            gripper_message = message_store.load(
                streams[args.gripper_action_topic].values[
                    selectors[args.gripper_action_topic][frame_index].index
                ]
            )
            feedback_message = message_store.load(
                streams[args.gripper_feedback_valid_topic].values[
                    selectors[args.gripper_feedback_valid_topic][
                        frame_index
                    ].index
                ]
            )
            contact_message = message_store.load(
                streams[args.gripper_contact_topic].values[
                    selectors[args.gripper_contact_topic][frame_index].index
                ]
            )
            gripper = _closed_value(gripper_message)
            state = np.concatenate(
                (_joint_positions(state_message), np.asarray([gripper]))
            ).astype(np.float32, copy=False)
            action = np.concatenate(
                (_joint_positions(action_message), np.asarray([gripper]))
            ).astype(np.float32, copy=False)
            head = camera_frame(
                args.head_topic, selectors[args.head_topic][frame_index]
            )
            wrist = camera_frame(
                args.wrist_topic,
                selectors[args.wrist_topic][frame_index],
            )
            if tuple(head.shape) != tuple(features[
                "observation.images.head"
            ]["shape"]):
                raise ValueError(
                    f"head image shape changed at frame {frame_index}: "
                    f"{head.shape}"
                )
            if tuple(wrist.shape) != tuple(features[
                "observation.images.wrist_right"
            ]["shape"]):
                raise ValueError(
                    f"wrist image shape changed at frame {frame_index}: "
                    f"{wrist.shape}"
                )
            dataset.add_frame(
                {
                    "task": task,
                    "observation.state": state,
                    "action": action,
                    "observation.gripper_contact": np.asarray(
                        [_bool_float(contact_message)], dtype=np.float32
                    ),
                    "observation.gripper_feedback_valid": np.asarray(
                        [_bool_float(feedback_message)], dtype=np.float32
                    ),
                    "observation.images.head": head,
                    "observation.images.wrist_right": wrist,
                }
            )
        dataset.save_episode()
        dataset.finalize()
    except Exception:
        if dataset.has_pending_frames():
            dataset.clear_episode_buffer(delete_images=True)
        dataset.finalize()
        message_store.close()
        raise

    message_store.close()

    quality["status"] = "complete"
    quality["dataset_episode_index"] = int(dataset.num_episodes) - 1
    _write_report(report_path, quality)
    print(
        f"Exported episode {quality['dataset_episode_index']} with "
        f"{len(grid)} frames at {fps} FPS."
    )
    print(f"Dataset: {dataset_root}")
    print(f"Quality report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
