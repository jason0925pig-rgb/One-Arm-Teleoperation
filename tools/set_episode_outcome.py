#!/usr/bin/env python3
"""Safely annotate a raw episode after the operator reviews it."""

from __future__ import annotations

import argparse
import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def write_metadata(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary.replace(path)


def recover_closed_sqlite_recording(
    episode_dir: Path,
    payload: dict[str, Any],
) -> None:
    """Recover metadata when rosbag closed but its supervisor was killed."""
    if payload.get("status") != "recording":
        return
    if payload.get("storage", "sqlite3") != "sqlite3":
        raise SystemExit(
            "automatic recovery currently supports sqlite3 ROS bags only"
        )
    bag_dir = episode_dir / "rosbag"
    rosbag_metadata = bag_dir / "metadata.yaml"
    databases = sorted(bag_dir.glob("*.db3"))
    if not rosbag_metadata.is_file() or not databases:
        raise SystemExit(
            "episode is still marked recording and has no closed ROS bag "
            "metadata/database to recover"
        )

    first_timestamp: int | None = None
    last_timestamp: int | None = None
    message_count = 0
    for database in databases:
        uri = database.resolve().as_uri() + "?mode=ro"
        try:
            connection = sqlite3.connect(uri, uri=True, timeout=2.0)
            try:
                quick_check = connection.execute(
                    "PRAGMA quick_check"
                ).fetchone()
                if quick_check is None or quick_check[0] != "ok":
                    raise SystemExit(
                        f"ROS bag integrity check failed for {database}: "
                        f"{quick_check}"
                    )
                row = connection.execute(
                    "SELECT MIN(timestamp), MAX(timestamp), COUNT(*) "
                    "FROM messages"
                ).fetchone()
            finally:
                connection.close()
        except sqlite3.Error as exc:
            raise SystemExit(
                f"cannot validate closed ROS bag database {database}: {exc}"
            ) from exc
        assert row is not None
        current_first, current_last, current_count = row
        if current_first is not None:
            first_timestamp = (
                int(current_first)
                if first_timestamp is None
                else min(first_timestamp, int(current_first))
            )
        if current_last is not None:
            last_timestamp = (
                int(current_last)
                if last_timestamp is None
                else max(last_timestamp, int(current_last))
            )
        message_count += int(current_count)

    if (
        first_timestamp is None
        or last_timestamp is None
        or message_count <= 0
    ):
        raise SystemExit("closed ROS bag contains no messages")

    payload["status"] = "complete"
    payload["ended_utc"] = datetime.fromtimestamp(
        last_timestamp / 1_000_000_000,
        timezone.utc,
    ).isoformat()
    payload["duration_seconds"] = (
        last_timestamp - first_timestamp
    ) / 1_000_000_000
    payload["stop_reason"] = "recovered_closed_rosbag"
    payload["recovered_utc"] = datetime.now(timezone.utc).isoformat()
    payload["recovered_message_count"] = message_count
    payload["rosbag_return_code"] = None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode_dir", type=Path)
    parser.add_argument(
        "--outcome",
        required=True,
        choices=("success", "failure"),
    )
    parser.add_argument("--notes")
    parser.add_argument(
        "--recover-closed-recording",
        action="store_true",
        help=(
            "if metadata is stuck at recording, require a closed, valid "
            "sqlite3 ROS bag and reconstruct completion metadata"
        ),
    )
    args = parser.parse_args()

    metadata_path = (
        args.episode_dir.expanduser().resolve() / "episode_metadata.json"
    )
    if not metadata_path.is_file():
        raise SystemExit(f"metadata is missing: {metadata_path}")
    episode_dir = args.episode_dir.expanduser().resolve()
    payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    if args.recover_closed_recording:
        recover_closed_sqlite_recording(episode_dir, payload)
    if payload.get("status") != "complete":
        raise SystemExit("only a completed episode may be reviewed")
    payload["outcome"] = args.outcome
    payload["reviewed_utc"] = datetime.now(timezone.utc).isoformat()
    if args.notes is not None:
        payload["review_notes"] = args.notes
    write_metadata(metadata_path, payload)
    print(f"Episode outcome set to {args.outcome}: {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
