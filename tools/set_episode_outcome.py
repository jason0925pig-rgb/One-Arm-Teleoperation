#!/usr/bin/env python3
"""Safely annotate a raw episode after the operator reviews it."""

from __future__ import annotations

import argparse
import json
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode_dir", type=Path)
    parser.add_argument(
        "--outcome",
        required=True,
        choices=("success", "failure"),
    )
    parser.add_argument("--notes")
    args = parser.parse_args()

    metadata_path = (
        args.episode_dir.expanduser().resolve() / "episode_metadata.json"
    )
    if not metadata_path.is_file():
        raise SystemExit(f"metadata is missing: {metadata_path}")
    payload = json.loads(metadata_path.read_text(encoding="utf-8"))
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
