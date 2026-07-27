#!/usr/bin/env python3
"""Extract movable-joint limits from a fully expanded URDF.

This tool is read-only. It does not import ROS, connect to a controller, or send
motion commands. Xacro files must first be expanded with the ROS `xacro` tool.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


MOVABLE_TYPES = {"revolute", "continuous", "prismatic"}


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def finite_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def load_joints(path: Path) -> list[dict[str, Any]]:
    text = path.read_text(encoding="utf-8")
    if "${" in text or "$(" in text:
        raise ValueError(
            "file still contains xacro substitutions; run: "
            "xacro input.xacro > expanded.urdf"
        )
    root = ET.fromstring(text)
    result: list[dict[str, Any]] = []
    for element in root.iter():
        if local_name(element.tag) != "joint":
            continue
        joint_type = element.attrib.get("type", "")
        if joint_type not in MOVABLE_TYPES:
            continue
        name = element.attrib.get("name", "")
        limit = next(
            (
                child
                for child in element
                if local_name(child.tag) == "limit"
            ),
            None,
        )
        attributes = limit.attrib if limit is not None else {}
        result.append(
            {
                "name": name,
                "type": joint_type,
                "lower": finite_float(attributes.get("lower")),
                "upper": finite_float(attributes.get("upper")),
                "velocity": finite_float(attributes.get("velocity")),
                "effort": finite_float(attributes.get("effort")),
            }
        )
    return result


def select_joints(
    joints: list[dict[str, Any]],
    exact_names: list[str],
    name_regex: str | None,
) -> list[dict[str, Any]]:
    if exact_names:
        lookup = {joint["name"]: joint for joint in joints}
        missing = [name for name in exact_names if name not in lookup]
        if missing:
            raise ValueError("joint names not found: " + ", ".join(missing))
        return [lookup[name] for name in exact_names]
    if name_regex:
        pattern = re.compile(name_regex)
        return [joint for joint in joints if pattern.search(joint["name"])]
    return joints


def format_number(value: float | None) -> str:
    return "MISSING" if value is None else f"{value:.9g}"


def print_report(
    path: Path,
    joints: list[dict[str, Any]],
    margin_deg: float,
) -> None:
    print(f"URDF: {path.resolve()}")
    print(f"Selected movable joints: {len(joints)}")
    print()
    print(
        f"{'#':>2}  {'joint':<32} {'type':<10} "
        f"{'lower(rad)':>12} {'upper(rad)':>12} {'vel(rad/s)':>12}"
    )
    for index, joint in enumerate(joints, start=1):
        print(
            f"{index:>2}  {joint['name']:<32} {joint['type']:<10} "
            f"{format_number(joint['lower']):>12} "
            f"{format_number(joint['upper']):>12} "
            f"{format_number(joint['velocity']):>12}"
        )

    complete = (
        len(joints) == 7
        and all(
            joint["type"] == "revolute"
            and joint["lower"] is not None
            and joint["upper"] is not None
            and joint["lower"] < joint["upper"]
            for joint in joints
        )
    )
    print()
    if not complete:
        print(
            "RESULT: INCOMPLETE. Do not copy these values into the motion "
            "configuration yet."
        )
        print(
            "Select exactly the seven right-arm revolute joints in controller "
            "order with repeated --joint arguments."
        )
        return

    print("RESULT: seven finite revolute-joint limits found.")
    print("Copy candidate (verify names/order against the controller first):")
    print(
        "joint_names: ["
        + ", ".join(f'"{joint["name"]}"' for joint in joints)
        + "]"
    )
    print(
        "joint_lower_limits: ["
        + ", ".join(format_number(joint["lower"]) for joint in joints)
        + "]"
    )
    print(
        "joint_upper_limits: ["
        + ", ".join(format_number(joint["upper"]) for joint in joints)
        + "]"
    )
    margin_rad = math.radians(margin_deg)
    safe_lower = [joint["lower"] + margin_rad for joint in joints]
    safe_upper = [joint["upper"] - margin_rad for joint in joints]
    if all(lower < upper for lower, upper in zip(safe_lower, safe_upper)):
        print(f"# Candidate first-test software limits with {margin_deg:g} deg margin:")
        print(
            "candidate_safe_lower_limits: ["
            + ", ".join(format_number(value) for value in safe_lower)
            + "]"
        )
        print(
            "candidate_safe_upper_limits: ["
            + ", ".join(format_number(value) for value in safe_upper)
            + "]"
        )
    print(
        "urdf_velocity_limits: ["
        + ", ".join(format_number(joint["velocity"]) for joint in joints)
        + "]"
    )
    print(
        "NOTE: URDF velocity limits are manufacturer maxima, not the initial "
        "teleoperation speed. Keep the initial 0.10 rad/s cap."
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read seven right-arm limits from an expanded URDF."
    )
    parser.add_argument("urdf", type=Path)
    parser.add_argument(
        "--joint",
        action="append",
        default=[],
        help="exact joint name, repeated in desired controller order",
    )
    parser.add_argument(
        "--name-regex",
        help="list only names matching this regular expression",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="print selected joint records as JSON instead of a table",
    )
    parser.add_argument(
        "--margin-deg",
        type=float,
        default=5.0,
        help="inward margin for candidate first-test software limits",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        joints = select_joints(
            load_joints(args.urdf),
            args.joint,
            args.name_regex,
        )
    except (OSError, ET.ParseError, ValueError, re.error) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    if not joints:
        print("ERROR: no matching movable joints found", file=sys.stderr)
        return 3
    if (
        not math.isfinite(args.margin_deg)
        or args.margin_deg < 0.0
        or args.margin_deg > 45.0
    ):
        print("ERROR: --margin-deg must be within 0..45", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(joints, ensure_ascii=False, indent=2))
    else:
        print_report(args.urdf, joints, args.margin_deg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
