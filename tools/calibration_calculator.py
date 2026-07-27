#!/usr/bin/env python3
"""Calculate one leader-to-follower joint sign and scale without moving a robot.

Take two leader encoder readings around a measured joint rotation. The signed
follower angle follows the robot controller/URDF convention. This program only
does arithmetic; it has no serial, network, ROS, or robot-control code.
"""

from __future__ import annotations

import argparse
import json
import math
import sys


def shortest_delta(start: float, end: float, period: float) -> float:
    delta = (end - start + period / 2.0) % period - period / 2.0
    if math.isclose(abs(delta), period / 2.0, abs_tol=1e-9):
        raise ValueError(
            "exactly half-period motion is direction-ambiguous; use about 90 degrees"
        )
    return delta


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compute q_delta = sign * scale_rad_per_pulse * leader_delta "
            "for one joint."
        )
    )
    parser.add_argument("--joint", type=int, choices=range(1, 8), required=True)
    parser.add_argument("--start-pulse", type=float, required=True)
    parser.add_argument("--end-pulse", type=float, required=True)
    parser.add_argument(
        "--follower-delta-deg",
        type=float,
        required=True,
        help=(
            "signed desired follower change in degrees according to the "
            "right-arm controller/URDF coordinate"
        ),
    )
    parser.add_argument("--period-pulses", type=float, default=2500.0)
    parser.add_argument("--json", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not math.isfinite(args.period_pulses) or args.period_pulses <= 0:
        print("ERROR: --period-pulses must be positive", file=sys.stderr)
        return 2
    try:
        pulse_delta = shortest_delta(
            args.start_pulse,
            args.end_pulse,
            args.period_pulses,
        )
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    if abs(pulse_delta) < 20:
        print(
            "ERROR: encoder change is under 20 pulses; use a larger measured "
            "rotation (about 90 degrees is recommended)",
            file=sys.stderr,
        )
        return 2
    angle_rad = math.radians(args.follower_delta_deg)
    if not math.isfinite(angle_rad) or abs(angle_rad) < math.radians(1.0):
        print(
            "ERROR: desired follower change must be at least 1 degree",
            file=sys.stderr,
        )
        return 2

    signed_ratio = angle_rad / pulse_delta
    result = {
        "joint": args.joint,
        "leader_pulse_delta": pulse_delta,
        "desired_follower_delta_deg": args.follower_delta_deg,
        "sign": 1 if signed_ratio > 0 else -1,
        "scale_rad_per_pulse": abs(signed_ratio),
    }
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(f"joint_{args.joint}")
        print(f"  leader pulse delta: {pulse_delta:.6f}")
        print(
            "  desired follower delta: "
            f"{args.follower_delta_deg:.6f} deg ({angle_rad:.9f} rad)"
        )
        print(f"  sign: {result['sign']:+d}")
        print(
            "  scale_rad_per_pulse: "
            f"{result['scale_rad_per_pulse']:.12f}"
        )
        print()
        print(
            "YAML values for this joint: "
            f"sign={result['sign']:+d}, "
            f"scale={result['scale_rad_per_pulse']:.12f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
