#!/usr/bin/env python3
"""Read a Windows DirectInput/WinMM gamepad and command the Orin over UDP."""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import socket
import time
import uuid
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np


JOY_RETURNALL = 0xFF
JOYERR_NOERROR = 0


class JOYINFOEX(ctypes.Structure):
    _fields_ = [(name, ctypes.c_uint32) for name in (
        "dwSize", "dwFlags", "dwXpos", "dwYpos", "dwZpos", "dwRpos",
        "dwUpos", "dwVpos", "dwButtons", "dwButtonNumber", "dwPOV",
        "dwReserved1", "dwReserved2"
    )]


def gamepad_state(device_id: int) -> JOYINFOEX:
    info = JOYINFOEX()
    info.dwSize = ctypes.sizeof(info)
    info.dwFlags = JOY_RETURNALL
    result = ctypes.windll.winmm.joyGetPosEx(device_id, ctypes.byref(info))
    if result != JOYERR_NOERROR:
        raise RuntimeError(f"Windows gamepad {device_id} is unavailable (joyGetPosEx={result})")
    return info


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]], dtype=np.float64)


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([[c+x*x*C, x*y*C-z*s, x*z*C+y*s],
                     [y*x*C+z*s, c+y*y*C, y*z*C-x*s],
                     [z*x*C-y*s, z*y*C+x*s, c+z*z*C]])


def transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = rpy_matrix(rpy)
    out[:3, 3] = xyz
    return out


def so3_log(rotation: np.ndarray) -> np.ndarray:
    cos_theta = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    theta = math.acos(cos_theta)
    vector = np.array([rotation[2,1]-rotation[1,2], rotation[0,2]-rotation[2,0], rotation[1,0]-rotation[0,1]])
    if theta < 1e-8:
        return 0.5 * vector
    return theta / (2.0 * math.sin(theta)) * vector


@dataclass
class ChainJoint:
    name: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class UrdfKinematics:
    def __init__(self, filename: Path, base: str, tool: str) -> None:
        root = ET.parse(filename).getroot()
        by_child = {}
        for element in root.findall("joint"):
            origin = element.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            axis_element = element.find("axis")
            axis = np.fromstring(axis_element.get("xyz", "0 0 1") if axis_element is not None else "0 0 1", sep=" ")
            limit = element.find("limit")
            lower = float(limit.get("lower", "-6.2832")) if limit is not None else -math.inf
            upper = float(limit.get("upper", "6.2832")) if limit is not None else math.inf
            by_child[element.find("child").get("link")] = (
                element.find("parent").get("link"),
                ChainJoint(element.get("name"), element.get("type"), transform(xyz, rpy), axis, lower, upper),
            )
        reverse = []
        link = tool
        while link != base:
            if link not in by_child:
                raise ValueError(f"No URDF chain from {base} to {tool}; stopped at {link}")
            parent, joint = by_child[link]
            reverse.append(joint)
            link = parent
        self.chain = list(reversed(reverse))
        self.active = [joint for joint in self.chain if joint.kind in ("revolute", "continuous")]
        if [joint.name for joint in self.active] != [f"r-j{i}" for i in range(1, 8)]:
            raise ValueError(f"Unexpected active chain: {[joint.name for joint in self.active]}")
        self.lower = np.array([joint.lower for joint in self.active])
        self.upper = np.array([joint.upper for joint in self.active])

    def fk_jacobian(self, q: np.ndarray):
        world = np.eye(4)
        joint_points, joint_axes = [], []
        active_index = 0
        for joint in self.chain:
            world = world @ joint.origin
            if joint.kind in ("revolute", "continuous"):
                joint_points.append(world[:3, 3].copy())
                joint_axes.append(world[:3, :3] @ joint.axis)
                motion = np.eye(4)
                motion[:3, :3] = axis_rotation(joint.axis, q[active_index])
                world = world @ motion
                active_index += 1
        point = world[:3, 3]
        jacobian = np.zeros((6, 7))
        for i, (origin, axis) in enumerate(zip(joint_points, joint_axes)):
            jacobian[:3, i] = np.cross(axis, point - origin)
            jacobian[3:, i] = axis
        return world, jacobian

    def solve(self, target: np.ndarray, seed: np.ndarray) -> tuple[bool, np.ndarray]:
        q = seed.copy()
        for _ in range(50):
            current, jacobian = self.fk_jacobian(q)
            error = np.concatenate((target[:3, 3] - current[:3, 3], so3_log(target[:3, :3] @ current[:3, :3].T)))
            if np.linalg.norm(error[:3]) < 0.0002 and np.linalg.norm(error[3:]) < math.radians(0.1):
                return True, q
            damping = 1e-3
            dq = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(6), error)
            norm = np.max(np.abs(dq))
            if norm > 0.08:
                dq *= 0.08 / norm
            q = np.clip(q + dq, self.lower, self.upper)
        return False, seed


def normalized(value: int) -> float:
    return float(np.clip((value - 32767.0) / 32767.0, -1.0, 1.0))


def pov_axes(value: int) -> tuple[float, float]:
    if value == 65535:
        return 0.0, 0.0
    angle = math.radians(value / 100.0)
    return math.sin(angle), -math.cos(angle)


def logical_axes(info: JOYINFOEX) -> np.ndarray:
    pov_x, pov_y = pov_axes(info.dwPOV)
    # Reconstruct the legacy Linux Joy layout: LX, LY, RX, RY, DPadX, DPadY.
    return np.array([normalized(info.dwXpos), normalized(info.dwYpos), normalized(info.dwZpos),
                     normalized(info.dwRpos), pov_x, pov_y], dtype=np.float64)


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", default="192.168.2.170:5010")
    parser.add_argument("--bind-host", default="192.168.2.131")
    parser.add_argument("--device-id", type=int, default=0)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--probe-only", action="store_true")
    parser.add_argument("--rate-hz", type=float, default=31.25)
    parser.add_argument(
        "--servo-ready-file",
        default="",
        help="Local readiness marker written by the parent after Orin servo startup.",
    )
    args = parser.parse_args()
    first = gamepad_state(args.device_id)
    kinematics = UrdfKinematics(Path(args.urdf), "base_link_jaka_right", "rt")
    print(f"WINDOWS_GAMEPAD_READY id={args.device_id} axes={logical_axes(first).round(3).tolist()} buttons=0x{first.dwButtons:X} pov={first.dwPOV}", flush=True)
    if args.probe_only:
        return
    target_host, target_port = args.target.rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind_host, 0))
    sock.setblocking(False)
    target = (target_host, int(target_port))
    session = f"windows_gamepad_{uuid.uuid4().hex[:12]}"
    sequence = 0
    current_q = None
    command_q = None
    target_pose = None
    # Three deliberate attended stages:
    #   1) first B4 captures an FK baseline only;
    #   2) second B4 asks the parent launcher to enter servo;
    #   3) third B4 stops the round.
    initialized = False
    active = False
    servo_ready = False
    servo_ready_file = Path(args.servo_ready_file) if args.servo_ready_file else None
    gripper_open = True
    previous_buttons = 0
    period = 1.0 / args.rate_hz
    previous_time = time.monotonic()
    last_input_report = 0.0
    last_reported_axes: tuple[float, ...] | None = None

    def send(kind: str, positions=None):
        nonlocal sequence
        sequence += 1
        packet = {"protocol":"onearm_gamepad_v1", "type":kind, "sequence":sequence,
                  "wall_time":time.time(), "session":session}
        if positions is not None:
            packet.update(positions=[float(v) for v in positions], gripper_open=gripper_open)
        sock.sendto(json.dumps(packet, separators=(",", ":")).encode(), target)

    try:
        while True:
            started = time.monotonic()
            info = gamepad_state(args.device_id)
            while True:
                try:
                    data, _ = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                try:
                    packet = json.loads(data.decode())
                    positions = packet.get("positions")
                    if packet.get("type") == "state" and isinstance(positions, list) and len(positions) == 7:
                        current_q = np.asarray(positions, dtype=np.float64)
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
                    pass
            buttons = int(info.dwButtons)
            start_edge = bool(buttons & (1 << 4)) and not bool(previous_buttons & (1 << 4))
            grip_edge = bool(buttons & (1 << 5)) and not bool(previous_buttons & (1 << 5))
            previous_buttons = buttons
            axes = np.array([apply_deadzone(v, 0.15) for v in logical_axes(info)])
            if initialized and not servo_ready and servo_ready_file is not None and servo_ready_file.exists():
                servo_ready = True
                print("GAMEPAD_SERVO_READY: press button[4] to begin teleoperation.", flush=True)
            if start_edge:
                if active:
                    send("stop")
                    print("GAMEPAD_ROUND_FINISHED", flush=True)
                    return
                if current_q is None:
                    print("WAITING_FOR_ORIN_JOINT_STATE", flush=True)
                elif np.any(axes):
                    print("GAMEPAD_INIT_REQUIRES_CENTERED_AXES", flush=True)
                elif not initialized:
                    command_q = current_q.copy()
                    target_pose, _ = kinematics.fk_jacobian(command_q)
                    # Run the solver once at the captured pose.  It validates
                    # the current FK/IK chain before servo is armed, rather
                    # than making the first operator motion pay this setup.
                    solved_ok, solved_q = kinematics.solve(target_pose, command_q)
                    if not solved_ok:
                        print("GAMEPAD_INIT_IK_FAILED", flush=True)
                        continue
                    command_q = solved_q
                    initialized = True
                    previous_time = time.monotonic()
                    print("GAMEPAD_INITIALIZED: baseline and IK check complete; waiting for servo preparation.", flush=True)
                elif not servo_ready:
                    print("GAMEPAD_WAITING_FOR_SERVO_READY", flush=True)
                else:
                    active = True
                    previous_time = time.monotonic()
                    print("GAMEPAD_TELEOP_START_REQUESTED", flush=True)
            if grip_edge and active:
                gripper_open = not gripper_open
                print("GAMEPAD_GRIPPER=" + ("OPEN" if gripper_open else "CLOSED"), flush=True)
            if active:
                now_for_report = time.monotonic()
                report_axes = tuple(float(v) for v in axes.round(3))
                if (report_axes != last_reported_axes or
                        now_for_report - last_input_report >= 0.5):
                    print(
                        "GAMEPAD_INPUT "
                        f"LX={report_axes[0]:+.3f} LY={report_axes[1]:+.3f} "
                        f"RX={report_axes[2]:+.3f} RY={report_axes[3]:+.3f} "
                        f"DPAD_X={report_axes[4]:+.3f} DPAD_Y={report_axes[5]:+.3f} "
                        f"buttons=0x{buttons:X}",
                        flush=True,
                    )
                    last_input_report = now_for_report
                    last_reported_axes = report_axes
                now = time.monotonic()
                elapsed = min(max(now - previous_time, 0.0), 2.0 * period)
                previous_time = now
                # Operator-facing base-frame mapping, confirmed on the
                    # physical arm: left stick forward/back is robot
                    # forward/back; left stick right/left is robot
                    # right/left; right-stick vertical is up/down.
                    # The previous mapping was rotated by 90 degrees in the
                    # table plane and inverted the vertical direction.
                translation = np.array([axes[0], -axes[3], axes[1]])
                # Physical D-pad calibration: the original X/Y assignment
                # made Up/Down tilt left/right and Left/Right tilt down/up.
                # Map D-pad Up/Down to the physical up/down tilt channel and
                # D-pad Left/Right to the physical left/right channel.
                rotation = np.array([axes[5], -axes[4], axes[2]])
                if np.any(translation) or np.any(rotation):
                    candidate = target_pose.copy()
                    candidate[:3, 3] += translation * 0.03 * elapsed
                    candidate[:3, :3] = target_pose[:3, :3] @ axis_rotation(
                        rotation if np.linalg.norm(rotation) else np.array([1.,0.,0.]),
                        np.linalg.norm(rotation) * math.radians(20.0) * elapsed,
                    )
                    success, solved = kinematics.solve(candidate, command_q)
                    if success:
                        command_q, target_pose = solved, candidate
                send("command", command_q)
            else:
                send("hello")
            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)
    except (KeyboardInterrupt, OSError, RuntimeError) as exc:
        try:
            send("stop")
        except OSError:
            pass
        print(f"GAMEPAD_SENDER_STOPPED: {exc}", flush=True)
        raise SystemExit(1)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
