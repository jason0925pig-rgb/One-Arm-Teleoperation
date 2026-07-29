#!/usr/bin/env bash
# Read-only preflight: report processes/nodes/serial owners that could conflict
# with the one-arm teleoperation stack. This script never kills or starts anything.

set -u

conflicts=0
gripper_device="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5ABB000800-if00"

echo "=== One-arm teleoperation conflict preflight (read-only) ==="
echo "host=$(hostname) time=$(date --iso-8601=seconds)"

echo
echo "[1/4] Known robot, teleoperation, gripper, and policy processes"
process_matches="$(
  pgrep -af 'robot_timer|servo_control|test_joint_trajectory_sub|gripper_controller|safe_one_arm_servo|safe_gripper_controller|udp_leader_bridge|[p]i0' 2>/dev/null || true
)"
if [[ -n "${process_matches}" ]]; then
  printf '%s\n' "${process_matches}"
  conflicts=1
else
  echo "none found"
fi

echo
echo "[2/4] ROS 2 nodes"
if command -v ros2 >/dev/null 2>&1; then
  node_list="$(ros2 node list 2>/dev/null || true)"
  if [[ -n "${node_list}" ]]; then
    printf '%s\n' "${node_list}"
    if printf '%s\n' "${node_list}" | grep -Eq \
      '/(robot_timer|gripper_controller|safe_one_arm_servo|safe_gripper_controller|udp_leader_bridge)$'; then
      conflicts=1
    fi
  else
    echo "none visible"
  fi
else
  echo "ros2 command not found; source ROS and the workspace, then rerun"
fi

echo
echo "[3/4] Publishers on motion/gripper command topics"
if command -v ros2 >/dev/null 2>&1; then
  for topic in \
    /right_arm/teleop_joint_command \
    /right_arm/joint_control \
    /right_arm/hand_control \
    /right_arm/gripper_command \
    /gripper_position; do
    info="$(ros2 topic info "${topic}" 2>/dev/null || true)"
    count="$(printf '%s\n' "${info}" | sed -n 's/^Publisher count: //p')"
    count="${count:-0}"
    echo "${topic}: publishers=${count}"
    if [[ "${count}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
      conflicts=1
    fi
  done
else
  echo "skipped because ros2 is not available in this shell"
fi

echo
echo "[4/4] CTAG2F120 serial identity and current owners"
if [[ -e "${gripper_device}" ]]; then
  readlink -f "${gripper_device}"
  if command -v fuser >/dev/null 2>&1; then
    owners="$(fuser "${gripper_device}" 2>/dev/null || true)"
    if [[ -n "${owners//[[:space:]]/}" ]]; then
      echo "serial device owner PID(s): ${owners}"
      ps -fp ${owners} 2>/dev/null || true
      conflicts=1
    else
      echo "no serial owner reported"
    fi
  else
    echo "fuser is unavailable; serial ownership was not checked"
  fi
else
  echo "stable serial path is not currently present: ${gripper_device}"
fi

echo
if (( conflicts > 0 )); then
  echo "RESULT: CONFLICT/ACTIVITY FOUND. Do not start another controller."
  echo "This report is informational; nothing was stopped or changed."
  exit 3
fi

echo "RESULT: no known conflict found by this preflight."
echo "This is not motion authorization; all safety gates must still pass."
