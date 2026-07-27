#!/usr/bin/env bash
# Read-only Ubuntu/ROS2/hardware inventory for the first integration session.
set -u

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

section() {
  echo
  echo "===== $1 ====="
}

section "SYSTEM"
date --iso-8601=seconds 2>/dev/null || date
uname -a
if command -v lsb_release >/dev/null 2>&1; then
  lsb_release -a 2>/dev/null
else
  sed -n '1,12p' /etc/os-release 2>/dev/null
fi

section "ROS2"
echo "ROS_DISTRO=${ROS_DISTRO:-UNSET}"
echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-UNSET}"
for command_name in ros2 colcon xacro python3 cmake g++; do
  if command -v "${command_name}" >/dev/null 2>&1; then
    echo "${command_name}: $(command -v "${command_name}")"
  else
    echo "${command_name}: MISSING"
  fi
done

section "CPU AND JAKA SDK"
echo "machine=$(uname -m)"
find "${PROJECT_ROOT}/servo_controller/lib" -maxdepth 2 -type f \
  \( -name 'libjakaAPI.so' -o -name 'libjakaAPI.a' \) -print 2>/dev/null

section "NETWORK"
ip -brief address 2>/dev/null || true
echo
ip route 2>/dev/null || true
echo
echo "Configured right-arm controller candidate: 192.168.2.226:10020"
if command -v ping >/dev/null 2>&1; then
  ping -c 2 -W 1 192.168.2.226 2>&1 || true
fi

section "SERIAL AND USB"
if [ -d /dev/serial/by-id ]; then
  ls -l /dev/serial/by-id 2>/dev/null || true
else
  echo "/dev/serial/by-id is absent"
fi
echo
if command -v lsusb >/dev/null 2>&1; then
  lsusb 2>/dev/null || true
else
  echo "lsusb is missing"
fi

section "CAMERAS"
find /dev -maxdepth 1 -type c -name 'video*' -print 2>/dev/null || true

section "ROBOT DESCRIPTION CANDIDATES IN REPOSITORY"
find "${PROJECT_ROOT}" -type f \
  \( -name '*.urdf' -o -name '*.urdf.xacro' -o -name '*.xacro' \) \
  -print 2>/dev/null | head -n 100

section "ROBOT DESCRIPTION CANDIDATES IN HOME"
find "${HOME}" -maxdepth 7 -type f \
  \( -name '*.urdf' -o -name '*.urdf.xacro' -o -name '*.xacro' \) \
  -print 2>/dev/null | head -n 100

section "PROJECT BUILD STATE"
if command -v ros2 >/dev/null 2>&1; then
  ros2 pkg prefix servo_controller 2>&1 || true
  ros2 pkg prefix one_arm_teleop_bridge 2>&1 || true
fi

section "END"
echo "This script performed inventory and ping checks only."
echo "It did not power, enable, initialize, or command the robot or gripper."
