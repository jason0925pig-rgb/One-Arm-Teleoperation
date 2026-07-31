#!/usr/bin/env bash
# One-time Ubuntu setup for offline ROS bag -> LeRobot Dataset v3 export.

set -Eeo pipefail

VENV="${ONE_ARM_LEROBOT_VENV:-/home/tele/.venvs/onearm-lerobot}"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
LEROBOT_VERSION="${ONE_ARM_LEROBOT_VERSION:-0.6.0}"
TORCH_VERSION="${ONE_ARM_TORCH_VERSION:-2.7.1}"
TORCHVISION_VERSION="${ONE_ARM_TORCHVISION_VERSION:-0.22.1}"
PYPI_INDEX="${ONE_ARM_PYPI_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"

[[ -r "${ROS_SETUP}" ]] || {
  echo "ERROR: ROS setup is missing: ${ROS_SETUP}" >&2
  exit 2
}
# shellcheck disable=SC1090
source "${ROS_SETUP}"

if [[ ! -x "${VENV}/bin/python" ]]; then
  python3 -m venv --system-site-packages "${VENV}"
fi
"${VENV}/bin/python" -m pip install --index-url "${PYPI_INDEX}" --upgrade pip
"${VENV}/bin/python" -m pip install \
  --index-url https://download.pytorch.org/whl/cpu \
  "torch==${TORCH_VERSION}+cpu" \
  "torchvision==${TORCHVISION_VERSION}+cpu"
"${VENV}/bin/python" -m pip install \
  --index-url "${PYPI_INDEX}" \
  "lerobot[dataset]==${LEROBOT_VERSION}"
"${VENV}/bin/python" - <<'PY'
import cv2
import lerobot
import pyarrow
import rclpy
import rosbag2_py

print(f"LeRobot environment ready: {lerobot.__version__}")
PY
echo "LEROBOT_VENV_READY=${VENV}"
