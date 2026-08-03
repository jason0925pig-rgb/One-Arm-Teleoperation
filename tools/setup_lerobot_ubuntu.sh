#!/usr/bin/env bash
# One-time Ubuntu setup for offline ROS bag -> LeRobot Dataset v3 export.

set -Eeo pipefail

VENV="${ONE_ARM_LEROBOT_VENV:-${HOME}/.venvs/onearm-lerobot}"
if [[ -n "${ROS_DISTRO:-}" ]]; then
  ROS_DISTRO_NAME="${ROS_DISTRO}"
elif [[ -r /opt/ros/humble/setup.bash ]]; then
  ROS_DISTRO_NAME="humble"
else
  ROS_DISTRO_NAME="jazzy"
fi
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
LEROBOT_VERSION="${ONE_ARM_LEROBOT_VERSION:-0.6.0}"
PYPI_INDEX="${ONE_ARM_PYPI_INDEX:-https://pypi.org/simple}"
ARCHITECTURE="$(uname -m)"

if [[ "${ARCHITECTURE}" == "aarch64" || "${ARCHITECTURE}" == "arm64" ]]; then
  # LeRobot 0.6 publishes its Linux ARM64 TorchCodec dependency only from
  # 0.11 onward; that wheel requires Torch 2.11.  This isolated environment
  # is for dataset export.  SmolVLA GPU inference should use a separate,
  # JetPack-compatible NVIDIA PyTorch/container environment.
  TORCH_VERSION="${ONE_ARM_TORCH_VERSION:-2.11.0}"
  TORCHVISION_VERSION="${ONE_ARM_TORCHVISION_VERSION:-0.26.0}"
  TORCHCODEC_VERSION="${ONE_ARM_TORCHCODEC_VERSION:-0.11.1}"
  # The regular PyPI ARM64 wheel pulls a full CUDA 13 toolkit, which is both
  # unnecessary for export and incompatible with this JetPack 6.0 host.
  # PyTorch's CPU index provides native ARM64 +cpu wheels without that stack.
  TORCH_INDEX="${ONE_ARM_TORCH_INDEX:-https://download.pytorch.org/whl/cpu}"
  TORCH_SPEC="torch==${TORCH_VERSION}+cpu"
  TORCHVISION_SPEC="torchvision==${TORCHVISION_VERSION}+cpu"
else
  TORCH_VERSION="${ONE_ARM_TORCH_VERSION:-2.8.0}"
  TORCHVISION_VERSION="${ONE_ARM_TORCHVISION_VERSION:-0.23.0}"
  TORCHCODEC_VERSION="${ONE_ARM_TORCHCODEC_VERSION:-0.7.0}"
  TORCH_INDEX="${ONE_ARM_TORCH_INDEX:-https://download.pytorch.org/whl/cpu}"
  TORCH_SPEC="torch==${TORCH_VERSION}+cpu"
  TORCHVISION_SPEC="torchvision==${TORCHVISION_VERSION}+cpu"
fi

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
  --index-url "${TORCH_INDEX}" \
  "${TORCH_SPEC}" \
  "${TORCHVISION_SPEC}"
"${VENV}/bin/python" -m pip install \
  --index-url "${PYPI_INDEX}" \
  "lerobot[dataset]==${LEROBOT_VERSION}"
"${VENV}/bin/python" -m pip install \
  --index-url "${PYPI_INDEX}" \
  "pandas>=2.2,<3" \
  "scipy>=1.13,<2" \
  "setuptools>=71,<80" \
  "numexpr>=2.10,<3" \
  "bottleneck>=1.4,<2" \
  "torchcodec==${TORCHCODEC_VERSION}"
"${VENV}/bin/python" - <<'PY'
import cv2
import lerobot
import pandas
import pyarrow
import rclpy
import rosbag2_py
import torch
import torchcodec
from lerobot.datasets import LeRobotDataset

print(
    "LeRobot environment ready: "
    f"lerobot={lerobot.__version__} pandas={pandas.__version__} "
    f"pyarrow={pyarrow.__version__} torch={torch.__version__} "
    f"torchcodec={torchcodec.__version__} dataset={LeRobotDataset.__name__}"
)
PY
echo "LEROBOT_VENV_READY=${VENV}"
