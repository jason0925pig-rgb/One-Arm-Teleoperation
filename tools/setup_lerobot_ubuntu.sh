#!/usr/bin/env bash
# One-time Ubuntu setup for offline ROS bag -> LeRobot Dataset v3 export.

set -Eeo pipefail

VENV="${ONE_ARM_LEROBOT_VENV:-${HOME}/.venvs/onearm-lerobot}"
PIP_CACHE_DIR="${ONE_ARM_PIP_CACHE_DIR:-$(dirname "${VENV}")/pip-cache}"
if [[ -n "${ROS_DISTRO:-}" ]]; then
  ROS_DISTRO_NAME="${ROS_DISTRO}"
elif [[ -r /opt/ros/humble/setup.bash ]]; then
  ROS_DISTRO_NAME="humble"
else
  ROS_DISTRO_NAME="jazzy"
fi
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
PYPI_INDEX="${ONE_ARM_PYPI_INDEX:-https://pypi.org/simple}"
ARCHITECTURE="$(uname -m)"
PYTHON_MINOR="$(python3 -c 'import sys; print(sys.version_info.minor)')"
if [[ -n "${ONE_ARM_LEROBOT_VERSION:-}" ]]; then
  LEROBOT_VERSION="${ONE_ARM_LEROBOT_VERSION}"
elif (( PYTHON_MINOR >= 12 )); then
  LEROBOT_VERSION="0.6.0"
else
  # LeRobot 0.4.4 supports Python 3.10 and already writes CODEBASE_VERSION
  # v3.0 datasets.  LeRobot 0.6.0 itself requires Python 3.12 or newer.
  LEROBOT_VERSION="0.4.4"
fi

if [[ "${ARCHITECTURE}" == "aarch64" || "${ARCHITECTURE}" == "arm64" ]]; then
  if [[ "${LEROBOT_VERSION}" == "0.4.4" ]]; then
    TORCH_VERSION="${ONE_ARM_TORCH_VERSION:-2.10.0}"
    TORCHVISION_VERSION="${ONE_ARM_TORCHVISION_VERSION:-0.25.0}"
    # LeRobot 0.4.4 intentionally uses PyAV on Linux ARM64.
    TORCHCODEC_VERSION="${ONE_ARM_TORCHCODEC_VERSION:-}"
  else
    TORCH_VERSION="${ONE_ARM_TORCH_VERSION:-2.11.0}"
    TORCHVISION_VERSION="${ONE_ARM_TORCHVISION_VERSION:-0.26.0}"
    TORCHCODEC_VERSION="${ONE_ARM_TORCHCODEC_VERSION:-0.11.1}"
  fi
  # This isolated environment is for dataset export.  SmolVLA GPU inference
  # should use a separate, JetPack-compatible NVIDIA PyTorch/container.
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

mkdir -p "${PIP_CACHE_DIR}"
export PIP_CACHE_DIR

if [[ ! -x "${VENV}/bin/python" ]]; then
  python3 -m venv --system-site-packages "${VENV}"
fi
"${VENV}/bin/python" -m pip install --index-url "${PYPI_INDEX}" --upgrade pip
"${VENV}/bin/python" -m pip install \
  --index-url "${TORCH_INDEX}" \
  "${TORCH_SPEC}" \
  "${TORCHVISION_SPEC}"
if [[ "${LEROBOT_VERSION}" == "0.4.4" ]]; then
  # 0.4.4 already includes the dataset package and does not define a
  # separately named `dataset` extra.  Asking for that extra only produces a
  # misleading warning on the Jetson's Python 3.10 environment.
  LEROBOT_SPEC="lerobot==${LEROBOT_VERSION}"
else
  LEROBOT_SPEC="lerobot[dataset]==${LEROBOT_VERSION}"
fi
"${VENV}/bin/python" -m pip install \
  --index-url "${PYPI_INDEX}" \
  "${LEROBOT_SPEC}"
EXTRA_PACKAGES=(
  "numpy>=2.0,<2.3" \
  "pandas>=2.2,<3" \
  "scipy>=1.13,<2" \
  "setuptools>=71,<80" \
  "numexpr>=2.10,<3" \
  "bottleneck>=1.4,<2"
)
if [[ -n "${TORCHCODEC_VERSION}" ]]; then
  EXTRA_PACKAGES+=("torchcodec==${TORCHCODEC_VERSION}")
fi
"${VENV}/bin/python" -m pip install \
  --index-url "${PYPI_INDEX}" \
  "${EXTRA_PACKAGES[@]}"
export ONE_ARM_EXPECT_TORCHCODEC="${TORCHCODEC_VERSION}"
"${VENV}/bin/python" - <<'PY'
import os

import cv2
import lerobot
import pandas
import pyarrow
import rclpy
import rosbag2_py
import torch
from lerobot.datasets import LeRobotDataset

torchcodec_version = "PyAV fallback"
if os.environ.get("ONE_ARM_EXPECT_TORCHCODEC"):
    import torchcodec

    torchcodec_version = torchcodec.__version__

print(
    "LeRobot environment ready: "
    f"lerobot={lerobot.__version__} pandas={pandas.__version__} "
    f"pyarrow={pyarrow.__version__} torch={torch.__version__} "
    f"video={torchcodec_version} dataset={LeRobotDataset.__name__}"
)
PY
echo "LEROBOT_VENV_READY=${VENV}"
