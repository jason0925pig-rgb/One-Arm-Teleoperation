#!/usr/bin/env bash
# Usage: bash tools/view_dataset_camera.sh {chest|wrist}

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup is missing: ${ROS_SETUP}" >&2
  exit 2
fi

# shellcheck disable=SC1090
source "${ROS_SETUP}"
export QT_X11_NO_MITSHM=1

if [[ -z "${DISPLAY:-}" && " ${*} " != *" --snapshot "* ]]; then
  echo "ERROR: DISPLAY is empty. Enable MobaXterm X11 forwarding and reconnect." >&2
  exit 2
fi

exec python3 "${PROJECT_ROOT}/tools/view_dataset_camera.py" "$@"
