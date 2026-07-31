#!/usr/bin/env bash
# Passive two-camera/ROS-bag lifecycle and LeRobot v3 export.
# This script never powers, enables, or commands the robot.

set -Eeo pipefail

ACTION="${1:-status}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
ORBBEC_SETUP="${ONE_ARM_ORBBEC_SETUP:-/home/tele/ros2_ws/install/setup.bash}"
WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"
LEROBOT_PYTHON="${ONE_ARM_LEROBOT_PYTHON:-/home/tele/.venvs/onearm-lerobot/bin/python}"
SSD_MOUNT="${ONE_ARM_DATASET_SSD_MOUNT:-/media/tele/f05c1455-ef49-4879-9332-d6cf5c5557c4}"
DATA_ROOT="${SSD_MOUNT}/onearm_Tele"
RAW_ROOT="${DATA_ROOT}/raw_episodes"
LEROBOT_ROOT="${DATA_ROOT}/lerobot_dataset"
RUNTIME_DIR="/tmp/one_arm_dataset_${UID}"
EPISODE_PATH_FILE="${RUNTIME_DIR}/episode_path.txt"
MINIMUM_FREE_BYTES="${ONE_ARM_DATASET_MIN_FREE_BYTES:-10737418240}"
HEAD_TOPIC="/camera_head/color/image_raw/compressed"
WRIST_TOPIC="/camera_wrist/color/image_raw/compressed"
HEAD_SERIAL="CPCD7530003J"
WRIST_SERIAL="CPCBC5300077"

mkdir -p "${RUNTIME_DIR}"

for setup_file in "${ROS_SETUP}" "${ORBBEC_SETUP}" "${WORKSPACE_SETUP}"; do
  if [[ ! -r "${setup_file}" ]]; then
    echo "ERROR: required setup is missing: ${setup_file}" >&2
    exit 2
  fi
done

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${ORBBEC_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"
set -u

component_pid_file() {
  printf '%s/%s.pid\n' "${RUNTIME_DIR}" "$1"
}

component_log_file() {
  printf '%s/%s.log\n' "${RUNTIME_DIR}" "$1"
}

component_alive() {
  local pid_file pid
  pid_file="$(component_pid_file "$1")"
  [[ -s "${pid_file}" ]] || return 1
  pid="$(<"${pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null
}

start_component() {
  local name="$1"
  shift
  local log_file pid_file pid
  log_file="$(component_log_file "${name}")"
  pid_file="$(component_pid_file "${name}")"
  nohup setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  pid=$!
  printf '%s\n' "${pid}" >"${pid_file}"
  echo "started ${name}: pid=${pid} log=${log_file}"
}

stop_component_group() {
  local name="$1"
  local pid_file pid deadline
  pid_file="$(component_pid_file "${name}")"
  [[ -s "${pid_file}" ]] || return 0
  pid="$(<"${pid_file}")"
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    deadline=$((SECONDS + 8))
    while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
      sleep 0.10
    done
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
      deadline=$((SECONDS + 5))
      while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
        sleep 0.10
      done
    fi
    if kill -0 "${pid}" 2>/dev/null; then
      echo "ERROR: ${name} did not stop: pid=${pid}" >&2
      return 1
    fi
  fi
  rm -f -- "${pid_file}"
}

topic_publisher_count() {
  local output count
  output="$(ros2 topic info "$1" 2>/dev/null || true)"
  count="$(sed -n 's/^Publisher count: //p' <<<"${output}")"
  printf '%s\n' "${count:-0}"
}

wait_for_topic_publisher() {
  local topic="$1"
  local timeout_seconds="${2:-20}"
  local deadline=$((SECONDS + timeout_seconds))
  local count
  while (( SECONDS < deadline )); do
    count="$(topic_publisher_count "${topic}")"
    if [[ "${count}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
      return 0
    fi
    sleep 0.25
  done
  echo "ERROR: no active publisher appeared for ${topic}" >&2
  return 1
}

format_bytes() {
  python3 - "$1" <<'PY'
import sys
value = int(sys.argv[1])
print(f"{value / (1024 ** 3):.2f} GiB")
PY
}

preflight_storage() {
  local source options available_blocks block_size available_bytes
  [[ -d "${SSD_MOUNT}" ]] || {
    echo "ERROR: SSD mount directory is absent: ${SSD_MOUNT}" >&2
    return 8
  }
  source="$(findmnt -rn -T "${SSD_MOUNT}" -o SOURCE)"
  options="$(findmnt -rn -T "${SSD_MOUNT}" -o OPTIONS)"
  [[ -n "${source}" ]] || {
    echo "ERROR: ${SSD_MOUNT} is not a mounted filesystem." >&2
    return 8
  }
  if ! grep -Eq '(^|,)rw(,|$)' <<<"${options}"; then
    echo "ERROR: dataset SSD is not mounted read/write: ${options}" >&2
    return 8
  fi
  [[ -w "${SSD_MOUNT}" ]] || {
    echo "ERROR: user ${USER} cannot write the SSD mount." >&2
    return 8
  }
  available_blocks="$(stat -f -c '%a' "${SSD_MOUNT}")"
  block_size="$(stat -f -c '%S' "${SSD_MOUNT}")"
  available_bytes=$((available_blocks * block_size))
  echo "Dataset SSD: source=${source} mount=${SSD_MOUNT}"
  echo "Available to ${USER}: $(format_bytes "${available_bytes}")"
  if (( available_bytes < MINIMUM_FREE_BYTES )); then
    echo "ERROR: less than $(format_bytes "${MINIMUM_FREE_BYTES}") is available to the recorder." >&2
    echo "The filesystem may still have root-reserved ext4 blocks; df free space alone is insufficient." >&2
    return 8
  fi
}

preflight_cameras() {
  local serial
  ros2 pkg prefix orbbec_camera >/dev/null
  for serial in "${HEAD_SERIAL}" "${WRIST_SERIAL}"; do
    if ! compgen -G "/dev/v4l/by-id/*${serial}*" >/dev/null; then
      echo "ERROR: required Orbbec serial is absent: ${serial}" >&2
      return 9
    fi
  done
}

preflight_python() {
  python3 - <<'PY'
import rclpy
import rosbag2_py
from sensor_msgs.msg import CompressedImage
print("ROS_PYTHON_OK")
PY
}

preflight_all() {
  preflight_storage
  preflight_cameras
  preflight_python
  echo "DATASET_PREFLIGHT_OK"
  echo "Only head=${HEAD_SERIAL} and right_wrist=${WRIST_SERIAL} are configured."
  echo "The two chest cameras are excluded."
}

start_cameras() {
  preflight_all
  if component_alive cameras; then
    echo "ERROR: this launcher's camera process is already running." >&2
    return 3
  fi
  if (( $(topic_publisher_count "${HEAD_TOPIC}") > 0 )) ||
     (( $(topic_publisher_count "${WRIST_TOPIC}") > 0 )); then
    echo "ERROR: a dataset camera topic already has an active publisher." >&2
    return 3
  fi
  start_component cameras \
    ros2 launch one_arm_teleop_bridge dataset_cameras.launch.py
  if ! wait_for_topic_publisher "${HEAD_TOPIC}" 25 ||
     ! wait_for_topic_publisher "${WRIST_TOPIC}" 25; then
    tail -n 80 "$(component_log_file cameras)" >&2 || true
    stop_component_group cameras || true
    return 9
  fi
  if ! python3 "${PROJECT_ROOT}/tools/check_camera_fps.py" \
      --duration 6 --warmup 1; then
    tail -n 80 "$(component_log_file cameras)" >&2 || true
    stop_component_group cameras || true
    return 9
  fi
  echo "DATASET_CAMERAS_READY_30FPS"
}

wait_for_record_topics() {
  local topic
  for topic in \
    /right_arm/executed_joint_command \
    /right_arm/joint_states \
    /right_arm/gripper_command \
    /right_arm/executed_gripper_command \
    /right_arm/gripper_feedback_valid \
    /right_arm/gripper_contact \
    "${HEAD_TOPIC}" \
    "${WRIST_TOPIC}"; do
    wait_for_topic_publisher "${topic}" 20 || return 1
  done
}

start_recording() {
  local episode_name="$1"
  local task_base64="$2"
  local operator_base64="${3:-}"
  local task operator
  [[ "${episode_name}" =~ ^[A-Za-z0-9_-]+$ ]] || {
    echo "ERROR: episode name must contain only A-Z, a-z, 0-9, _ or -." >&2
    return 2
  }
  component_alive cameras || {
    echo "ERROR: dataset cameras are not running." >&2
    return 3
  }
  if component_alive recorder; then
    echo "ERROR: an episode recorder is already running." >&2
    return 3
  fi
  preflight_storage
  wait_for_record_topics
  task="$(printf '%s' "${task_base64}" | base64 --decode)"
  operator="$(printf '%s' "${operator_base64}" | base64 --decode)"
  [[ -n "${task}" && "${task}" == "${task#"${task%%[![:space:]]*}"}" &&
     "${task}" == "${task%"${task##*[![:space:]]}"}" ]] || {
    echo "ERROR: task must be nonempty with no leading/trailing whitespace." >&2
    return 2
  }

  mkdir -p "${RAW_ROOT}" "${LEROBOT_ROOT}"
  rm -f -- "${EPISODE_PATH_FILE}"
  start_component recorder \
    python3 "${PROJECT_ROOT}/tools/ros2_episode_recorder.py" \
      --name "${episode_name}" \
      --task "${task}" \
      --operator "${operator}" \
      --fps 30 \
      --profile lerobot \
      --storage sqlite3 \
      --head-topic "${HEAD_TOPIC}" \
      --wrist-topic "${WRIST_TOPIC}" \
      --output-root "${RAW_ROOT}" \
      --episode-path-file "${EPISODE_PATH_FILE}"

  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if [[ -s "${EPISODE_PATH_FILE}" ]]; then
      echo "DATASET_RECORDING_STARTED"
      echo "EPISODE_DIR=$(<"${EPISODE_PATH_FILE}")"
      return 0
    fi
    if ! component_alive recorder; then
      echo "ERROR: episode recorder exited during startup." >&2
      tail -n 100 "$(component_log_file recorder)" >&2 || true
      return 10
    fi
    sleep 0.20
  done
  echo "ERROR: recorder did not publish its episode directory." >&2
  tail -n 100 "$(component_log_file recorder)" >&2 || true
  return 10
}

stop_recorder() {
  local pid_file pid deadline
  pid_file="$(component_pid_file recorder)"
  if [[ -s "${pid_file}" ]]; then
    pid="$(<"${pid_file}")"
    if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
      # Signal only the Python supervisor; it closes rosbag cleanly itself.
      kill -INT "${pid}" 2>/dev/null || true
      deadline=$((SECONDS + 30))
      while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
        sleep 0.20
      done
      if kill -0 "${pid}" 2>/dev/null; then
        echo "WARNING: recorder shutdown timed out; terminating its process group." >&2
        kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
        deadline=$((SECONDS + 5))
        while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
          sleep 0.10
        done
      fi
      if kill -0 "${pid}" 2>/dev/null; then
        echo "ERROR: recorder is still alive: pid=${pid}" >&2
        return 1
      fi
    fi
    rm -f -- "${pid_file}"
  fi
  if [[ -s "${EPISODE_PATH_FILE}" ]]; then
    echo "DATASET_RECORDING_STOPPED"
    echo "EPISODE_DIR=$(<"${EPISODE_PATH_FILE}")"
  fi
}

stop_all() {
  local failed=0
  stop_recorder || failed=1
  stop_component_group cameras || failed=1
  if (( failed != 0 )); then
    return 1
  fi
  echo "DATASET_CAPTURE_STOPPED"
}

finalize_episode() {
  local outcome="$1"
  local repo_id="$2"
  local episode_dir
  [[ "${outcome}" == "success" || "${outcome}" == "failure" ]] || {
    echo "ERROR: outcome must be success or failure." >&2
    return 2
  }
  [[ "${repo_id}" =~ ^[^/[:space:]]+/[^/[:space:]]+$ ]] || {
    echo "ERROR: repo_id must have owner/name form." >&2
    return 2
  }
  [[ -s "${EPISODE_PATH_FILE}" ]] || {
    echo "ERROR: no completed episode path is available." >&2
    return 10
  }
  episode_dir="$(<"${EPISODE_PATH_FILE}")"
  python3 "${PROJECT_ROOT}/tools/set_episode_outcome.py" \
    "${episode_dir}" --outcome "${outcome}"
  if [[ "${outcome}" == "failure" ]]; then
    echo "EPISODE_MARKED_FAILURE_RAW_PRESERVED"
    echo "EPISODE_DIR=${episode_dir}"
    return 0
  fi
  [[ -x "${LEROBOT_PYTHON}" ]] || {
    echo "ERROR: LeRobot environment is missing: ${LEROBOT_PYTHON}" >&2
    echo "Run tools/setup_lerobot_ubuntu.sh once." >&2
    return 11
  }
  "${LEROBOT_PYTHON}" - <<'PY'
import lerobot
import pyarrow
import cv2
import rosbag2_py
print("LEROBOT_EXPORT_ENV_OK")
PY
  (
    flock -n 9 || {
      echo "ERROR: another LeRobot exporter is writing this dataset." >&2
      exit 12
    }
    "${LEROBOT_PYTHON}" "${PROJECT_ROOT}/tools/export_rosbag_to_lerobot.py" \
      --episode-dir "${episode_dir}" \
      --dataset-root "${LEROBOT_ROOT}" \
      --repo-id "${repo_id}" \
      --fps 30 \
      --head-topic "${HEAD_TOPIC}" \
      --wrist-topic "${WRIST_TOPIC}"
  ) 9>"${RUNTIME_DIR}/lerobot_export.lock"
  echo "LEROBOT_EPISODE_EXPORTED"
  echo "LEROBOT_DATASET=${LEROBOT_ROOT}"
}

show_status() {
  local name
  echo "DATA_ROOT=${DATA_ROOT}"
  for name in cameras recorder; do
    if component_alive "${name}"; then
      echo "${name}: running pid=$(<"$(component_pid_file "${name}")")"
    else
      echo "${name}: stopped"
    fi
  done
  if [[ -s "${EPISODE_PATH_FILE}" ]]; then
    echo "EPISODE_DIR=$(<"${EPISODE_PATH_FILE}")"
  fi
  preflight_storage || true
}

case "${ACTION}" in
  preflight)
    preflight_all
    ;;
  start)
    start_cameras
    ;;
  record-start)
    [[ $# -ge 3 ]] || {
      echo "Usage: $0 record-start EPISODE_NAME TASK_BASE64 [OPERATOR_BASE64]" >&2
      exit 2
    }
    start_recording "$2" "$3" "${4:-}"
    ;;
  stop)
    stop_all
    ;;
  finalize)
    [[ $# -eq 3 ]] || {
      echo "Usage: $0 finalize {success|failure} OWNER/DATASET" >&2
      exit 2
    }
    finalize_episode "$2" "$3"
    ;;
  status)
    show_status
    ;;
  *)
    echo "Usage: $0 {preflight|start|record-start|stop|finalize|status}" >&2
    exit 2
    ;;
esac
