#!/usr/bin/env bash
# Passive two-camera/ROS-bag lifecycle and LeRobot v3 export.
# This script never powers, enables, or commands the robot.

set -Eeo pipefail

ACTION="${1:-status}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
ORBBEC_SETUP="${ONE_ARM_ORBBEC_SETUP:-${ROS_SETUP}}"
WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"
LEROBOT_PYTHON="${ONE_ARM_LEROBOT_PYTHON:-/home/tele/.venvs/onearm-lerobot/bin/python}"
DEFAULT_DATA_ROOT="${PROJECT_ROOT}/datasets/onearm_Tele"
DATA_ROOT="${ONE_ARM_DATASET_DATA_ROOT:-${DEFAULT_DATA_ROOT}}"
DATA_ROOT="${DATA_ROOT%/}"
RAW_ROOT="${DATA_ROOT}/raw_episodes"
LEROBOT_ROOT="${DATA_ROOT}/lerobot_dataset"
RUNTIME_DIR="/tmp/one_arm_dataset_${UID}"
EPISODE_PATH_FILE="${RUNTIME_DIR}/episode_path.txt"
MINIMUM_FREE_BYTES="${ONE_ARM_DATASET_MIN_FREE_BYTES:-10737418240}"
RECORDER_STOP_TIMEOUT_SECONDS="${ONE_ARM_RECORDER_STOP_TIMEOUT_SECONDS:-180}"
PRIMARY_CAMERA_ROLE="${ONE_ARM_PRIMARY_CAMERA_ROLE:-head}"
case "${PRIMARY_CAMERA_ROLE}" in
  head)
    PRIMARY_CAMERA_NAME="camera_head"
    PRIMARY_CAMERA_FEATURE="observation.images.head"
    ;;
  chest)
    PRIMARY_CAMERA_NAME="camera_chest"
    PRIMARY_CAMERA_FEATURE="observation.images.chest"
    ;;
  *)
    echo "ERROR: ONE_ARM_PRIMARY_CAMERA_ROLE must be head or chest." >&2
    exit 2
    ;;
esac
HEAD_TOPIC="/${PRIMARY_CAMERA_NAME}/color/image_raw/compressed"
WRIST_TOPIC="/camera_wrist/color/image_raw/compressed"
HEAD_SERIAL="${ONE_ARM_HEAD_SERIAL:-CPCD7530003J}"
WRIST_SERIAL="${ONE_ARM_WRIST_SERIAL:-CPCBC5300077}"
BACKGROUND_CPU_LIST="${ONE_ARM_BACKGROUND_CPUS:-}"
PREVIEW_BIND_HOST="${ONE_ARM_CAMERA_PREVIEW_BIND_HOST:-0.0.0.0}"
PREVIEW_PORT="${ONE_ARM_CAMERA_PREVIEW_PORT:-8088}"

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

configure_background_cpu_set() {
  local cpu_count
  cpu_count="$(nproc)"
  if [[ -z "${BACKGROUND_CPU_LIST}" ]]; then
    if (( cpu_count >= 3 )); then
      BACKGROUND_CPU_LIST="2-$((cpu_count - 1))"
    else
      BACKGROUND_CPU_LIST="0"
    fi
  fi
  taskset -c "${BACKGROUND_CPU_LIST}" true >/dev/null
}

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
  python3 - "$1" <<'PY' 2>/dev/null || printf '0\n'
import os
import sys
import time

import rclpy
from rclpy.node import Node

topic = sys.argv[1]
rclpy.init(args=None)
node = Node(f"onearm_topic_count_probe_{os.getpid()}")
try:
    deadline = time.monotonic() + 0.5
    count = 0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        count = len(node.get_publishers_info_by_topic(topic))
        if count > 0:
            break
    print(count)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

print_topic_diagnostics() {
  local topic
  echo "--- ROS 2 graph diagnostics ---" >&2
  echo "ROS_DISTRO=${ROS_DISTRO:-unknown}" >&2
  echo "PROJECT_ROOT=${PROJECT_ROOT}" >&2
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 pkg prefix servo_controller 2>/dev/null | sed 's/^/servo_controller prefix: /' >&2 || true
  ros2 pkg prefix one_arm_teleop_bridge 2>/dev/null | sed 's/^/one_arm_teleop_bridge prefix: /' >&2 || true
  echo "Nodes:" >&2
  ros2 node list 2>/dev/null >&2 || true
  for topic in "$@"; do
    echo "--- ${topic} ---" >&2
    ros2 topic info "${topic}" --verbose 2>/dev/null >&2 ||
      ros2 topic info "${topic}" 2>/dev/null >&2 ||
      true
  done
  echo "--- launcher component logs ---" >&2
  for topic in arm gripper bridge cameras recorder; do
    local log_file
    log_file="$(component_log_file "${topic}")"
    if [[ -r "${log_file}" ]]; then
      echo "--- ${topic}.log tail ---" >&2
      tail -n 35 "${log_file}" >&2 || true
    fi
  done
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

wait_for_topic_publisher_stable() {
  local topic="$1"
  local timeout_seconds="${2:-25}"
  local stable_samples="${3:-5}"
  local deadline=$((SECONDS + timeout_seconds))
  local count consecutive=0
  while (( SECONDS < deadline )); do
    count="$(topic_publisher_count "${topic}")"
    if [[ "${count}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
      consecutive=$((consecutive + 1))
      if (( consecutive >= stable_samples )); then
        return 0
      fi
    else
      consecutive=0
    fi
    sleep 0.20
  done
  echo "ERROR: no stable active publisher for ${topic}" >&2
  return 1
}

wait_for_preview() {
  local timeout_seconds="${1:-15}"
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    if python3 - "http://127.0.0.1:${PREVIEW_PORT}/health" <<'PY'
import json
import sys
import urllib.request

try:
    with urllib.request.urlopen(sys.argv[1], timeout=1.0) as response:
        status = json.load(response)
    raise SystemExit(0 if all(status[name]["frames"] > 0 for name in ("chest", "wrist")) else 1)
except Exception:
    raise SystemExit(1)
PY
    then
      return 0
    fi
    if ! component_alive preview; then
      echo "ERROR: dual-camera preview exited during startup." >&2
      tail -n 80 "$(component_log_file preview)" >&2 || true
      return 1
    fi
    sleep 0.25
  done
  echo "ERROR: dual-camera preview did not receive both image streams." >&2
  tail -n 80 "$(component_log_file preview)" >&2 || true
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
  local check_path source options available_blocks block_size available_bytes
  mkdir -p "${DATA_ROOT}"
  check_path="${DATA_ROOT}"
  source="$(findmnt -rn -T "${check_path}" -o SOURCE)"
  options="$(findmnt -rn -T "${check_path}" -o OPTIONS)"
  [[ -n "${source}" ]] || {
    echo "ERROR: ${check_path} is not on a mounted filesystem." >&2
    return 8
  }
  if ! grep -Eq '(^|,)rw(,|$)' <<<"${options}"; then
    echo "ERROR: dataset filesystem is not mounted read/write: ${options}" >&2
    return 8
  fi
  [[ -w "${check_path}" ]] || {
    echo "ERROR: user ${USER} cannot write the dataset path: ${check_path}" >&2
    return 8
  }
  available_blocks="$(stat -f -c '%a' "${check_path}")"
  block_size="$(stat -f -c '%S' "${check_path}")"
  available_bytes=$((available_blocks * block_size))
  echo "Dataset storage: source=${source} check_path=${check_path}"
  echo "DATA_ROOT=${DATA_ROOT}"
  echo "Available to ${USER}: $(format_bytes "${available_bytes}")"
  if (( available_bytes < MINIMUM_FREE_BYTES )); then
    echo "ERROR: less than $(format_bytes "${MINIMUM_FREE_BYTES}") is available to the recorder." >&2
    echo "The filesystem may still have root-reserved ext4 blocks; df free space alone is insufficient." >&2
    return 8
  fi
  mkdir -p "${RAW_ROOT}"
}

preflight_cameras() {
  local serial missing_serials=() orbbec_count
  ros2 pkg prefix orbbec_camera >/dev/null
  for serial in "${HEAD_SERIAL}" "${WRIST_SERIAL}"; do
    if ! compgen -G "/dev/v4l/by-id/*${serial}*" >/dev/null; then
      missing_serials+=("${serial}")
    fi
  done
  if ((${#missing_serials[@]} == 0)); then
    return 0
  fi

  orbbec_count="$(
    lsusb 2>/dev/null | grep -Eci 'Orbbec|2bc5:0807|Gemini 336L' || true
  )"
  if [[ "${orbbec_count}" =~ ^[0-9]+$ ]] && ((orbbec_count >= 2)); then
    echo "WARNING: /dev/v4l/by-id is missing serial link(s): ${missing_serials[*]}" >&2
    echo "WARNING: ${orbbec_count} Orbbec USB device(s) are visible; launch will validate the configured serial numbers." >&2
    return 0
  fi

  echo "ERROR: required Orbbec serial link(s) are absent: ${missing_serials[*]}" >&2
  echo "ERROR: visible Orbbec USB device count is ${orbbec_count:-0}, expected at least 2." >&2
  return 9
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
  # The ROS 2 CLI daemon can become stale after interrupted launches and then
  # make `ros2 topic info/list` fail with `rclpy.ok()` XMLRPC errors. Stopping
  # it is safe for running ROS nodes and keeps diagnostics from poisoning the
  # camera startup path.
  ros2 daemon stop >/dev/null 2>&1 || true
  preflight_storage
  preflight_cameras
  preflight_python
  echo "DATASET_PREFLIGHT_OK"
  echo "Primary ${PRIMARY_CAMERA_ROLE}=${HEAD_SERIAL}; right_wrist=${WRIST_SERIAL}."
  echo "Every other installed camera is excluded from this dataset profile."
}

start_cameras() {
  preflight_all
  configure_background_cpu_set
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
    nice -n 5 taskset -c "${BACKGROUND_CPU_LIST}" \
      ros2 launch one_arm_teleop_bridge dataset_cameras.launch.py \
      primary_camera_name:="${PRIMARY_CAMERA_NAME}" \
      head_serial:="${HEAD_SERIAL}" \
      wrist_serial:="${WRIST_SERIAL}"
  if ! wait_for_topic_publisher "${HEAD_TOPIC}" 25 ||
     ! wait_for_topic_publisher "${WRIST_TOPIC}" 25; then
    tail -n 80 "$(component_log_file cameras)" >&2 || true
    stop_component_group cameras || true
    return 9
  fi
  if ! python3 "${PROJECT_ROOT}/tools/check_camera_fps.py" \
      --duration 6 --warmup 1 \
      --topic "${HEAD_TOPIC}" --topic "${WRIST_TOPIC}"; then
    tail -n 80 "$(component_log_file cameras)" >&2 || true
    stop_component_group cameras || true
    return 9
  fi
  start_component preview \
    nice -n 10 taskset -c "${BACKGROUND_CPU_LIST}" \
      python3 "${PROJECT_ROOT}/tools/dual_camera_preview_server.py" \
      --host "${PREVIEW_BIND_HOST}" \
      --port "${PREVIEW_PORT}" \
      --chest-topic "${HEAD_TOPIC}" \
      --wrist-topic "${WRIST_TOPIC}"
  if ! wait_for_preview 15; then
    stop_component_group preview || true
    stop_component_group cameras || true
    return 9
  fi
  echo "DATASET_CAMERAS_READY_30FPS"
  echo "CAMERA_PREVIEW_READY port=${PREVIEW_PORT} layout=chest-left,wrist-right"
}

wait_for_record_topics() {
  local topic
  local -a topics
  topics=(
    /right_arm/executed_joint_command
    /right_arm/joint_states
    /right_arm/gripper_command
    /right_arm/executed_gripper_command
    /right_arm/gripper_feedback_valid
    /right_arm/gripper_contact
    "${HEAD_TOPIC}"
    "${WRIST_TOPIC}"
  )
  for topic in "${topics[@]}"; do
    if ! wait_for_topic_publisher_stable "${topic}" 30 6; then
      print_topic_diagnostics "${topics[@]}"
      return 1
    fi
  done
  echo "DATASET_REQUIRED_TOPICS_READY"
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
  configure_background_cpu_set
  wait_for_record_topics
  task="$(printf '%s' "${task_base64}" | base64 --decode)"
  operator="$(printf '%s' "${operator_base64}" | base64 --decode)"
  [[ -n "${task}" && "${task}" == "${task#"${task%%[![:space:]]*}"}" &&
     "${task}" == "${task%"${task##*[![:space:]]}"}" ]] || {
    echo "ERROR: task must be nonempty with no leading/trailing whitespace." >&2
    return 2
  }

  mkdir -p "${RAW_ROOT}"
  rm -f -- "${EPISODE_PATH_FILE}"
  start_component recorder \
    nice -n 5 taskset -c "${BACKGROUND_CPU_LIST}" \
      python3 "${PROJECT_ROOT}/tools/ros2_episode_recorder.py" \
      --name "${episode_name}" \
      --task "${task}" \
      --operator "${operator}" \
      --fps 30 \
      --profile lerobot \
      --storage sqlite3 \
      --head-topic "${HEAD_TOPIC}" \
      --wrist-topic "${WRIST_TOPIC}" \
      --primary-camera-feature "${PRIMARY_CAMERA_FEATURE}" \
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
      deadline=$((SECONDS + RECORDER_STOP_TIMEOUT_SECONDS))
      while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
        sleep 0.20
      done
      if kill -0 "${pid}" 2>/dev/null; then
        echo "WARNING: recorder shutdown timed out after ${RECORDER_STOP_TIMEOUT_SECONDS}s; asking its supervisor to terminate rosbag." >&2
        kill -TERM "${pid}" 2>/dev/null || true
        deadline=$((SECONDS + 20))
        while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
          sleep 0.10
        done
      fi
      if kill -0 "${pid}" 2>/dev/null; then
        local child_pid
        while read -r child_pid; do
          [[ "${child_pid}" =~ ^[0-9]+$ ]] || continue
          kill -KILL -- "-${child_pid}" 2>/dev/null ||
            kill -KILL "${child_pid}" 2>/dev/null || true
        done < <(pgrep -P "${pid}" 2>/dev/null || true)
        kill -KILL "${pid}" 2>/dev/null || true
        echo "ERROR: recorder required SIGKILL: pid=${pid}" >&2
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
  stop_component_group preview || failed=1
  stop_component_group cameras || failed=1
  if (( failed != 0 )); then
    return 1
  fi
  echo "DATASET_CAPTURE_STOPPED"
}

archive_runtime_logs() {
  local episode_dir="$1"
  local destination="${episode_dir}/runtime_logs"
  local source
  mkdir -p "${destination}"
  for source in \
    "${RUNTIME_DIR}/recorder.log" \
    "${RUNTIME_DIR}/preview.log" \
    "${RUNTIME_DIR}/cameras.log" \
    "/tmp/one_arm_teleop_full_${UID}/arm.log" \
    "/tmp/one_arm_teleop_full_${UID}/gripper.log" \
    "/tmp/one_arm_teleop_full_${UID}/bridge.log"; do
    if [[ -r "${source}" ]]; then
      cp -f -- "${source}" "${destination}/$(basename "${source}")"
    fi
  done
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
  if [[ "${outcome}" == "failure" ]]; then
    discard_episode
    echo "EPISODE_FAILURE_DISCARDED_NO_RAW_KEPT"
    return 0
  fi
  episode_dir="$(<"${EPISODE_PATH_FILE}")"
  archive_runtime_logs "${episode_dir}"
  python3 "${PROJECT_ROOT}/tools/set_episode_outcome.py" \
    "${episode_dir}" \
    --outcome "${outcome}" \
    --recover-closed-recording
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
      --wrist-topic "${WRIST_TOPIC}" \
      --primary-camera-feature "${PRIMARY_CAMERA_FEATURE}"
  ) 9>"${RUNTIME_DIR}/lerobot_export.lock"
  echo "LEROBOT_EPISODE_EXPORTED"
  echo "LEROBOT_DATASET=${LEROBOT_ROOT}"
}

discard_episode() {
  local episode_dir
  local episode_real
  local raw_root_real
  [[ -s "${EPISODE_PATH_FILE}" ]] || {
    echo "ERROR: no completed episode path is available to discard." >&2
    return 10
  }
  if component_alive recorder; then
    echo "ERROR: refusing to discard while the recorder is still running." >&2
    return 13
  fi
  episode_dir="$(<"${EPISODE_PATH_FILE}")"
  [[ -d "${episode_dir}" ]] || {
    echo "ERROR: episode directory is missing: ${episode_dir}" >&2
    return 10
  }
  raw_root_real="$(realpath -e "${RAW_ROOT}")"
  episode_real="$(realpath -e "${episode_dir}")"
  [[ "$(dirname "${episode_real}")" == "${raw_root_real}" ]] || {
    echo "ERROR: refusing to discard path outside raw root: ${episode_real}" >&2
    return 14
  }
  [[ "$(basename "${episode_real}")" =~ ^[0-9]{8}_[0-9]{6}_[A-Za-z0-9_-]+$ ]] || {
    echo "ERROR: refusing unexpected episode directory name: ${episode_real}" >&2
    return 14
  }
  if pgrep -af "export_rosbag_to_lerobot.py.*--episode-dir[[:space:]]+${episode_real}" >/dev/null; then
    echo "ERROR: refusing to discard while this episode is exporting." >&2
    return 13
  fi
  rm -rf -- "${episode_real}"
  rm -f -- "${EPISODE_PATH_FILE}"
  echo "EPISODE_DISCARDED=${episode_real}"
}

show_status() {
  local name
  echo "DATA_ROOT=${DATA_ROOT}"
  for name in cameras preview recorder; do
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
  discard)
    [[ $# -eq 1 ]] || {
      echo "Usage: $0 discard" >&2
      exit 2
    }
    discard_episode
    ;;
  status)
    show_status
    ;;
  *)
    echo "Usage: $0 {preflight|start|record-start|stop|finalize|discard|status}" >&2
    exit 2
    ;;
esac
