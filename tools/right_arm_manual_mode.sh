#!/usr/bin/env bash
# Attended right-arm power/enable/drag mode with ordered Ctrl+C shutdown.

set -Eeo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"
ARM_CONFIG="${PROJECT_ROOT}/servo_controller/config/full_teleop_attended.yaml"
RUNTIME_DIR="/tmp/one_arm_manual_mode_${UID}"
LOCK_DIR="${RUNTIME_DIR}.lock"
PID_FILE="${RUNTIME_DIR}/arm.pid"
LOG_FILE="${RUNTIME_DIR}/arm.log"
ARM_PID=""
CLEANUP_STARTED=0
OPERATOR_STOP_REQUESTED=0

[[ -r "${ROS_SETUP}" ]] || {
  echo "ERROR: ROS setup is missing: ${ROS_SETUP}" >&2
  exit 2
}
[[ -r "${WORKSPACE_SETUP}" ]] || {
  echo "ERROR: workspace is not built: ${WORKSPACE_SETUP}" >&2
  exit 2
}
[[ -r "${ARM_CONFIG}" ]] || {
  echo "ERROR: arm configuration is missing: ${ARM_CONFIG}" >&2
  exit 2
}

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"
set -u

topic_once() {
  timeout "${2:-3}" ros2 topic echo --once --full-length "$1" 2>/dev/null
}

wait_for_service() {
  local service_name="$1"
  local deadline=$((SECONDS + ${2:-20}))
  while (( SECONDS < deadline )); do
    if ros2 service list 2>/dev/null | grep -Fxq "${service_name}"; then
      return 0
    fi
    sleep 0.20
  done
  echo "ERROR: service did not appear: ${service_name}" >&2
  return 1
}

wait_status_field() {
  local expected="$1"
  local deadline=$((SECONDS + ${2:-30}))
  local status=""
  while (( SECONDS < deadline )); do
    status="$(topic_once /right_arm/safety_status 2 || true)"
    if grep -Fq "${expected}" <<<"${status}"; then
      printf '%s\n' "${status}"
      return 0
    fi
    sleep 0.20
  done
  echo "ERROR: right-arm status did not reach ${expected}" >&2
  [[ -n "${status}" ]] && printf '%s\n' "${status}" >&2
  return 1
}

call_set_bool() {
  local service_name="$1"
  local value="$2"
  local output=""
  output="$(
    timeout 20 ros2 service call \
      "${service_name}" std_srvs/srv/SetBool "{data: ${value}}" 2>&1
  )" || {
    printf '%s\n' "${output}" >&2
    return 1
  }
  printf '%s\n' "${output}"
  grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}"
}

cleanup() {
  local exit_code=$?
  if (( CLEANUP_STARTED != 0 )); then
    return
  fi
  CLEANUP_STARTED=1
  trap - EXIT INT TERM HUP
  set +e

  echo
  echo "Stopping manual mode: drag off -> disable -> power off..."
  if ros2 service list 2>/dev/null | grep -Fxq /right_arm/set_drag_enabled; then
    call_set_bool /right_arm/set_drag_enabled false || true
    wait_status_field "robot_drag_status=0" 8 >/dev/null || true
    call_set_bool /right_arm/set_motion_enabled false || true
    call_set_bool /right_arm/set_robot_enabled false || true
    wait_status_field "robot_enabled=0" 12 >/dev/null || true
    call_set_bool /right_arm/set_powered_on false || true
    wait_status_field "robot_powered_on=0" 20 >/dev/null || true
  fi

  if [[ -n "${ARM_PID}" ]] && kill -0 "${ARM_PID}" 2>/dev/null; then
    kill -INT -- "-${ARM_PID}" 2>/dev/null || kill -INT "${ARM_PID}" 2>/dev/null || true
    local deadline=$((SECONDS + 8))
    while kill -0 "${ARM_PID}" 2>/dev/null && (( SECONDS < deadline )); do
      sleep 0.10
    done
    if kill -0 "${ARM_PID}" 2>/dev/null; then
      kill -TERM -- "-${ARM_PID}" 2>/dev/null || kill -TERM "${ARM_PID}" 2>/dev/null || true
    fi
  fi
  rm -f -- "${PID_FILE}"
  rmdir -- "${RUNTIME_DIR}" 2>/dev/null || true
  rmdir -- "${LOCK_DIR}" 2>/dev/null || true
  echo "RIGHT_ARM_MANUAL_MODE_STOPPED"
  if (( OPERATOR_STOP_REQUESTED != 0 )); then
    exit_code=0
  fi
  exit "${exit_code}"
}

request_stop() {
  OPERATOR_STOP_REQUESTED=1
  exit 0
}

trap cleanup EXIT
trap request_stop INT TERM HUP

if ! mkdir "${LOCK_DIR}" 2>/dev/null; then
  echo "ERROR: right-arm manual mode is already running." >&2
  exit 3
fi
mkdir -p "${RUNTIME_DIR}"

if ! ping -c 1 -W 1 192.168.2.226 >/dev/null; then
  echo "ERROR: Armstrong controller 192.168.2.226 is unreachable." >&2
  exit 4
fi

if pgrep -af \
  'safe_one_arm_servo|robot_timer|tele_robot|servo_control.launch.py' \
  | grep -Ev "right_arm_manual_mode|pgrep -af" >/dev/null; then
  echo "ERROR: another arm-control process is already running." >&2
  pgrep -af \
    'safe_one_arm_servo|robot_timer|tele_robot|servo_control.launch.py' \
    | grep -Ev "right_arm_manual_mode|pgrep -af" >&2 || true
  exit 5
fi

if ros2 node list 2>/dev/null | grep -Eq \
  '^/(safe_one_arm_servo|robot_timer)$'; then
  echo "ERROR: a conflicting ROS 2 arm node is already visible." >&2
  ros2 node list >&2 || true
  exit 5
fi

echo "============================================================"
echo "RIGHT ARM MANUAL DRAG MODE"
echo "The arm may settle when enabled and becomes hand-guidable in drag mode."
echo "Keep the physical E-stop within reach and support any payload."
echo "Press Ctrl+C at any time for ordered drag-off/disable/power-off."
echo "Starting in 3 seconds..."
echo "============================================================"
sleep 3

setsid ros2 run servo_controller safe_one_arm_servo --ros-args \
  --params-file "${ARM_CONFIG}" \
  -p dry_run:=false \
  -p hardware_power_authorized:=true \
  -p hardware_enable_authorized:=true \
  -p hardware_drag_authorized:=true \
  -p hardware_motion_authorized:=false \
  -p limits_configured:=true \
  >"${LOG_FILE}" 2>&1 < /dev/null &
ARM_PID=$!
printf '%s\n' "${ARM_PID}" >"${PID_FILE}"

wait_for_service /right_arm/set_powered_on 20
wait_for_service /right_arm/set_robot_enabled 20
wait_for_service /right_arm/set_drag_enabled 20

status="$(topic_once /right_arm/safety_status 4 || true)"
printf '%s\n' "${status}"
for required in \
  "connected=1" \
  "feedback_valid=1" \
  "robot_emergency_stop=0" \
  "robot_protective_stop=0" \
  "robot_socket_connected=1" \
  "robot_error_code=0" \
  "motion_enabled=0" \
  "servo_mode_entered=0"; do
  if ! grep -Fq "${required}" <<<"${status}"; then
    echo "ERROR: right-arm safety prerequisite is missing: ${required}" >&2
    exit 6
  fi
done

call_set_bool /right_arm/set_powered_on true
wait_status_field "robot_powered_on=1" 30 >/dev/null
call_set_bool /right_arm/set_robot_enabled true
wait_status_field "robot_enabled=1" 30 >/dev/null
call_set_bool /right_arm/set_drag_enabled true
wait_status_field "robot_drag_status=1" 15 >/dev/null

echo
echo "RIGHT_ARM_MANUAL_DRAG_READY"
echo "You may now reposition the right arm by hand."
echo "Press Ctrl+C to exit drag mode, disable the arm, and power it off."

while kill -0 "${ARM_PID}" 2>/dev/null; do
  sleep 1
done

echo "ERROR: right-arm node exited unexpectedly. See ${LOG_FILE}" >&2
exit 7
