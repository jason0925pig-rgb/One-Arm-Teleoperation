#!/usr/bin/env bash
# Attended one-arm teleoperation lifecycle for the Armstrong Ubuntu host.
# Starting processes is motion-free. Hardware motion is authorized only by
# the explicit "arm" action after leader preview and robot safety checks pass.

set -Eeo pipefail

ACTION="${1:-status}"
EXPECTED_SOURCE_IP="${2:-${ONE_ARM_WINDOWS_IP:-192.168.0.105}}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"
RUNTIME_DIR="/tmp/one_arm_teleop_full_${UID}"
GRIPPER_DEVICE="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5ABB000800-if00"
ARM_CONFIG="${PROJECT_ROOT}/servo_controller/config/full_teleop_attended.yaml"
BRIDGE_CONFIG="${PROJECT_ROOT}/one_arm_teleop_bridge/config/full_teleop_attended.yaml"

mkdir -p "${RUNTIME_DIR}"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found: ${ROS_SETUP}" >&2
  exit 2
fi
if [[ ! -r "${WORKSPACE_SETUP}" ]]; then
  echo "ERROR: workspace is not built: ${WORKSPACE_SETUP}" >&2
  exit 2
fi

# shellcheck disable=SC1090
source "${ROS_SETUP}"
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
  local pid_file
  pid_file="$(component_pid_file "$1")"
  [[ -s "${pid_file}" ]] || return 1
  local pid
  pid="$(<"${pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null
}

wait_for_service() {
  local service_name="$1"
  local timeout_seconds="${2:-15}"
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    if ros2 service list 2>/dev/null | grep -Fxq "${service_name}"; then
      return 0
    fi
    sleep 0.25
  done
  echo "ERROR: service did not appear: ${service_name}" >&2
  return 1
}

topic_once() {
  local topic_name="$1"
  local timeout_seconds="${2:-3}"
  timeout "${timeout_seconds}" ros2 topic echo --once "${topic_name}" 2>/dev/null
}

wait_status_field() {
  local expected="$1"
  local timeout_seconds="${2:-20}"
  local deadline=$((SECONDS + timeout_seconds))
  local output=""
  while (( SECONDS < deadline )); do
    output="$(topic_once /right_arm/safety_status 2 || true)"
    if grep -Fq "${expected}" <<<"${output}"; then
      return 0
    fi
    sleep 0.25
  done
  echo "ERROR: safety status did not reach ${expected}" >&2
  [[ -n "${output}" ]] && printf '%s\n' "${output}" >&2
  return 1
}

call_set_bool() {
  local service_name="$1"
  local value="$2"
  local output
  output="$(
    timeout 15 ros2 service call \
      "${service_name}" std_srvs/srv/SetBool "{data: ${value}}" 2>&1
  )" || {
    printf '%s\n' "${output}" >&2
    return 1
  }
  printf '%s\n' "${output}"
  grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}"
}

try_set_bool() {
  call_set_bool "$1" "$2" || {
    echo "WARNING: service request failed: $1 -> $2" >&2
    return 1
  }
}

start_component() {
  local name="$1"
  shift
  local log_file pid_file
  log_file="$(component_log_file "${name}")"
  pid_file="$(component_pid_file "${name}")"
  nohup setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  local pid=$!
  printf '%s\n' "${pid}" >"${pid_file}"
  echo "started ${name}: pid=${pid} log=${log_file}"
}

stop_component() {
  local name="$1"
  local pid_file pid
  pid_file="$(component_pid_file "${name}")"
  [[ -s "${pid_file}" ]] || return 0
  pid="$(<"${pid_file}")"
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    local deadline=$((SECONDS + 5))
    while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
      sleep 0.10
    done
    if kill -0 "${pid}" 2>/dev/null; then
      echo "WARNING: ${name} did not exit after SIGINT; sending SIGTERM" >&2
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
      local term_deadline=$((SECONDS + 3))
      while kill -0 "${pid}" 2>/dev/null && (( SECONDS < term_deadline )); do
        sleep 0.10
      done
    fi
    if kill -0 "${pid}" 2>/dev/null; then
      echo "ERROR: ${name} is still running with pid=${pid}." >&2
      return 1
    fi
  fi
  rm -f -- "${pid_file}"
}

show_logs() {
  local name
  for name in arm gripper bridge; do
    local log_file
    log_file="$(component_log_file "${name}")"
    if [[ -r "${log_file}" ]]; then
      echo
      echo "--- ${name} log (last 20 lines) ---"
      tail -n 20 "${log_file}"
    fi
  done
}

known_conflicts() {
  local matches nodes owners real_device
  matches="$(
    pgrep -af \
      'robot_timer|test_joint_trajectory_sub|gripper_controller|safe_one_arm_servo|safe_gripper_controller|udp_leader_bridge' \
      2>/dev/null || true
  )"
  if [[ -n "${matches}" ]]; then
    echo "ERROR: a known robot/gripper process is already running:" >&2
    printf '%s\n' "${matches}" >&2
    return 1
  fi

  nodes="$(ros2 node list 2>/dev/null || true)"
  if grep -Eq \
    '^/(robot_timer|gripper_controller|safe_one_arm_servo|safe_gripper_controller|udp_leader_bridge)$' \
    <<<"${nodes}"; then
    echo "ERROR: a conflicting ROS 2 node is visible:" >&2
    printf '%s\n' "${nodes}" >&2
    return 1
  fi

  if [[ ! -e "${GRIPPER_DEVICE}" ]]; then
    echo "ERROR: CTAG2F120 serial path is missing: ${GRIPPER_DEVICE}" >&2
    return 1
  fi
  real_device="$(readlink -f "${GRIPPER_DEVICE}")"
  owners="$(fuser "${real_device}" 2>/dev/null || true)"
  if [[ -n "${owners//[[:space:]]/}" ]]; then
    echo "ERROR: CTAG2F120 serial device is busy: ${real_device}" >&2
    ps -fp ${owners} 2>/dev/null || true
    return 1
  fi
}

ensure_components_running() {
  local name
  for name in arm gripper bridge; do
    if ! component_alive "${name}"; then
      echo "ERROR: ${name} component is not running." >&2
      show_logs
      return 1
    fi
  done
}

safe_hardware_shutdown() {
  local failures=0
  try_set_bool /teleop/set_enabled false || failures=1
  try_set_bool /right_arm/set_motion_enabled false || failures=1
  try_set_bool /right_arm/set_gripper_enabled false || failures=1
  try_set_bool /right_arm/set_robot_enabled false || failures=1
  wait_status_field "robot_enabled=0" 10 || failures=1
  try_set_bool /right_arm/set_powered_on false || failures=1
  wait_status_field "robot_powered_on=0" 15 || failures=1
  return "${failures}"
}

start_stack() {
  if component_alive arm || component_alive gripper || component_alive bridge; then
    echo "ERROR: this launcher's stack is already running. Run stop first." >&2
    exit 3
  fi
  known_conflicts
  [[ -r "${ARM_CONFIG}" ]] || {
    echo "ERROR: missing config: ${ARM_CONFIG}" >&2
    exit 2
  }
  [[ -r "${BRIDGE_CONFIG}" ]] || {
    echo "ERROR: missing config: ${BRIDGE_CONFIG}" >&2
    exit 2
  }
  ping -c 1 -W 1 192.168.2.226 >/dev/null || {
    echo "ERROR: Armstrong controller 192.168.2.226 is unreachable." >&2
    exit 4
  }

  start_component arm \
    ros2 run servo_controller safe_one_arm_servo --ros-args \
      --params-file "${ARM_CONFIG}" \
      -p dry_run:=false \
      -p hardware_power_authorized:=true \
      -p hardware_enable_authorized:=true \
      -p hardware_motion_authorized:=true \
      -p limits_configured:=true
  start_component gripper \
    ros2 run servo_controller safe_gripper_controller --ros-args \
      --params-file "${ARM_CONFIG}" \
      -p dry_run:=false
  start_component bridge \
    ros2 run one_arm_teleop_bridge udp_leader_bridge --ros-args \
      --params-file "${BRIDGE_CONFIG}" \
      -p dry_run:=false \
      -p expected_source_ip:="${EXPECTED_SOURCE_IP}"

  if ! wait_for_service /right_arm/set_powered_on 20 ||
     ! wait_for_service /right_arm/set_gripper_enabled 20 ||
     ! wait_for_service /teleop/set_enabled 20; then
    show_logs
    stop_component bridge || true
    stop_component gripper || true
    stop_component arm || true
    exit 5
  fi
  ensure_components_running
  echo "STACK_STARTED_NO_MOTION"
  echo "Expected Windows UDP source: ${EXPECTED_SOURCE_IP}"
  topic_once /right_arm/safety_status 3 || true
}

wait_for_leader_preview() {
  local deadline=$((SECONDS + 120))
  local output=""
  echo "Waiting for Windows Enter/start-pose capture and fresh UDP preview..."
  while (( SECONDS < deadline )); do
    output="$(topic_once /teleop/bridge_status 2 || true)"
    if grep -Eq 'session=(none)?;' <<<"${output}"; then
      :
    elif grep -Eq 'sequence=[0-9]+' <<<"${output}" &&
         grep -Fq 'deadman_held=False' <<<"${output}"; then
      printf '%s\n' "${output}"
      return 0
    fi
    sleep 0.25
  done
  echo "ERROR: no safe Windows preview arrived within 120 seconds." >&2
  [[ -n "${output}" ]] && printf '%s\n' "${output}" >&2
  return 1
}

verify_robot_safe_to_arm() {
  local status
  status="$(topic_once /right_arm/safety_status 4 || true)"
  printf '%s\n' "${status}"
  local required
  for required in \
    "connected=1" \
    "feedback_valid=1" \
    "robot_emergency_stop=0" \
    "robot_protective_stop=0" \
    "robot_on_soft_limit=0" \
    "robot_socket_connected=1" \
    "robot_error_code=0" \
    "limits_configured=1" \
    "robot_powered_on=0" \
    "robot_enabled=0" \
    "motion_enabled=0" \
    "servo_mode_entered=0"; do
    if ! grep -Fq "${required}" <<<"${status}"; then
      echo "ERROR: robot safety prerequisite is missing: ${required}" >&2
      return 1
    fi
  done
}

arm_stack() {
  ensure_components_running
  wait_for_leader_preview
  verify_robot_safe_to_arm

  echo "Safety checks passed. Applying explicit staged hardware gates..."
  if ! call_set_bool /right_arm/set_powered_on true ||
     ! wait_status_field "robot_powered_on=1" 30 ||
     ! call_set_bool /right_arm/set_robot_enabled true ||
     ! wait_status_field "robot_enabled=1" 30 ||
     ! call_set_bool /right_arm/set_motion_enabled true ||
     ! wait_status_field "motion_enabled=1" 10 ||
     ! call_set_bool /right_arm/set_gripper_enabled true ||
     ! call_set_bool /teleop/set_enabled true; then
    echo "ERROR: arming failed. Running ordered safety shutdown." >&2
    safe_hardware_shutdown || true
    show_logs
    exit 6
  fi

  echo "FULL_TELEOP_READY"
  echo "The robot is servo-ready but receives no motion command until Space."
}

stop_stack() {
  local shutdown_failed=0
  local process_stop_failed=0
  if component_alive arm || component_alive gripper || component_alive bridge; then
    safe_hardware_shutdown || shutdown_failed=1
  fi
  stop_component bridge || process_stop_failed=1
  stop_component gripper || process_stop_failed=1
  stop_component arm || process_stop_failed=1
  if (( shutdown_failed != 0 || process_stop_failed != 0 )); then
    echo "ERROR: software shutdown or process exit was not fully confirmed." >&2
    echo "Press the physical emergency stop and inspect the logs." >&2
    exit 7
  fi
  echo "FULL_TELEOP_STOPPED"
}

show_status() {
  local name
  for name in arm gripper bridge; do
    if component_alive "${name}"; then
      echo "${name}: running pid=$(<"$(component_pid_file "${name}")")"
    else
      echo "${name}: stopped"
    fi
  done
  echo
  ros2 node list 2>/dev/null || true
  echo
  topic_once /right_arm/safety_status 2 || true
  topic_once /right_arm/gripper_status 2 || true
  topic_once /teleop/bridge_status 2 || true
}

case "${ACTION}" in
  start)
    start_stack
    ;;
  arm)
    arm_stack
    ;;
  stop)
    stop_stack
    ;;
  status)
    show_status
    ;;
  *)
    echo "Usage: $0 {start|arm|stop|status} [expected_windows_source_ip]" >&2
    exit 2
    ;;
esac
