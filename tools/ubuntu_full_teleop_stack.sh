#!/usr/bin/env bash
# Attended one-arm teleoperation lifecycle for the Armstrong Ubuntu host.
# Starting processes is motion-free. Hardware motion is authorized only by
# the explicit "arm" action after leader preview and robot safety checks pass.

set -Eeo pipefail

ACTION="${1:-status}"
EXPECTED_SOURCE_IP="${2:-${ONE_ARM_WINDOWS_IP:-192.168.2.130}}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"
RUNTIME_DIR="/tmp/one_arm_teleop_full_${UID}"
MANUAL_RUNTIME_DIR="/tmp/one_arm_manual_mode_${UID}"
MANUAL_LOCK_DIR="${MANUAL_RUNTIME_DIR}.lock"
MANUAL_PID_FILE="${MANUAL_RUNTIME_DIR}/arm.pid"
GRIPPER_DEVICE="${ONE_ARM_GRIPPER_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5ABB000800-if00}"
ARM_CONFIG="${PROJECT_ROOT}/servo_controller/config/full_teleop_attended.yaml"
BRIDGE_CONFIG="${PROJECT_ROOT}/one_arm_teleop_bridge/config/full_teleop_attended.yaml"
CONTROL_CPU="${ONE_ARM_CONTROL_CPU:-1}"
BACKGROUND_CPU_LIST="${ONE_ARM_BACKGROUND_CPUS:-}"

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

# Interrupted launches can leave the ROS 2 CLI daemon with stale graph state.
# Stopping it does not stop ROS nodes; it only forces the next `ros2 ...`
# diagnostic call to reconnect to the live graph.
ros2 daemon stop >/dev/null 2>&1 || true

configure_cpu_sets() {
  local cpu_count
  cpu_count="$(nproc)"
  if [[ ! "${CONTROL_CPU}" =~ ^[0-9]+$ ]] ||
     (( CONTROL_CPU >= cpu_count )); then
    echo "ERROR: control CPU ${CONTROL_CPU} is invalid for ${cpu_count} CPUs." >&2
    return 2
  fi
  if [[ -z "${BACKGROUND_CPU_LIST}" ]]; then
    if (( cpu_count >= 3 )); then
      BACKGROUND_CPU_LIST="2-$((cpu_count - 1))"
    else
      BACKGROUND_CPU_LIST="0"
    fi
  fi
  taskset -c "${CONTROL_CPU}" true >/dev/null
  taskset -c "${BACKGROUND_CPU_LIST}" true >/dev/null
}

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
  python3 "${PROJECT_ROOT}/tools/wait_for_string_topic.py" \
    "${topic_name}" --timeout "${timeout_seconds}"
}

wait_status_field() {
  local expected="$1"
  local timeout_seconds="${2:-20}"
  python3 "${PROJECT_ROOT}/tools/wait_for_string_topic.py" \
    /right_arm/safety_status \
    --timeout "${timeout_seconds}" \
    --contains "${expected}"
}

wait_status_fields() {
  local timeout_seconds="$1"
  shift
  local -a arguments=(
    "${PROJECT_ROOT}/tools/wait_for_string_topic.py"
    /right_arm/safety_status
    --timeout "${timeout_seconds}"
  )
  local expected
  for expected in "$@"; do
    arguments+=(--contains "${expected}")
  done
  python3 "${arguments[@]}"
}

wait_gripper_fields() {
  local timeout_seconds="$1"
  shift
  local -a arguments=(
    "${PROJECT_ROOT}/tools/wait_for_string_topic.py"
    /right_arm/gripper_status
    --timeout "${timeout_seconds}"
  )
  local expected
  for expected in "$@"; do
    arguments+=(--contains "${expected}")
  done
  python3 "${arguments[@]}"
}

wait_attended_servo_ready() {
  local timeout_seconds="${1:-10}"
  local output=""
  output="$(wait_status_fields "${timeout_seconds}" \
    "motion_enabled=1" \
    "robot_error_code=0" \
    "robot_emergency_stop=0" \
    "robot_protective_stop=0" \
    "robot_on_soft_limit=0")" || return 1
  printf '%s\n' "${output}"
  if ! grep -Fq "servo_mode_entered=1" <<<"${output}"; then
    echo "WARNING: servo_mode_entered=1 was not observed in this status sample; continuing because motion service succeeded and robot safety fields are healthy." >&2
  fi
}

initialize_gripper_open() {
  echo "INITIAL_GRIPPER_OPENING: follower gripper will move to its configured open position."
  if ! call_set_bool /right_arm/set_gripper_open true; then
    echo "ERROR: gripper node did not accept the initial OPEN command." >&2
    return 1
  fi
  # The installed CTAG2F120 can report a stale/endpoint torque bit and a
  # measured position different from the commanded endpoint.  Do not require
  # an exact position or contact=0 here.  Hardware-command acknowledgement,
  # continued enable state, and completion of the attempt are the reliable
  # startup fields available from this firmware.
  if ! wait_gripper_fields 8 \
      "enabled=1" \
      "moving=0" \
      "requested_open=1"; then
    return 1
  fi
  echo "INITIAL_GRIPPER_OPEN_COMMAND_CONFIRMED"
  echo "WARNING: visually confirm that the follower gripper is open before pressing Space."
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

stop_manual_mode_for_handoff() {
  if [[ ! -s "${MANUAL_PID_FILE}" ]]; then
    # Remove only a stale lock created by our own manual-mode launcher.
    if pgrep -f '[r]ight_arm_manual_mode.sh' >/dev/null 2>&1; then
      echo "ERROR: right-arm manual mode is still starting; stop it or retry shortly." >&2
      return 1
    fi
    if [[ -d "${MANUAL_LOCK_DIR}" ]]; then
      rmdir -- "${MANUAL_LOCK_DIR}" 2>/dev/null || true
    fi
    return 0
  fi

  local manual_pid manual_command
  manual_pid="$(<"${MANUAL_PID_FILE}")"
  if [[ ! "${manual_pid}" =~ ^[0-9]+$ ]] ||
     ! kill -0 "${manual_pid}" 2>/dev/null; then
    rm -f -- "${MANUAL_PID_FILE}"
    rmdir -- "${MANUAL_LOCK_DIR}" 2>/dev/null || true
    return 0
  fi

  manual_command="$(tr '\0' ' ' <"/proc/${manual_pid}/cmdline" 2>/dev/null || true)"
  if [[ "${manual_command}" != *"safe_one_arm_servo"* ]] ||
     [[ "${manual_command}" != *"hardware_motion_authorized:=false"* ]]; then
    echo "ERROR: manual-mode PID file points to an unexpected process:" >&2
    ps -fp "${manual_pid}" >&2 || true
    return 1
  fi

  echo "MANUAL_MODE_HANDOFF: stopping drag/manual mode before full teleoperation."
  echo "The arm will be disabled and powered off briefly; no motion command is sent."
  wait_for_service /right_arm/set_drag_enabled 5
  wait_for_service /right_arm/set_motion_enabled 5
  wait_for_service /right_arm/set_robot_enabled 5
  wait_for_service /right_arm/set_powered_on 5
  try_set_bool /right_arm/set_drag_enabled false
  try_set_bool /right_arm/set_motion_enabled false
  try_set_bool /right_arm/set_robot_enabled false
  wait_status_field "robot_enabled=0" 15
  try_set_bool /right_arm/set_powered_on false
  wait_status_field "robot_powered_on=0" 20

  kill -INT -- "-${manual_pid}" 2>/dev/null ||
    kill -INT "${manual_pid}" 2>/dev/null || true
  local deadline=$((SECONDS + 8))
  while kill -0 "${manual_pid}" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 0.10
  done
  if kill -0 "${manual_pid}" 2>/dev/null; then
    kill -TERM -- "-${manual_pid}" 2>/dev/null ||
      kill -TERM "${manual_pid}" 2>/dev/null || true
    sleep 1
  fi
  if kill -0 "${manual_pid}" 2>/dev/null; then
    echo "ERROR: manual-mode process did not exit: pid=${manual_pid}" >&2
    return 1
  fi

  # If the interactive wrapper is still alive, let its EXIT cleanup finish
  # before a new node exposes services with the same names.
  deadline=$((SECONDS + 20))
  while pgrep -f '[r]ight_arm_manual_mode.sh' >/dev/null 2>&1 &&
        (( SECONDS < deadline )); do
    sleep 0.20
  done
  if pgrep -f '[r]ight_arm_manual_mode.sh' >/dev/null 2>&1; then
    echo "ERROR: manual-mode wrapper did not finish its shutdown." >&2
    return 1
  fi

  rm -f -- "${MANUAL_PID_FILE}"
  rmdir -- "${MANUAL_LOCK_DIR}" 2>/dev/null || true
  echo "MANUAL_MODE_HANDOFF_COMPLETE"
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
  local candidates matches nodes owners real_device topic info count
  candidates="$(
    pgrep -af \
      'robot_timer|test_joint_trajectory_sub|gripper_controller|safe_one_arm_servo|safe_gripper_controller|udp_leader_bridge|tele_robot' \
      2>/dev/null || true
  )"
  # Only treat an actual installed executable or an explicit ros2 run/launch
  # command as a process conflict. A grep/editor merely reading a source path
  # such as .../src/gripper_controller/... must not block startup.
  matches="$(
    grep -E \
      '(/install/[^ ]*/lib/[^ ]*/(robot_timer|test_joint_trajectory_sub|gripper_controller|safe_one_arm_servo|safe_gripper_controller|udp_leader_bridge)([[:space:]]|$)|(^|[[:space:]])ros2[[:space:]]+(run|launch)[[:space:]]+(servo_controller|gripper_controller|one_arm_teleop_bridge)([[:space:]]|$)|(^|[[:space:]])(\./|/[^ ]*/)(tele_robot|robot_timer|test_joint_trajectory_sub)([[:space:]]|$))' \
      <<<"${candidates}" || true
  )"
  if [[ -n "${matches}" ]]; then
    echo "ERROR: a known robot/gripper process is already running:" >&2
    printf '%s\n' "${matches}" >&2
    return 1
  fi

  nodes="$(ros2 node list 2>/dev/null || true)"
  if grep -Eq \
    '^/(robot_timer|gripper_controller|test_joint_trajectory_sub|test_joint_trajectory_subscriber|safe_one_arm_servo|safe_gripper_controller|udp_leader_bridge)$' \
    <<<"${nodes}"; then
    echo "ERROR: a conflicting ROS 2 node is visible:" >&2
    printf '%s\n' "${nodes}" >&2
    return 1
  fi

  for topic in \
    /right_arm/teleop_joint_command \
    /right_arm/joint_control \
    /right_arm/gripper_command \
    /gripper_position; do
    info="$(ros2 topic info "${topic}" 2>/dev/null || true)"
    count="$(sed -n 's/^Publisher count: //p' <<<"${info}")"
    count="${count:-0}"
    if [[ "${count}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
      echo "ERROR: ${topic} already has ${count} publisher(s)." >&2
      return 1
    fi
  done

  if [[ ! -e "${GRIPPER_DEVICE}" ]]; then
    echo "ERROR: CTAG2F120 serial path is missing: ${GRIPPER_DEVICE}" >&2
    return 1
  fi
  real_device="$(readlink -f "${GRIPPER_DEVICE}")"
  if [[ ! -r "${real_device}" || ! -w "${real_device}" ]]; then
    echo "ERROR: CTAG2F120 serial device is not readable/writable by user ${USER}: ${real_device}" >&2
    echo "Run once: sudo usermod -aG dialout ${USER}" >&2
    echo "Then log out of every SSH session and log in again (or reboot)." >&2
    ls -l "${real_device}" >&2 || true
    id >&2 || true
    return 1
  fi
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
  stop_manual_mode_for_handoff
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
  configure_cpu_sets

  start_component arm \
    taskset -c "${CONTROL_CPU}" \
      ros2 run servo_controller safe_one_arm_servo --ros-args \
      --params-file "${ARM_CONFIG}" \
      -p dry_run:=false \
      -p hardware_power_authorized:=true \
      -p hardware_enable_authorized:=true \
      -p hardware_motion_authorized:=true \
      -p limits_configured:=true
  start_component gripper \
    taskset -c "${BACKGROUND_CPU_LIST}" \
      ros2 run servo_controller safe_gripper_controller --ros-args \
      --params-file "${ARM_CONFIG}" \
      -p dry_run:=false \
      -p port:="${GRIPPER_DEVICE}"
  start_component bridge \
    taskset -c "${BACKGROUND_CPU_LIST}" \
      ros2 run one_arm_teleop_bridge udp_leader_bridge --ros-args \
      --params-file "${BRIDGE_CONFIG}" \
      -p dry_run:=false \
      -p expected_source_ip:="${EXPECTED_SOURCE_IP}"

  if ! wait_for_service /right_arm/set_powered_on 20 ||
     ! wait_for_service /right_arm/set_gripper_enabled 20 ||
     ! wait_for_service /right_arm/set_gripper_open 20 ||
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
  echo "CPU placement: arm=${CONTROL_CPU} background=${BACKGROUND_CPU_LIST}"
  topic_once /right_arm/safety_status 3 || true
}

wait_for_leader_preview() {
  local output=""
  echo "Waiting for Windows Enter/start-pose capture and fresh UDP preview..."
  output="$(python3 "${PROJECT_ROOT}/tools/wait_for_string_topic.py" \
    /teleop/bridge_status \
    --timeout 120 \
    --contains 'deadman_held=False' \
    --regex 'session=(?!none(?:;|$))[^;]+' \
    --regex 'sequence=[0-9]+' \
    --regex 'consecutive_accepted_packets=([5-9]|[1-9][0-9]+)')" && {
      printf '%s\n' "${output}"
      return 0
    }
  echo "ERROR: no safe Windows preview arrived within 120 seconds." >&2
  if [[ -n "${output}" ]]; then
    echo "--- last /teleop/bridge_status sample ---" >&2
    printf '%s\n' "${output}" >&2
  else
    echo "No /teleop/bridge_status sample was readable." >&2
  fi
  echo "--- bridge log tail ---" >&2
  tail -n 120 "$(component_log_file bridge)" >&2 || true
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
    "motion_enabled=0" \
    "servo_mode_entered=0"; do
    if ! grep -Fq "${required}" <<<"${status}"; then
      echo "ERROR: robot safety prerequisite is missing: ${required}" >&2
      return 1
    fi
  done
  echo "Existing robot power/enable state is accepted; shutdown will disable and power off."
}

arm_stack() {
  ensure_components_running
  if ! wait_for_leader_preview; then
    echo "ERROR: arming stopped before hardware power/enable because leader UDP preview was not accepted." >&2
    exit 8
  fi
  verify_robot_safe_to_arm

  echo "Safety checks passed. Applying explicit staged hardware gates..."
  local status
  status="$(topic_once /right_arm/safety_status 4 || true)"
  if grep -Fq "robot_powered_on=1" <<<"${status}"; then
    echo "Robot is already powered on; skipping the power-on request."
  elif ! call_set_bool /right_arm/set_powered_on true ||
       ! wait_status_field "robot_powered_on=1" 30; then
    echo "ERROR: arming failed during robot power-on." >&2
    safe_hardware_shutdown || true
    show_logs
    exit 6
  fi

  status="$(topic_once /right_arm/safety_status 4 || true)"
  if grep -Fq "robot_enabled=1" <<<"${status}"; then
    echo "Robot is already enabled; skipping the robot-enable request."
  elif ! call_set_bool /right_arm/set_robot_enabled true ||
       ! wait_status_field "robot_enabled=1" 30; then
    echo "ERROR: arming failed during robot enable." >&2
    safe_hardware_shutdown || true
    show_logs
    exit 6
  fi

  # Arm the mapper first. Released preview frames are intentionally ignored,
  # so this cannot publish a motion target before Windows Space. This ordering
  # also prevents the servo gate from sitting open while later startup work
  # competes for CPU time.
  if ! call_set_bool /teleop/set_enabled true ||
     ! call_set_bool /right_arm/set_gripper_enabled true ||
     ! initialize_gripper_open ||
     ! call_set_bool /right_arm/set_motion_enabled true ||
     ! wait_status_field "motion_enabled=1" 10; then
    echo "ERROR: arming failed. Running ordered safety shutdown." >&2
    safe_hardware_shutdown || true
    show_logs
    exit 6
  fi

  # A service success is not enough: keep observing the real node after motion
  # is enabled. Do not require servo_mode_entered=1 to appear in the same
  # status sample; this SDK/node combination can report the service success
  # before that field is visible in /right_arm/safety_status.
  sleep 1
  ensure_components_running
  local armed_status
  if ! armed_status="$(wait_attended_servo_ready 10)"; then
    echo "ERROR: servo gate did not pass attended ready checks." >&2
    safe_hardware_shutdown || true
    show_logs
    exit 6
  fi
  printf '%s\n' "${armed_status}"

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
  preflight)
    known_conflicts
    echo "PREFLIGHT_OK"
    ;;
  *)
    echo "Usage: $0 {start|arm|stop|status|preflight} [expected_windows_source_ip]" >&2
    exit 2
    ;;
esac
