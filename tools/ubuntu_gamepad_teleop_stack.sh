#!/usr/bin/env bash
# Attended lifecycle for a Windows HID gamepad connected directly to the Orin
# over UDP. The local bridge owns the sole teleop_joint_command publisher.
set -Eeo pipefail

ACTION="${1:-status}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"
RUNTIME_DIR="/tmp/one_arm_gamepad_teleop_${UID}"
GRIPPER_DEVICE="${ONE_ARM_GRIPPER_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
ARM_CONFIG="${PROJECT_ROOT}/servo_controller/config/full_teleop_attended.yaml"
CONTROL_CPU="${ONE_ARM_CONTROL_CPU:-1}"
BACKGROUND_CPU_LIST="${ONE_ARM_BACKGROUND_CPUS:-}"
GAMEPAD_SOURCE_IP="${ONE_ARM_GAMEPAD_SOURCE_IP:-192.168.2.131}"
GAMEPAD_UDP_PORT="${ONE_ARM_GAMEPAD_UDP_PORT:-5010}"
mkdir -p "${RUNTIME_DIR}"

for setup in "${ROS_SETUP}" "${WORKSPACE_SETUP}"; do
  [[ -r "${setup}" ]] || { echo "ERROR: required setup is missing: ${setup}" >&2; exit 2; }
done
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"

pid_file() { printf '%s/%s.pid\n' "${RUNTIME_DIR}" "$1"; }
log_file() { printf '%s/%s.log\n' "${RUNTIME_DIR}" "$1"; }
alive() { local p; p="$(pid_file "$1")"; [[ -s "${p}" ]] && kill -0 "$(<"${p}")" 2>/dev/null; }
start_component() {
  local name="$1"; shift
  nohup setsid "$@" >"$(log_file "${name}")" 2>&1 < /dev/null &
  printf '%s\n' "$!" >"$(pid_file "${name}")"
  echo "started ${name}: pid=$!"
}
stop_component() {
  local name="$1" p pid deadline
  p="$(pid_file "${name}")"; [[ -s "${p}" ]] || return 0; pid="$(<"${p}")"
  if kill -0 "${pid}" 2>/dev/null; then
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    deadline=$((SECONDS + 8)); while kill -0 "${pid}" 2>/dev/null && ((SECONDS < deadline)); do sleep .1; done
  fi
  if kill -0 "${pid}" 2>/dev/null; then kill -TERM -- "-${pid}" 2>/dev/null || true; fi
  rm -f -- "${p}"
}
wait_service() {
  local name="$1" deadline=$((SECONDS + ${2:-20}))
  while ((SECONDS < deadline)); do ros2 service list 2>/dev/null | grep -Fxq "${name}" && return 0; sleep .25; done
  echo "ERROR: service did not appear: ${name}" >&2; return 1
}
call_bool() {
  timeout 15 ros2 service call "$1" std_srvs/srv/SetBool "{data: $2}" 2>&1 | tee /dev/stderr | grep -Eq 'success[=:][[:space:]]*(true|True)'
}
# The safety payload is deliberately long. Without --full-length, ros2 CLI
# abbreviates it with "...", which makes healthy fields such as connected=1
# invisible to require_healthy and causes a false startup rejection.
status() { timeout 4 ros2 topic echo --once --full-length /right_arm/safety_status 2>/dev/null || true; }
wait_status() {
  local token="$1" deadline=$((SECONDS + ${2:-15})) out
  # A fresh ros2 CLI subscriber can first receive the previous status sample
  # (for example motion_enabled=0 immediately after enabling). Keep one
  # subscriber open until the requested current state actually arrives.
  while ((SECONDS < deadline)); do
    out="$(timeout 3 bash -o pipefail -c "ros2 topic echo --full-length /right_arm/safety_status 2>/dev/null | grep -m1 -F '${token}'" || true)"
    [[ -n "${out}" ]] && { printf '%s\n' "${out}"; return 0; }
  done
  echo "ERROR: safety status did not reach ${token}" >&2; return 1
}
require_healthy() {
  local s; s="$(status)"
  for token in connected=1 feedback_valid=1 robot_emergency_stop=0 robot_protective_stop=0 robot_on_soft_limit=0 robot_error_code=0 limits_configured=1; do
    grep -Fq "${token}" <<<"${s}" || { echo "ERROR: missing safety prerequisite ${token}" >&2; printf '%s\n' "${s}" >&2; return 1; }
  done
}
publish_stop() { timeout 5 ros2 topic pub --once /teleop/stop_request std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; }
close_motion() { publish_stop; call_bool /right_arm/set_motion_enabled false || true; }
shutdown_hardware() {
  call_bool /teleop/set_enabled false || true
  close_motion
  call_bool /right_arm/set_gripper_enabled false || true
  call_bool /right_arm/set_robot_enabled false || true
  call_bool /right_arm/set_powered_on false || true
}
configure_cpu_sets() {
  local n; n="$(nproc)"; [[ "${CONTROL_CPU}" =~ ^[0-9]+$ ]] && ((CONTROL_CPU < n)) || { echo "ERROR: invalid control CPU" >&2; exit 2; }
  [[ -n "${BACKGROUND_CPU_LIST}" ]] || BACKGROUND_CPU_LIST=$([[ "${n}" -ge 3 ]] && echo "2-$((n-1))" || echo 0)
}
known_conflict() {
  if pgrep -af 'safe_one_arm_servo|safe_gripper_controller|udp_leader_bridge|udp_gamepad_bridge.py' >/dev/null; then
    echo "ERROR: a robot/teleop process is already running; use stop or finish the active session first." >&2
    pgrep -af 'safe_one_arm_servo|safe_gripper_controller|udp_leader_bridge|udp_gamepad_bridge.py' >&2 || true
    return 1
  fi
  [[ -e "${GRIPPER_DEVICE}" ]] || { echo "ERROR: CTAG2F120 path is missing: ${GRIPPER_DEVICE}" >&2; return 1; }
}
start_stack() {
  (alive arm || alive gripper || alive bridge) && { echo "ERROR: gamepad stack already running" >&2; exit 3; }
  known_conflict
  ping -c1 -W1 192.168.2.226 >/dev/null || { echo "ERROR: robot controller .226 unreachable" >&2; exit 4; }
  configure_cpu_sets
  start_component arm taskset -c "${CONTROL_CPU}" ros2 run servo_controller safe_one_arm_servo --ros-args --params-file "${ARM_CONFIG}" -p dry_run:=false -p hardware_power_authorized:=true -p hardware_enable_authorized:=true -p hardware_motion_authorized:=true -p limits_configured:=true
  start_component gripper taskset -c "${BACKGROUND_CPU_LIST}" ros2 run servo_controller safe_gripper_controller --ros-args --params-file "${ARM_CONFIG}" -p dry_run:=false -p port:="${GRIPPER_DEVICE}"
  start_component bridge taskset -c "${BACKGROUND_CPU_LIST}" python3 "${PROJECT_ROOT}/tools/udp_gamepad_bridge.py" --port "${GAMEPAD_UDP_PORT}" --expected-source-ip "${GAMEPAD_SOURCE_IP}"
  wait_service /right_arm/set_powered_on && wait_service /right_arm/set_gripper_enabled && wait_service /right_arm/set_gripper_open && wait_service /teleop/set_enabled
  echo "GAMEPAD_STACK_STARTED_NO_MOTION"
  echo "Expected Windows source: ${GAMEPAD_SOURCE_IP}:${GAMEPAD_UDP_PORT}"
}
prepare_stack() {
  alive arm && alive gripper && alive bridge || { echo "ERROR: stack is not running" >&2; exit 1; }
  require_healthy
  # Keep the UDP command gate and servo closed while preparing hardware.
  # The bridge watchdog only runs after its gate opens, so this avoids a
  # false timeout while the operator is still positioning their hands.
  call_bool /teleop/set_enabled false || true
  close_motion
  call_bool /right_arm/set_powered_on true || true; wait_status robot_powered_on=1 30
  call_bool /right_arm/set_robot_enabled true || true; wait_status robot_enabled=1 30
  call_bool /right_arm/set_gripper_enabled true
  call_bool /right_arm/set_gripper_open true
  wait_status motion_enabled=0 10
  echo "GAMEPAD_HARDWARE_PREPARED: powered, enabled and gripper open; press button[4] to enter servo."
}

motion_start() {
  alive arm && alive gripper && alive bridge || { echo "ERROR: stack is not running" >&2; exit 1; }
  require_healthy
  call_bool /teleop/set_enabled true
  call_bool /right_arm/set_motion_enabled true; wait_status motion_enabled=1 10
  echo "GAMEPAD_SERVO_READY"
}
round_stop() { call_bool /teleop/set_enabled false || true; close_motion; wait_status motion_enabled=0 10; echo "GAMEPAD_ROUND_STOPPED_SERVO_EXITED"; }
stop_stack() { shutdown_hardware; stop_component bridge; stop_component gripper; stop_component arm; echo "GAMEPAD_STACK_STOPPED"; }

case "${ACTION}" in
  preflight) known_conflict; echo "GAMEPAD_PREFLIGHT_OK" ;;
  start) start_stack ;;
  # arm/round-arm are retained as compatibility aliases for old callers.
  arm|round-arm|prepare) prepare_stack ;;
  motion-start) motion_start ;;
  round-stop) round_stop ;;
  stop) stop_stack ;;
  status) status ;;
  *) echo "Usage: $0 {preflight|start|prepare|motion-start|round-stop|stop|status}" >&2; exit 2 ;;
esac
