# Offline safety implementation status

This file separates code that can be completed without the Armstrong hardware
from facts that must be measured or approved on the real system.

## Implemented offline

- Independent right-arm power, robot-enable, and servo-motion services.
- Deprecated automatic power/enable parameters are accepted only for
  compatibility and ignored.
- Idempotent UDP STOP packets, repeated by Windows on Esc, Ctrl+C, normal exit,
  program error, or deadman release.
- Space-key hold-to-run deadman for real Windows streaming.
- Immediate ROS2 STOP path to both the arm executor and gripper controller.
- Fixed Windows source-IP gate, packet wall-clock age/future-skew checks,
  session/sequence validation, unwrap jump rejection, and 300 ms watchdogs.
- Exactly-one bridge publisher/subscriber checks before real servo motion.
- Configurable median, low-pass, and encoder deadband filters; neutral defaults
  preserve current data until calibration supplies tuning evidence.
- Joint position/NaN checks, velocity and acceleration limiting, braking-aware
  slew planning, feedback watchdog, and 8 ms deadline statistics/abort.
- Fail-stop SDK disconnect handling plus an operator-only reconnect service
  that is locked unless power, enable, servo, and motion are all off.
- Stable CTAG2F120 `/dev/serial/by-id` path in configuration, while motion
  remains locked because endpoints are still unknown.
- Read-only Ubuntu process/ROS topic/serial conflict preflight.
- Passive ROS2 episode recorder for leader, target, follower, gripper, safety,
  STOP, and optional RGB/depth topics.
- Real right-arm readback verified on 2026-07-29 through
  `get_robot_status`: non-zero seven-joint feedback remains available while
  the robot is powered and explicitly disabled.
- Independent robot-disable service verified: the final observed state was
  `powered_on=true`, `robot_enabled=false`, `motion_enabled=false`.
- Legacy Noitom/Axis Studio teleoperation evidence audited and sanitized in
  `LEGACY_MOCAP_TELEOP_AUDIT.md`.

## Intentionally still locked

The repository keeps the following values unusable instead of guessing:

- seven real task-safe lower/upper joint limits;
- seven leader-to-follower signs;
- seven `scale_rad_per_pulse` values;
- CTAG2F120 open/closed positions, direction, safe speed/force, and
  torque-reached behavior;
- the actual Windows source IPv4 address;
- camera topic names and dataset success/failure labels.

Therefore the checked-in defaults remain `dry_run: true`,
`calibration_complete: false`, `limits_configured: false`,
all hardware authorizations false, mapping scales zero, and gripper
`configuration_complete: false`.

## Requires the real robot or site geometry

- Confirm controller soft limits and choose narrower task limits.
- Calibrate each joint one at a time and validate direction/scale.
- Verify the legacy CTAG2F120 candidates (`open=0`, `closed=12000`) at
  operator-chosen safe poses and confirm torque-reached behavior.
- Validate STOP latency, watchdog behavior, and 125 Hz deadline statistics
  under the actual Ubuntu/Pi0/camera load.
- Add self-collision, table, base, and workspace constraints only after the
  correct right-arm kinematic model and site geometry are available.
- Discover the actual RGB/depth topics and measure Windows/Ubuntu clock offset.

The software STOP/deadman is an additional layer. It never replaces the
Armstrong physical emergency stop or an operator at the robot.
