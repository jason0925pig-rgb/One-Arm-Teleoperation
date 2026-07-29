# One-Arm-Teleoperation：Windows 到 Armstrong ROS2 全流程

首次连接请优先按仓库根目录的 [FAST_START.md](../FAST_START.md) 操作；其中包含
Windows/Ubuntu 分机命令、A–H 暂停点、URDF 限位提取和只读夹爪反馈标定。

## 1. 三层职责

### Windows 主臂层

`tools/zlink2_leader_recorder.py` 只通过 ZLink2 发送 `PRAD` 查询，读取
`joint_1..joint_7 + gripper`。它始终保存原始 CSV；指定 `--udp-target`
时，还会把完整帧发送到 Ubuntu。UDP 发送本身不授权机器人运动。

### Ubuntu ROS2 映射层

`one_arm_teleop_bridge/udp_leader_bridge` 接收 UDP，完成：

- 协议、会话、序号、七轴顺序检查；
- 固定 Windows 来源 IP、包时间戳和 deadman 检查；
- 循环编码器连续展开与异常跳变检查；
- 可配置中值、低通和小抖动死区滤波；
- 同时采集主臂起点 `p0` 和从臂起点 `q0`；
- `q_target = q0 + sign * scale * (p - p0)`；
- 逐关节软限位和主从状态超时检查；
- 发布目标预览，以及解锁后发布真实 ROS2 目标。
- 接收 Windows STOP 或看门狗超时后立即关闭映射并通知执行端。

Windows 每得到一套完整八通道数据才发送一次 UDP。ZLink2 实测完整轮询约
58 ms，因此新数据约为 15–17 Hz；桥接不会把重复旧帧伪装成 8 ms 新数据。

### Ubuntu ROS2 执行层

`servo_controller/safe_one_arm_servo` 只建立一条机械臂连接，完成：

- 按关节名称重新排序；
- 七轴数量、NaN/Inf、软限位的第二次检查；
- 每关节最大速度和最大加速度限制；
- 每 8 ms（125 Hz）平滑推进并调用 JAKA `servo_j/edg_servo_j`，
  `step_num=1`；
- 300 ms 命令/反馈看门狗和 8 ms 周期抖动统计；
- 每次通过 SDK 重新读取真实关节状态；
- 独立、显式且默认锁定的驱动上电、机器人使能、伺服运动服务；
- SDK 断线失败停止，以及仅在全断电/未使能时允许的人工重连。

它不会在启动时自动上电、使能或进入伺服模式。

`servo_controller/safe_gripper_controller` 当前配置为知行（ZhiXing/
ChangingTek）CTAG2F120，内部沿用历史驱动名 `ZX`，支持
速度、力、位置、到位/夹持反馈和运动超时。型号、端点未配置时拒绝解锁。

## 2. 默认状态为何一定不会驱动真机

两个 YAML 都故意不可用于真机：

- `dry_run: true`
- `calibration_complete: false`
- `limits_configured: false`
- `hardware_power_authorized: false`
- `hardware_enable_authorized: false`
- `hardware_motion_authorized: false`
- `expected_source_ip` 未配置
- 关节比例为 0
- 关节上下限尚未填写
- `gripper_type: zx`，`gripper_model: CTAG2F120` 已确认；
- `configuration_complete: false`

即使误调用解锁服务，程序也会返回失败。

## 3. Ubuntu 24.04 / ROS2 Jazzy 构建

将仓库放入 Ubuntu 后，在仓库根目录运行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt install libmodbus-dev libyaml-cpp-dev libeigen3-dev
colcon build --packages-select servo_controller one_arm_teleop_bridge
source install/setup.bash
```

只启动新的安全栈：

```bash
ros2 launch one_arm_teleop_bridge full_safe_stack.launch.py
```

不要再启动旧的：

```text
servo_control.launch.py
robot_timer
```

## 4. Windows 发送主臂完整帧

先确认 Ubuntu 地址，例如 `192.168.50.2`。Windows PowerShell：

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation
.\tools\run_zlink2_recorder.cmd `
  --rate-hz 15 `
  --udp-target 192.168.50.2:5005 `
  --session-name network_dry_run
```

Windows 仍会在 `recordings/` 保存 `frames.csv` 和 `metadata.json`。

Ubuntu 验证网络数据：

```bash
ros2 topic echo /teleop/leader_pulses
ros2 topic echo /teleop/bridge_status
ros2 topic echo /right_arm/joint_states
```

此阶段机器人不会运动。

### 4.1 独立的仅上电诊断

`/right_arm/set_powered_on` 与运动授权完全分离。只有启动参数显式设置
`hardware_power_authorized: true`，并且控制器反馈新鲜、通信正常、急停和保护停
未触发、错误码为零、机器人未使能时，`{data: true}` 才会调用一次 JAKA
`power_on()`。

它不会清错、使能、进入伺服模式或发送关节目标，也不要求先填写关节限位。上电
后通过 `/right_arm/powered_on`、`/right_arm/safety_status` 和
`/right_arm/joint_states` 核对状态。`{data: false}` 只允许在机器人未使能且
运动门关闭时执行。

## 5. 真机标定前必须填写

编辑：

```text
one_arm_teleop_bridge/config/teleop_bridge.yaml
servo_controller/config/safe_one_arm.yaml
```

需要从机器人官方资料、URDF 或控制器参数取得：

- 七个关节的真实软限位；
- 每个关节允许的初始最大速度；
- 目标是左臂还是右臂；
- 机器人 IP 和控制端口；
- CTAG2F120 的开/关安全位置、串口、速度和力范围。

然后逐关节标定：

- `joint_signs`
- `scale_rad_per_pulse`

确认无误后才设置：

```yaml
calibration_complete: true
limits_configured: true
```

第一轮仍保持桥接 `dry_run: true`、执行端
`hardware_motion_authorized: false`，只检查 `/teleop/target_preview`。

## 6. 四重解锁

真机配置完成且现场急停可用后，明确设置：

```yaml
# bridge
dry_run: false
expected_source_ip: "<WINDOWS_IP>"

# executor
dry_run: false
hardware_power_authorized: true
hardware_enable_authorized: true
hardware_motion_authorized: true
control_rate_hz: 125.0
```

Windows 发送器必须加 `--deadman` 并按住 Space。现场确认后依次打开四个门：

```bash
ros2 service call /right_arm/set_powered_on \
  std_srvs/srv/SetBool "{data: true}"
ros2 service call /right_arm/set_robot_enabled \
  std_srvs/srv/SetBool "{data: true}"
ros2 service call /right_arm/set_motion_enabled \
  std_srvs/srv/SetBool "{data: true}"
ros2 service call /teleop/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

第二个服务成功时，桥接节点同时记录当前主臂 `p0` 和当前从臂 `q0`，
这就是带起始偏移的绝对控制零点。

松开 Space 会先发送 STOP。随后显式关门并取消使能：

```bash
ros2 service call /teleop/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /right_arm/set_motion_enabled \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /right_arm/set_robot_enabled \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /right_arm/set_powered_on \
  std_srvs/srv/SetBool "{data: false}"
```

任何数据超时、来源/时间戳/deadman 异常、越界、非有限数、周期严重超期或 SDK
失败都会关闭运动门。

## 7. 夹爪

夹爪已确认为 CTAG2F120，`gripper_type: zx` 已写入。使用只读反馈探针确认
`open_position`、`closed_position`、`speed`、`force` 后，最后设置：

```yaml
configuration_complete: true
```

解锁：

```bash
ros2 service call /right_arm/set_gripper_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

主臂 `OPEN/CLOSED` 会发布到 `/right_arm/gripper_command`。知行驱动可反馈
位置到达、力矩到达、当前位置和报警。

## 8. 数据记录和视频

遥操作不要求必须录制。为训练采集时，先查出真实 RGB/深度话题，然后使用被动
episode 记录器：

```bash
python3 tools/ros2_episode_recorder.py \
  --name pick_cup_001 \
  --require-topic /right_arm/joint_states \
  --extra-topic /ACTUAL_RGB_TOPIC \
  --extra-topic /ACTUAL_DEPTH_TOPIC
```

相机数据建议直接保存在 Ubuntu，不实时传回 Windows。Windows CSV 是主臂
原始数据备份；Ubuntu rosbag 保存动作、从臂反馈、安全状态、STOP 和视频的统一
时间线，`episode_metadata.json` 保存任务、操作者、结果和停止原因。
