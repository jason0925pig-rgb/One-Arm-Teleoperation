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
- 循环编码器连续展开与异常跳变检查；
- 同时采集主臂起点 `p0` 和从臂起点 `q0`；
- `q_target = q0 + sign * scale * (p - p0)`；
- 逐关节软限位和主从状态超时检查；
- 发布目标预览，以及解锁后发布真实 ROS2 目标。

### Ubuntu ROS2 执行层

`servo_controller/safe_one_arm_servo` 只建立一条机械臂连接，完成：

- 按关节名称重新排序；
- 七轴数量、NaN/Inf、软限位的第二次检查；
- 每关节最大速度限制；
- 300 ms 命令看门狗；
- 每次通过 SDK 重新读取真实关节状态；
- 显式服务解锁和退出伺服模式。

它不会在启动时自动上电、使能或进入伺服模式。

`servo_controller/safe_gripper_controller` 当前配置为知行（ZhiXing/
ChangingTek）CTAG2F120，内部沿用历史驱动名 `ZX`，支持
速度、力、位置、到位/夹持反馈和运动超时。型号、端点未配置时拒绝解锁。

## 2. 默认状态为何一定不会驱动真机

两个 YAML 都故意不可用于真机：

- `dry_run: true`
- `calibration_complete: false`
- `limits_configured: false`
- 关节比例为 0
- 关节上下限尚未填写
- `gripper_type: zx`，`gripper_model: CTAG2F120` 已确认；
- `configuration_complete: false`
- `configuration_complete: false`

即使误调用解锁服务，程序也会返回失败。

## 3. Ubuntu 22 / ROS2 构建

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

第一轮仍保持 `dry_run: true`，只检查 `/teleop/target_preview`。

## 6. 双重解锁

真机配置完成且现场急停可用后，先启动执行端运动门：

```bash
ros2 service call /right_arm/set_motion_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

再启动主从映射门：

```bash
ros2 service call /teleop/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

第二个服务成功时，桥接节点同时记录当前主臂 `p0` 和当前从臂 `q0`，
这就是带起始偏移的绝对控制零点。

停止顺序相反：

```bash
ros2 service call /teleop/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /right_arm/set_motion_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

任何数据超时、越界、非有限数或 SDK 失败都会关闭运动门。

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

遥操作不要求必须录制。为训练采集时，Ubuntu 用 rosbag2 统一记录：

```bash
ros2 bag record \
  /teleop/leader_pulses \
  /teleop/target_preview \
  /right_arm/teleop_joint_command \
  /right_arm/joint_states \
  /right_arm/gripper_command \
  /right_arm/gripper_state \
  /camera/color/image_raw \
  /camera/depth/image_raw
```

相机数据建议直接保存在 Ubuntu，不实时传回 Windows。Windows CSV 是主臂
原始数据备份；Ubuntu rosbag 保存动作、从臂反馈和视频的统一时间线。
