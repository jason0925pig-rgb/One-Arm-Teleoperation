# One-Arm-Teleoperation Fast Start

本指南只适用于当前确定的方案：

```text
Windows ZLink2 主臂
    → UDP
Ubuntu 24.04 / ROS2 Jazzy
    → JAKA SDK
Armstrong 右臂 + 知行 CTAG2F120
```

代码只通过 GitHub 在两台电脑间同步：

https://github.com/jason0925pig-rgb/One-Arm-Teleoperation

看到“暂停点”就先停下，把要求的输出发给我。不要跳过暂停点。

## 0. 先理解 8 ms 和安全门

ZLink2 在 115200 波特率下依次读取 8 个电机，实测完整扫描平均约 58 ms，
极限约 17 Hz，所以 Windows 不能每 8 ms 产生一套新的八电机数据。

本项目的时序是：

```text
Windows：约 15 Hz 读取并发送新数据
Ubuntu：收到新数据后做映射、限位和看门狗检查
执行端：每 8 ms（125 Hz）向 JAKA 输出平滑目标
```

默认配置下，连接机器人和收到消息都不会运动，因为：

- 桥接层 `dry_run: true`
- 桥接层 `calibration_complete: false`
- 执行层 `limits_configured: false`
- 执行层 `hardware_power_authorized: false`
- 执行层 `hardware_enable_authorized: false`
- 执行层 `hardware_motion_authorized: false`
- 执行层 `motion_enabled: false`
- 桥接层要求配置固定 Windows 来源 IP 和有效数据包时间戳
- 真实运动要求 Windows `--deadman`，松开 Space 立即发送 STOP
- 启动时不自动上电、不自动使能、不自动进入伺服模式
- 夹爪 `configuration_complete: false`

真机运动必须在参数复核后，再由操作者依次调用四个显式服务。只接收 UDP、
只读登录 Armstrong、读取关节状态、查看目标预览都不需要打开运动门。

## 1. Windows 本地采集 CSV

电脑：Windows

Armstrong：不需要连接

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation

.\tools\run_zlink2_recorder.cmd --validate-only

.\tools\run_zlink2_recorder.cmd `
  --port COM10 `
  --rate-hz 15 `
  --duration 10 `
  --session-name windows_csv_test
```

第二条命令提示时按回车，并在 10 秒内缓慢转动几个关节。正常结果应包含：

- `Startup check passed: 8/8 IDs replied`
- 每帧 `complete=完整帧数/总帧数`
- `frames.csv`
- `metadata.json`

文件保存在：

```text
E:\AAA__Github_Project\One-Arm-Teleoperation\recordings\<时间_名称>\
```

### 暂停点 A

发送：

1. 两条命令的完整终端输出；
2. 新生成的 `frames.csv`；
3. 新生成的 `metadata.json`。

如果不是 8/8 完整回复，不继续。

## 2. Ubuntu 通过 GitHub 获取代码

电脑：Armstrong 的 Ubuntu 上位机

首次下载：

```bash
mkdir -p ~/onearm_teleop
cd ~/onearm_teleop
git clone https://github.com/jason0925pig-rgb/One-Arm-Teleoperation.git
cd One-Arm-Teleoperation
git status
git log -1 --oneline
```

以后 Windows 推送了修改，Ubuntu 在工作区干净时更新：

```bash
cd ~/onearm_teleop/One-Arm-Teleoperation
git pull --ff-only origin main
```

不要用 U 盘覆盖仓库中的部分文件，也不要同时修改两台电脑上的同一个配置文件。

## 3. Ubuntu 预检与构建

先运行只读预检：

```bash
cd ~/onearm_teleop/One-Arm-Teleoperation
bash tools/ubuntu_preflight.sh | tee ubuntu_preflight_report.txt

source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true
bash tools/ubuntu_conflict_check.sh | tee ubuntu_conflict_report.txt
```

`ubuntu_conflict_check.sh` 只读取进程、ROS2 节点、命令话题发布者和夹爪串口占用；
它不会停止任何程序。若结果为 `CONFLICT/ACTIVITY FOUND`，不要启动本项目节点，
先让正在使用 Pi0/旧控制器的人结束任务并确认。

确认实际 ROS2 版本：

```bash
ls /opt/ros
uname -m
```

当前上位机已经确认是 Ubuntu 24.04、ROS2 Jazzy、x86_64，项目路径是
`/home/tele/onearm_teleop/One-Arm-Teleoperation`，直接执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/onearm_teleop/One-Arm-Teleoperation

sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep libmodbus-dev

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

rosdep install \
  --from-paths servo_controller one_arm_teleop_bridge \
  --ignore-src -r -y

set -o pipefail
colcon build --symlink-install \
  --packages-select servo_controller one_arm_teleop_bridge \
  --cmake-args \
    -DBUILD_LEGACY_DUAL_ARM_CONTROLLER=OFF \
    -DBUILD_LEGACY_TEST_EXECUTABLES=OFF \
  2>&1 | tee build_report.txt

source install/setup.bash
ros2 pkg executables servo_controller
ros2 pkg executables one_arm_teleop_bridge
```

期望看到：

- `servo_controller safe_one_arm_servo`
- `servo_controller safe_gripper_controller`
- `servo_controller gripper_feedback_probe`
- `one_arm_teleop_bridge udp_leader_bridge`

### 暂停点 B

发送：

1. `ubuntu_preflight_report.txt`
2. `ubuntu_conflict_report.txt`
3. `build_report.txt`
4. `ls /opt/ros` 和 `uname -m` 输出
5. 两条 `ros2 pkg executables` 输出

构建有红色错误就先停下。

## 4. 阶段一：只验证 Windows → Ubuntu 接收

这一阶段不启动 Armstrong 执行节点，不登录机器人。

先检查两台电脑都已自动同步时间。Windows PowerShell：

```powershell
w32tm /query /status
Get-Date -Format o
```

Ubuntu：

```bash
timedatectl show -p NTPSynchronized -p TimeUSec
date --iso-8601=ns
```

真实运动模式会拒绝超过 `max_packet_age_seconds` 的旧包和时间明显来自未来的包；
若两台电脑时间相差超过约 0.25 秒，先修复系统时间同步，不要放宽检查来绕过。

Ubuntu 终端 U1：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/onearm_teleop/One-Arm-Teleoperation
source install/setup.bash
ros2 launch one_arm_teleop_bridge udp_leader_bridge.launch.py
```

Ubuntu 终端 U2：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/onearm_teleop/One-Arm-Teleoperation
source install/setup.bash
ros2 topic echo /teleop/leader_pulses
```

Windows PowerShell，把 `<UBUNTU_IP>` 换成 Windows 能访问的 Ubuntu 地址：

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation

.\tools\run_zlink2_recorder.cmd `
  --port COM10 `
  --rate-hz 15 `
  --udp-target <UBUNTU_IP>:5005 `
  --duration 15 `
  --session-name udp_receive_test
```

此时 Ubuntu 只会发布 `/teleop/leader_pulses`。桥接配置仍是：

```yaml
dry_run: true
calibration_complete: false
```

所以它不会发布 `/right_arm/teleop_joint_command`，更没有执行节点可以驱动机器人。

### 暂停点 C

发送：

1. Windows 15 秒输出；
2. Ubuntu `/teleop/leader_pulses` 的连续 10 帧；
3. 以下输出：

```bash
ros2 topic echo --once /teleop/bridge_status
ros2 topic hz /teleop/leader_pulses
```

`topic hz` 看到稳定频率后按 `Ctrl+C`。

## 5. 阶段二：只读登录 Armstrong 右臂

先停止阶段一的节点。确认机器人周围无人、急停可用。

### 5.1 建立独立的机器人有线网络

Ubuntu 的两个网口用途不同：

```text
wlo1：Wi-Fi，连接 Windows、GitHub 和互联网
enp86s0：有线网口，只连接 Armstrong 控制器
```

使用一根普通 Cat5e/Cat6、两端都是 RJ45 的以太网线。一端插 Ubuntu 电脑的
有线网口，另一端插 Armstrong 控制柜标明的 LAN/调试网口，或者机器人随附交换机。
不要插入舵机总线、48 V、PoE 或用途不明的接口；不确定控制柜端口时先让老师确认。

机器人控制器必须供电，网口才会建立链路并允许 SDK 登录；此时不要求机器人电机
上电或使能。插线后检查：

```bash
ip -br link show enp86s0
nmcli device status
```

如果仍显示 `DOWN` 或 `disconnected`，先检查控制器供电、控制柜端口和网线，不启动
机器人节点。

控制器地址是 `192.168.2.226`，因此 Ubuntu 有线口必须使用同一 `/24` 网段内的
另一个未占用地址。当前测试建议使用 `192.168.2.10/24`；绝不能把 Ubuntu 也设置成
`192.168.2.226`。如果现场老师指定了其他上位机地址，以现场分配为准。

先查看 NetworkManager 中是否已有绑定到 `enp86s0` 的连接：

```bash
nmcli -t -f NAME,DEVICE connection show
```

如果存在，例如 `Wired connection 1:enp86s0`，执行：

```bash
sudo nmcli connection modify "Wired connection 1" \
  ipv4.method manual \
  ipv4.addresses 192.168.2.10/24 \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  ipv6.method disabled
sudo nmcli connection up "Wired connection 1"
```

把 `Wired connection 1` 换成上一步显示的实际名称。如果没有任何连接绑定
`enp86s0`，执行：

```bash
sudo nmcli connection add \
  type ethernet \
  ifname enp86s0 \
  con-name armstrong-wired \
  ipv4.method manual \
  ipv4.addresses 192.168.2.10/24 \
  ipv4.never-default yes \
  ipv6.method disabled
sudo nmcli connection up armstrong-wired
```

不要给这条有线连接填写网关或 DNS，这样 Wi-Fi 仍负责 Windows/GitHub/互联网，
有线口只负责 `192.168.2.0/24` 机器人网络。验证：

```bash
ip -br addr show enp86s0
ip route get 192.168.2.226
ping -c 4 192.168.2.226
```

期望看到：

- `enp86s0` 为 `UP`，地址是 `192.168.2.10/24`；
- 路由结果包含 `dev enp86s0 src 192.168.2.10`；
- 四次 ping 能收到回复。

### 暂停点 D1：有线网络

发送：

1. `nmcli device status`；
2. `ip -br addr`；
3. `ip route get 192.168.2.226`；
4. `ping -c 4 192.168.2.226`。

ping 不通就停在这里，不运行 SDK 节点。

### 5.2 启动只读右臂节点

机器人控制器必须供电才能登录 SDK，但下面的程序不会自动给电机上电或使能。

Ubuntu 终端 U1：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/onearm_teleop/One-Arm-Teleoperation
source install/setup.bash

ros2 run servo_controller safe_one_arm_servo --ros-args \
  --params-file "$HOME/onearm_teleop/One-Arm-Teleoperation/servo_controller/config/safe_one_arm.yaml" \
  -p dry_run:=false \
  -p hardware_power_authorized:=false \
  -p hardware_motion_authorized:=false \
  -p limits_configured:=false \
  -p power_on_on_arm:=false \
  -p enable_robot_on_arm:=false
```

这个节点只会：

- 登录配置中的右臂 `192.168.2.226:10020`；
- 通过 JAKA `get_robot_status` 周期读取七个真实关节角，并同时读取控制器的
  上电、使能、急停、碰撞保护、软限位和连接状态；
- 发布 `/right_arm/joint_states` 和安全状态。

它不会：

- `power_on`
- `enable_robot`
- 进入伺服模式
- 接受运动解锁

即使此时有人向关节命令 topic 发消息，`motion_enabled` 仍为 false，消息会被忽略；
而且 `hardware_motion_authorized` 和 `limits_configured` 两个门也都关闭。

Ubuntu 终端 U2：

```bash
source /opt/ros/jazzy/setup.bash
source ~/onearm_teleop/One-Arm-Teleoperation/install/setup.bash

ros2 topic echo --once /right_arm/joint_states
ros2 topic echo --once /right_arm/safety_status
ros2 topic echo --once /right_arm/powered_on
ros2 topic echo --once /right_arm/motion_enabled
```

七个关节角不能在未核对机器人实际位姿时直接接受为全零。`safety_status` 必须包含
`feedback_valid=1`，并核对 `feedback_source=get_robot_status`、控制器状态字段与
现场界面一致。节点首次成功读取时，U1 还会打印 `joint_position=[...]` 以及旧
`get_joint_position` 接口的对照结果。

### 暂停点 D2：只读 SDK 登录

发送：

1. U1 从启动到登录成功/失败的完整输出；
2. 三个 `--once` 的输出；
3. Armstrong 控制器的实际 IP 和端口。

不要调用 `/right_arm/set_motion_enabled`。

### 5.3 只给右臂驱动上电，不使能、不运动

仅当现场人员同意、机械臂周围清空、急停可立即触及时执行。该阶段不要求七轴
限位已经配置，因为程序不会开启运动门。

先用 `Ctrl+C` 停止 U1 的只读节点，再重新启动；这一轮只打开独立的上电授权：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/onearm_teleop/One-Arm-Teleoperation
source install/setup.bash

ros2 run servo_controller safe_one_arm_servo --ros-args \
  --params-file "$HOME/onearm_teleop/One-Arm-Teleoperation/servo_controller/config/safe_one_arm.yaml" \
  -p dry_run:=false \
  -p hardware_power_authorized:=true \
  -p hardware_enable_authorized:=false \
  -p hardware_motion_authorized:=false \
  -p limits_configured:=false \
  -p power_on_on_arm:=false \
  -p enable_robot_on_arm:=false
```

节点启动本身仍然只登录和读取。U2 先确认服务存在及状态安全：

```bash
source /opt/ros/jazzy/setup.bash
source ~/onearm_teleop/One-Arm-Teleoperation/install/setup.bash

ros2 service list | grep /right_arm/set_powered_on
ros2 service list | grep /right_arm/set_robot_enabled
ros2 topic echo --once /right_arm/safety_status
```

只有状态同时满足以下条件时，服务才允许调用 JAKA `power_on()`：

- `feedback_valid=1`
- `robot_socket_connected=1`
- `robot_enabled=0`
- `robot_emergency_stop=0`
- `robot_protective_stop=0`
- `robot_error_code=0`
- `motion_enabled=0`

确认后只调用上电服务：

```bash
ros2 service call /right_arm/set_powered_on \
  std_srvs/srv/SetBool "{data: true}"
```

该服务不会调用 `clear_error`、`enable_robot`、`servo_move_enable` 或任何关节运动
接口。等待一秒后检查：

```bash
ros2 topic echo --once /right_arm/powered_on
ros2 topic echo --once /right_arm/safety_status
ros2 topic echo --once /right_arm/joint_states
ros2 topic echo --once /right_arm/motion_enabled
```

预期 `robot_powered_on=1`、`robot_enabled=0`、`motion_enabled=0`，七个关节位置应
与机器人实际姿态一致。检查结束后，在机器人仍未使能时软件下电：

```bash
ros2 service call /right_arm/set_powered_on \
  std_srvs/srv/SetBool "{data: false}"
```

如果机器人已经使能或运动门已打开，程序会拒绝直接 `power_off`，避免突然掉电。

### 暂停点 D3：仅上电反馈

发送上电服务响应、四个 `--once` 输出和 U1 新日志。不要调用
`/right_arm/set_robot_enabled` 或 `/right_arm/set_motion_enabled`。

## 6. 得到真机参数

在允许目标预览前还需要：

- Armstrong 右臂七个真实关节上下限；
- 七个 `sign`；
- 七个 `scale_rad_per_pulse`；
- CTAG2F120 串口和安全开/闭位置。

### 6.1 从 URDF 提取七轴限位

拿到 URDF 或展开后的 xacro 后：

```bash
python3 tools/extract_urdf_joint_limits.py /path/to/armstrong.urdf
```

再按控制器 J1→J7 顺序使用七个 `--joint` 重新执行。不要用程序撞限位。

### 6.2 计算主从方向和比例

Windows 每次只测一个主臂关节，记录起点、终点 pulse 和实际转角：

```powershell
.\tools\run_calibration_calculator.cmd `
  --joint 1 `
  --start-pulse 1000 `
  --end-pulse 1600 `
  --follower-delta-deg 90
```

对七个关节分别计算。示例数字不能直接写进配置。

### 6.3 CTAG2F120 只读反馈

```bash
ros2 run servo_controller gripper_feedback_probe --ros-args \
  -p arm_name:=right \
  -p gripper_type:=zx \
  -p gripper_model:=CTAG2F120 \
  -p port:=/dev/serial/by-id/<ACTUAL_DEVICE> \
  -p slave_id:=1 \
  -p baudrate:=115200 \
  -p connect_hardware:=true
```

探针只有反馈，没有运动订阅器。使用厂家界面低速移动到任务安全的开/闭位置，
分别记录反馈；不要自动搜索物理极限。

### 暂停点 E

发送七轴限位、七组比例计算结果、URDF，以及夹爪两端反馈。由我统一回填并复核
两个 YAML。

## 7. 阶段三：目标预览，仍然不动

回填参数后：

```yaml
# one_arm_teleop_bridge/config/teleop_bridge.yaml
dry_run: true
calibration_complete: true

# servo_controller/config/safe_one_arm.yaml
dry_run: false
limits_configured: true
hardware_power_authorized: false
hardware_enable_authorized: false
hardware_motion_authorized: false
```

分别启动只读执行节点和 UDP 桥接，然后查看：

```bash
ros2 topic echo /teleop/target_preview
ros2 topic echo /right_arm/joint_states
```

确认主臂与从臂都处于舒适起始位姿后，只解锁“映射预览”：

```bash
ros2 service call /teleop/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

此服务会同时记录主臂起点 `p0` 和从臂起点 `q0`：

```text
q_target = q0 + sign × scale × (p - p0)
```

因为桥接仍为 `dry_run: true`，预览目标不会发布到机器人命令 topic。

### 暂停点 F

缓慢转动每个主臂关节，发送 `/teleop/target_preview` 和
`/right_arm/joint_states` 的对照结果。方向、幅度或限位有任何异常都不进入运动。

## 8. 阶段四：首次低速真机运动

只有暂停点 F 全部通过、现场急停可用、老师同意后才修改：

```yaml
# one_arm_teleop_bridge/config/teleop_bridge.yaml
dry_run: false
expected_source_ip: "<WINDOWS_IP>"
require_expected_source_ip_for_motion: true
require_deadman_for_motion: true
enforce_packet_timestamps_for_motion: true

# servo_controller/config/safe_one_arm.yaml
hardware_power_authorized: true
hardware_enable_authorized: true
hardware_motion_authorized: true
```

首测继续保持：

```yaml
power_on_on_arm: false
enable_robot_on_arm: false
max_velocity_rad_s: [0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10]
max_acceleration_rad_s2: [0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20]
control_rate_hz: 125.0
control_deadline_abort_seconds: 0.05
```

先在 Windows 启动带 deadman 的发送器；此命令不会授权机器人运动：

```powershell
.\tools\run_zlink2_recorder.cmd `
  --port COM10 `
  --rate-hz 15 `
  --udp-target <UBUNTU_IP>:5005 `
  --deadman `
  --session-name first_live_test
```

按住 Space 才会发送 `deadman_held=true`。松开 Space、按 Esc、按 Ctrl+C、
程序异常或正常结束都会重复发送 STOP；Ubuntu 收到 STOP 会立即关闭映射门和
伺服门，但不会代替实体急停。

确认冲突预检无活动、现场急停可用后，启动节点。按现场流程依次人工打开四个门；
每个服务只做它名字对应的一件事，绝不自动清错误：

```bash
# 第一层：只上电
ros2 service call /right_arm/set_powered_on \
  std_srvs/srv/SetBool "{data: true}"

# 第二层：只使能机器人
ros2 service call /right_arm/set_robot_enabled \
  std_srvs/srv/SetBool "{data: true}"

# 第三层：进入伺服模式
ros2 service call /right_arm/set_motion_enabled \
  std_srvs/srv/SetBool "{data: true}"

# 第四层：按当前主从姿态重新采集 p0/q0，并允许桥接发布目标
# 调用这一条时 Windows Space 必须正被按住。
ros2 service call /teleop/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

首次只转一个主臂关节很小角度，立即停下核对；不要七轴同时大幅移动。

停止时先松开 Space（立即 STOP），再显式关门、取消使能；确认机器人状态后才下电：

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

数据超过 300 ms 未更新、来源 IP 不符、数据包过旧/来自未来、deadman 松开、
越界、NaN、关节状态超时、8 ms 循环严重超期或 JAKA SDK 返回错误时，节点会
发布/响应 STOP 并关闭运动门。`/right_arm/safety_status` 中会持续给出
`control_deadline_misses`、`last_control_period_s` 和 `max_control_period_s`。

SDK/控制器网络断开时不会自动重连、自动上电或恢复运动。节点会先失败停止。
只有已从官方界面确认机器人断电、未使能且运动门关闭时，才可人工调用：

```bash
ros2 service call /right_arm/reconnect std_srvs/srv/Trigger "{}"
```

若服务拒绝或 `login_out` 失败，保持实体急停可用并重启节点；不要循环调用重连。

## 8.5 正式采集时统一记录 ROS2 数据

只有低速真机测试通过以后才开始数据集采集。先列出实际相机话题：

```bash
ros2 topic list | grep -Ei 'image|camera|depth'
```

下面的被动记录器不会发布控制消息。用 `--extra-topic` 逐个加入真实 RGB/深度话题，
并用 `--require-topic` 要求关键话题必须存在，否则拒绝开始：

```bash
python3 tools/ros2_episode_recorder.py \
  --name pick_cup_001 \
  --task "拿起杯子并放到托盘" \
  --operator Lucky \
  --require-topic /right_arm/joint_states \
  --require-topic /right_arm/safety_status \
  --extra-topic /ACTUAL_RGB_TOPIC \
  --extra-topic /ACTUAL_DEPTH_TOPIC
```

每个 episode 保存 ROS bag 和 `episode_metadata.json`，其中包含 Git 提交、开始/结束
UTC、操作者、任务、成功/失败标签、停止原因、话题清单和 rosbag 返回码。默认记录
主臂原始/滤波值、目标预览、实际命令、从臂状态、夹爪、安全状态、STOP 和各安全门。
Windows `frames.csv` 仍单独保留，并通过已同步的系统时间戳与 ROS bag 对齐。

## 9. GitHub 日常同步

Windows 修改并验证后：

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation
git status
git add <明确的文件>
git commit -m "描述本次修改"
git push origin main
```

Ubuntu 拉取：

```bash
cd ~/onearm_teleop/One-Arm-Teleoperation
git status
git pull --ff-only origin main
colcon build --symlink-install \
  --packages-select servo_controller one_arm_teleop_bridge
source install/setup.bash
```

`recordings/`、构建目录和机器检查报告不会上传 GitHub。机械 STL/STEP、代码、
配置和说明文档会正常同步。
