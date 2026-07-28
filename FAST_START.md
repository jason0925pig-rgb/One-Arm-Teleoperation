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
- 执行层 `hardware_motion_authorized: false`
- 执行层 `motion_enabled: false`
- 启动时不自动上电、不自动使能、不自动进入伺服模式
- 夹爪 `configuration_complete: false`

真机运动必须在参数复核后，再由操作者依次调用两个显式服务。只接收 UDP、
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
```

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
2. `build_report.txt`
3. `ls /opt/ros` 和 `uname -m` 输出
4. 两条 `ros2 pkg executables` 输出

构建有红色错误就先停下。

## 4. 阶段一：只验证 Windows → Ubuntu 接收

这一阶段不启动 Armstrong 执行节点，不登录机器人。

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

# servo_controller/config/safe_one_arm.yaml
hardware_motion_authorized: true
```

首测继续保持：

```yaml
power_on_on_arm: false
enable_robot_on_arm: false
max_velocity_rad_s: [0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10]
control_rate_hz: 125.0
```

机器人由官方界面按现场流程上电/使能。启动节点后，必须依次人工打开两个门：

```bash
# 第一层：允许执行节点进入伺服模式
ros2 service call /right_arm/set_motion_enabled \
  std_srvs/srv/SetBool "{data: true}"

# 第二层：重新采集 p0/q0，并允许桥接发布目标
ros2 service call /teleop/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

首次只转一个主臂关节很小角度，立即停下核对；不要七轴同时大幅移动。

停止时先关映射门，再关运动门：

```bash
ros2 service call /teleop/set_enabled \
  std_srvs/srv/SetBool "{data: false}"

ros2 service call /right_arm/set_motion_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

数据超过 300 ms 未更新、越界、NaN、关节状态超时或 JAKA SDK 返回错误时，
节点会自动关闭运动门。

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
