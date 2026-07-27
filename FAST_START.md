# One-Arm-Teleoperation：Windows 主臂 → Armstrong 右臂 Fast Start

这份文档只针对当前确定的方案：

- Windows 读取 ZLink2 主臂的 `7 个关节 + 1 个夹爪`；
- Windows 用 UDP 把原始绝对编码器数据发给 Ubuntu；
- Ubuntu 运行 ROS2、连接 Armstrong **右臂**；
- 使用“带起始偏移的绝对控制”；
- 夹爪使用带滞回的 `OPEN/CLOSED` 状态机；
- 所有真机动作必须经过限位、限速、看门狗和双重人工解锁。

在文档明确写着“暂停并发给我”时，先不要继续下一阶段。

## 0. 现在还缺什么

| 参数 | 正确来源 | 已有工具 | 是否能自动撞限位测量 |
| --- | --- | --- | --- |
| 右臂 7 个真实关节上下限 | Armstrong/JAKA 的 URDF、控制器配置或厂家手册 | `tools/extract_urdf_joint_limits.py` | **不能，也不允许** |
| 7 个 `sign` | 右臂正方向定义 + 主臂期望跟随方向 | `tools/calibration_calculator.py` | 不需要撞限位 |
| 7 个 `scale_rad_per_pulse` | 主臂编码器脉冲变化 + 实测转角 | `tools/calibration_calculator.py` | 不需要真机运动 |
| 左/右臂 | 现场确认 | 已确认写成 `right` | 已完成 |
| 夹爪型号 | 已确认：知行 ChangingTek CTAG2F120 | 现有 `ZX_gripper` 即知行驱动 | 已完成 |
| 夹爪安全开/闭位置 | 官方范围内由操作员选定的安全姿态 | `gripper_feedback_probe` 只读反馈 | **不自动冲击机械极限** |

JAKA SDK 的 `get_joint_position()` 只能读取当前位置，`is_on_limit()` 只能判断当前是否触限，不能返回 7 个上下限。因此限位必须来自机器人描述或厂家资料，不能靠程序驱动到头来猜。

夹爪已经确认是知行机器人（ChangingTek）`CTAG2F120`。官网给出的关键信息是
Modbus RTU/IO、最大夹持力 80 N、重复定位精度 ±0.03 mm。老师代码中的
`ZX_gripper` 是“知行/ZhiXing”的历史内部名称；旧代码曾使用位置
`0 / 6000 / 12000`，其中注释把 `12000` 当闭合、`6000` 当半开。但在完整
寄存器手册和真机反馈确认前，这些值只能作为排查线索，不能直接作为安全端点。

## 1. 两台电脑和网线应当怎样连接

推荐拓扑：

```text
ZLink2 + 主臂 ──USB── Windows
                       │
                 同一局域网/Wi-Fi
                       │ UDP 5005
                       ▼
                  Ubuntu/ROS2
                       │ 独立有线网卡
                       ▼
            Armstrong 右臂控制器 192.168.2.226
```

最省事的方式是：

- Windows 与 Ubuntu 通过同一个 Wi-Fi/路由器通信；
- Ubuntu 的有线网口单独连接机器人控制器；
- Ubuntu 有线网口设为 `192.168.2.x/24`，但不要在这个机器人专用网口设置默认网关；
- 当前老师配置中的右臂候选地址是 `192.168.2.226:10020`，仍需现场验证。

机器人控制柜/控制器必须供电才能建立 SDK 连接；“电机上电和使能”是另一件事。只读检查阶段不会由我们的节点给机械臂上电或使能。

## 2. 把当前代码送到 Ubuntu

本项目已选择通过 GitHub 在 Windows 与 Ubuntu 之间同步代码。SCP 仅作为网络受限时的备用方式。

### 方法 A：同网段用 SCP，只传运行所需文件

Ubuntu：

```bash
sudo apt update
sudo apt install openssh-server
sudo systemctl enable --now ssh
hostname -I
mkdir -p ~/One-Arm-Teleoperation
```

Windows PowerShell，把 `<UBUNTU_USER>` 和 `<UBUNTU_IP>` 换成实际值：

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation
scp -r .\one_arm_teleop_bridge .\servo_controller .\tools `
  .\gripper_calibration.json .\zlink2_joint_id_map.json .\FAST_START.md `
  <UBUNTU_USER>@<UBUNTU_IP>:~/One-Arm-Teleoperation/
```

### 方法 B：GitHub（本项目采用）

Windows 端把已验证的修改推送到指定仓库后，Ubuntu 执行：

```bash
git clone https://github.com/jason0925pig-rgb/One-Arm-Teleoperation.git
cd One-Arm-Teleoperation
```

以后同步修改：

```powershell
# Windows：提交并推送修改
git push origin main
```

```bash
# Ubuntu：在没有本地未提交修改时拉取
cd ~/One-Arm-Teleoperation
git pull --ff-only origin main
```

首版仓库保留机械臂的 CAD/STL/STEP 文件；上游约 591 MB、与当前实机遥操作无关的 ManiSkill 仿真资源不纳入首版，获取方式见 `UPSTREAM_CONTENT.md`。

## 3. 检查 Windows 主臂

电脑：**Windows**

机器人：**不需要连接或运动**

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation
.\tools\run_zlink2_recorder.cmd --validate-only
```

期望看到：

- 找到 CH340/ZLink2 对应 COM 口；
- `8/8 IDs replied`；
- `joint_1..joint_7` 和 `gripper` 的 ID 映射正确。

### 暂停点 A

把这条命令的完整终端输出发给我。若不是 `8/8`，不要继续。

## 4. 收集 Ubuntu、ROS2、网络和 USB 信息

电脑：**Ubuntu**

机器人：此阶段不使能、不运动

```bash
cd ~/One-Arm-Teleoperation
bash tools/ubuntu_preflight.sh | tee ubuntu_preflight_report.txt
```

它只检查系统、网卡、ROS2、JAKA 库、串口、相机和 URDF 文件，并 `ping` 右臂候选 IP；不会发送机器人或夹爪运动命令。

### 暂停点 B

把下面内容发给我：

1. `ubuntu_preflight_report.txt`；
2. 老师告诉你的 Armstrong 操作系统和 ROS2 版本（如果知道）；
3. 右臂控制器的实际 IP/端口（如果知道）；
4. CTAG2F120 的接线方式，以及是否配有知行官方调试软件/说明书。

这里重点确认：

- Ubuntu 是 `x86_64` 还是 `aarch64`；
- `/opt/ros` 下是什么版本；
- 是否能 `ping 192.168.2.226`；
- 夹爪对应哪个 `/dev/serial/by-id/...`；
- 是否已经有 Armstrong 的 URDF/xacro。

## 5. 在 Ubuntu 构建安全栈

只有暂停点 B 的系统架构和 ROS2 版本合理时再执行。

电脑：**Ubuntu**

先查看已安装的 ROS2：

```bash
ls /opt/ros
```

下面的 `<ROS_DISTRO>` 换成实际目录名，例如 Ubuntu 22.04 常见的是 `humble`，但不要仅凭猜测填写：

```bash
source /opt/ros/<ROS_DISTRO>/setup.bash
cd ~/One-Arm-Teleoperation

sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep libmodbus-dev

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

期望存在：

- `servo_controller safe_one_arm_servo`
- `servo_controller safe_gripper_controller`
- `servo_controller gripper_feedback_probe`
- `one_arm_teleop_bridge udp_leader_bridge`

不要构建或运行旧的 `robot_timer`。

### 暂停点 C

把 `build_report.txt` 和两个 `ros2 pkg executables` 的输出发给我。构建有任何红色错误都先停下。

## 6. 只验证 Windows → Ubuntu UDP，不连接真机运动

Ubuntu 终端 U1：

```bash
source /opt/ros/<ROS_DISTRO>/setup.bash
cd ~/One-Arm-Teleoperation
source install/setup.bash
ros2 launch one_arm_teleop_bridge full_safe_stack.launch.py
```

默认配置仍是：

- `dry_run: true`
- `calibration_complete: false`
- `limits_configured: false`
- 7 个 scale 为 `0`
- 夹爪已识别为 `gripper_type: zx`、`gripper_model: CTAG2F120`，但
  `configuration_complete: false`

因此它不会驱动真机。

Ubuntu 终端 U2：

```bash
source /opt/ros/<ROS_DISTRO>/setup.bash
cd ~/One-Arm-Teleoperation
source install/setup.bash
ros2 topic echo /teleop/leader_pulses
```

Windows PowerShell，把 `<UBUNTU_WIFI_OR_LAN_IP>` 换成暂停点 B 中 Windows 能访问的 Ubuntu 地址，不是机器人地址：

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation
.\tools\run_zlink2_recorder.cmd `
  --udp-target <UBUNTU_WIFI_OR_LAN_IP>:5005 `
  --duration 15 `
  --session-name network_dry_run
```

如果 Ubuntu 防火墙处于启用状态，才需要针对 Windows IP 放行 UDP 5005。

### 暂停点 D

把这些内容发给我：

1. Windows 15 秒测试的完整输出；
2. Ubuntu `/teleop/leader_pulses` 连续约 10 帧；
3. 下面命令的输出：

```bash
ros2 topic echo --once /teleop/bridge_status
```

## 7. 只读连接 Armstrong 右臂并读取 7 个实际角度

先在 U1 按 `Ctrl+C` 停止整套 dry-run。确认机械臂周围无人、急停可用。此步骤只登录并读状态，不调用运动解锁服务。

Ubuntu 终端 U1：

```bash
source /opt/ros/<ROS_DISTRO>/setup.bash
cd ~/One-Arm-Teleoperation
source install/setup.bash

ros2 run servo_controller safe_one_arm_servo --ros-args \
  --params-file "$HOME/One-Arm-Teleoperation/servo_controller/config/safe_one_arm.yaml" \
  -p dry_run:=false
```

这个节点启动时：

- 登录当前配置的右臂 `192.168.2.226:10020`；
- 不自动 `power_on`；
- 不自动 `enable_robot`；
- 不进入伺服运动模式；
- 因真实限位还没配置，运动服务仍被锁死。

Ubuntu 终端 U2：

```bash
source /opt/ros/<ROS_DISTRO>/setup.bash
cd ~/One-Arm-Teleoperation
source install/setup.bash
ros2 topic echo --once /right_arm/joint_states
ros2 topic echo --once /right_arm/safety_status
```

### 暂停点 E

把 U1 从启动到“登录成功/失败”的完整输出，以及两个 `--once` 输出发给我。不要调用 `/right_arm/set_motion_enabled`。

## 8. 得到 7 个真实关节限位

优先向老师索要以下任意一项：

- Armstrong 右臂的 `robot_description` ROS2 包；
- 完整 URDF/xacro；
- 控制器导出的 7 轴上下限；
- 准确机器人/关节模组型号对应的厂家手册。

如果拿到 xacro，先展开：

```bash
source /opt/ros/<ROS_DISTRO>/setup.bash
sudo apt install "ros-${ROS_DISTRO}-xacro"
ros2 run xacro xacro /path/to/armstrong.urdf.xacro \
  > /tmp/armstrong_expanded.urdf
```

先列出全部活动关节：

```bash
python3 tools/extract_urdf_joint_limits.py \
  /tmp/armstrong_expanded.urdf
```

确认名字后，按控制器 J1→J7 的顺序重复 `--joint`。下面只是格式示例，名字不能照抄：

```bash
python3 tools/extract_urdf_joint_limits.py \
  /tmp/armstrong_expanded.urdf \
  --joint right_joint1 \
  --joint right_joint2 \
  --joint right_joint3 \
  --joint right_joint4 \
  --joint right_joint5 \
  --joint right_joint6 \
  --joint right_joint7 \
  | tee right_arm_limits.txt
```

工具同时显示：

- URDF 原始机械限位；
- 向内留 5° 的首次测试软件限位候选；
- URDF 最大速度。

首次遥操仍保持代码中的 `0.10 rad/s`，不要直接套用厂家最大速度。

### 暂停点 F

把 URDF/xacro 文件和 `right_arm_limits.txt` 发给我。我会核对关节顺序，并回填两个 YAML；不要自己先设置 `limits_configured: true`。

## 9. 得到 7 个 sign 和 scale

这一阶段不用让 Armstrong 跟着主臂运动。

每次只测一个主臂关节：

1. 其他关节保持不动；
2. 记录起始 pulse；
3. 用量角器或可靠机械基准将该关节单方向转约 90°；
4. 不要转回来，保持终点；
5. 记录终点 pulse；
6. 对 7 个关节分别重复。

Windows 记录示例：

```powershell
.\tools\run_zlink2_recorder.cmd `
  --duration 8 `
  --session-name joint_1_scale
```

程序每秒会在终端显示 8 路 pulse，也会把完整数据保存到 `recordings/.../frames.csv`。

`sign` 不是电机固有参数，而是“主臂这样转时，从臂 q 应该增大还是减小”。用 Armstrong 官方界面把对应右臂关节以最低速度 `+Jog` 很小角度，观察机器人正方向；不要用我们的遥操代码试方向。

例如，主臂 J1 从 100 转到 725 pulse；希望右臂 J1 按控制器正方向变化 `+90°`：

```powershell
.\tools\run_calibration_calculator.cmd `
  --joint 1 `
  --start-pulse 100 `
  --end-pulse 725 `
  --follower-delta-deg 90
```

如果同一个主臂动作应该让右臂 q 减小，则写 `--follower-delta-deg -90`。工具会输出该关节的：

- `sign`
- `scale_rad_per_pulse`

90° 只是推荐测量幅度；命令中填写你实际测到的有符号角度。

### 暂停点 G

把 7 次的起始 pulse、终点 pulse、实际角度和计算器完整输出发给我。我会统一检查量级、方向和异常值，再写进 `teleop_bridge.yaml`。

## 10. 读取知行 CTAG2F120 的安全位置

型号已经确认：知行机器人（ChangingTek）`CTAG2F120`，代码内部驱动名称为
`zx`。配置已经写入这个型号，但 `configuration_complete` 仍为 `false`。

使用稳定的 `/dev/serial/by-id/...`，不要长期依赖可能变化的
`/dev/ttyUSB0`。只运行下面的反馈探针，不要同时运行
`safe_gripper_controller`。

CTAG2F120：

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

这个可执行文件没有命令订阅器，也没有运动服务，只读当前位置、到位、力矩到达
和报警状态。

用厂家界面/控制器把夹爪低速、低力移动到两个**任务安全位置**：

- 舒适且不会撞结构的打开位置；
- 安全闭合位置。

每个姿态分别执行：

```bash
ros2 topic echo --once /right_arm/gripper_probe_state
ros2 topic echo --once /right_arm/gripper_probe_status
```

不要用手硬掰减速夹爪，也不要用程序自动搜寻机械极限。我们需要的是安全工作端点，
不是一定要达到的物理极限。真实抓到物体时，知行驱动的 `torque_reached`
会让执行节点停止继续闭合。

### 暂停点 H

把以下信息发给我：

1. `/dev/serial/by-id` 的实际路径；
2. 安全打开位置的两条 topic 输出；
3. 安全闭合位置的两条 topic 输出；
4. 知行说明书中的寄存器表，以及速度、力和位置范围（如果有）。

我会再回填 `open_position`、`closed_position`、低速和低力参数。

## 11. 到这里代码完成到什么程度

完成的是“最小安全遥操链路”：

- Windows ZLink2 八通道读取与原始 CSV；
- Windows → Ubuntu UDP；
- ROS2 数据校验、序号/会话检查和 300 ms 看门狗；
- 右臂单臂模式；
- 带起始偏移的绝对映射；
- 双层关节限位、NaN 检查和逐关节限速；
- 双重人工解锁；
- 夹爪 OPEN/CLOSED 状态机、低速/低力接口和反馈停止；
- 只读关节状态与夹爪反馈工具。

还没有完成的是“真机交付验证和完整数据集产品”：

- Ubuntu 上首次真实编译；
- 实际 7 轴参数和夹爪参数回填；
- 每个关节单独的低速真机验证；
- 相机驱动、准确 topic 名和时间同步；
- rosbag 录制验收；
- 如最终需要 LeRobot 数据格式，还要增加 rosbag/CSV → LeRobot dataset 的转换与质量检查。

所以：控制代码骨架已经基本完成，但在暂停点 A–H 的硬件证据回来前，不能说真机已经调完。

## 12. 暂时不要运行的命令

在我核对完 A–H、现场急停可用、老师同意低速测试前，不要：

- 把两个 YAML 的 `dry_run` 改成 `false` 后直接启动整套系统；
- 设置 `calibration_complete: true`；
- 设置 `limits_configured: true`；
- 设置夹爪 `configuration_complete: true`；
- 调用 `/right_arm/set_motion_enabled`；
- 调用 `/teleop/set_enabled`；
- 运行旧 `robot_timer`。

后续首次真机测试会按 `J1 一小段 → 停止检查 → J2 一小段` 的顺序进行，不会一上来同时放开七个关节。
