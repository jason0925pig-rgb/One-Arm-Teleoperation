# One-Arm-Teleoperation

Windows ZLink2 主臂到 Armstrong 右臂的安全 ROS2 遥操作项目。

当前确定的硬件与方案：

- Windows：ZLink2 V2.1.9，读取 7 个关节电机和 1 个夹爪电机；
- Ubuntu：ROS2 桥接、主从映射、安全检查和 JAKA/Armstrong 执行；
- 从臂：Armstrong 右臂，7 个关节；
- 夹爪：知行机器人（ChangingTek）CTAG2F120；
- 控制方式：带起始偏移的绝对关节控制；
- 主臂夹爪：带滞回的 `OPEN/CLOSED` 状态机；
- 代码传输：GitHub。

## 数据链路

```text
ZLink2 主臂
    │  8 路 PRAD 只读查询，完整扫描实测约 58 ms
    ▼
Windows 采集器
    ├─ 保存 frames.csv + metadata.json
    ├─ 每得到一套完整数据就发送一个 UDP 包
    └─ Esc、Ctrl+C 或 deadman 松开时重复发送 STOP
                         │
                         ▼
Ubuntu ROS2 UDP 桥接
    ├─ 固定来源 IP、时间戳、序号、会话、超时和跳变检查
    ├─ 可配置中值/低通/死区滤波
    ├─ 带起始偏移的绝对映射
    ├─ 目标预览
    └─ deadman 按住且显式解锁后才发布控制目标
                         │
                         ▼
Armstrong 安全执行节点
    ├─ 上电、机器人使能、伺服运动三个独立服务
    ├─ 二次限位、NaN、限速、限加速度和双看门狗检查
    ├─ 断线立即失败停止；仅全断电/未使能时允许人工重连
    └─ 每 8 ms 调用一次 JAKA 伺服输出并统计周期抖动
```

Windows 不能每 8 ms 产生一套新的八电机数据：115200 波特率下实测满速约
17 Hz。8 ms 是 Ubuntu 执行端对 JAKA 的输出周期；两组主臂数据之间由执行端
限速和平滑推进。

## 默认安全状态

仓库默认配置无法驱动真机：

- `dry_run: true`
- `calibration_complete: false`
- `limits_configured: false`
- `hardware_power_authorized: false`
- `hardware_enable_authorized: false`
- `hardware_motion_authorized: false`
- Windows 来源 IP 尚未配置，真实映射会被拒绝
- 主从比例为 0，真实七轴限位尚未填写
- 夹爪 `configuration_complete: false`
- 运动门初始关闭

仅连接机器人、收到 UDP 或收到 ROS2 关节消息都不会运动。真机运动必须在参数
复核后，再由操作者依次调用上电、机器人使能、伺服运动和映射四个显式服务。
旧参数 `power_on_on_arm` 和 `enable_robot_on_arm` 均被忽略，不会自动操作硬件。

## 快速开始

严格按照 [FAST_START.md](FAST_START.md) 的暂停点执行。首次操作顺序是：

1. Windows 本地 CSV 采集；
2. GitHub 克隆到 Ubuntu；
3. Ubuntu 只接收并打印 UDP；
4. 只读登录 Armstrong，核对七轴反馈；
5. 通过独立服务只给右臂驱动上电，保持未使能并核对真实反馈；
6. 回填真实限位、方向、比例和夹爪端点；
7. 保持 `dry_run` 做目标预览；
8. 最后才授权并显式解锁低速单关节运动。

当前已完成与仍需真机测量的边界见
[docs/OFFLINE_SAFETY_STATUS.md](docs/OFFLINE_SAFETY_STATUS.md)。
旧 Axis Studio/Noitom 体感遥操作工程中可复用的限位、初始姿态、8 ms 时序、
CTAG2F120 寄存器和数据记录证据见
[docs/LEGACY_MOCAP_TELEOP_AUDIT.md](docs/LEGACY_MOCAP_TELEOP_AUDIT.md)。

## 目录

```text
mechanical/              保留的 STL/STEP 机械模型
tools/                   Windows ZLink2 读取、CSV、标定和 Ubuntu 预检工具
one_arm_teleop_bridge/   Windows UDP → ROS2 的安全桥接
servo_controller/        Armstrong 右臂和 CTAG2F120 ROS2 执行端
docs/                    原理与全流程说明
```

本仓库不再包含当前路线未使用的 ROS1/Catkin、LeRobot 机器人示例、xArm Python
SDK、ManiSkill 仿真镜像和演示 GIF。来源与重新获取方式见
[UPSTREAM_CONTENT.md](UPSTREAM_CONTENT.md)。

## 上游来源

主臂机械设计与原始通用遥操作项目来自
[MINT-SJTU/LeRobot-Anything-U-Arm](https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm)。
本仓库保留上游归属说明，并针对 Armstrong、Windows/ROS2 网络桥接和真机安全
门控进行适配。
