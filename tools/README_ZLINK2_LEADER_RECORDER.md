# Windows 主臂数据采集器

`zlink2_leader_recorder.py` 在 Windows 上读取 ZLink2 的 8 路位置反馈。默认只发送
`PRAD` 查询，不发送位置、速度、校准、ID 修改或复位命令。

## 1. 检查配置和串口

```powershell
.\tools\run_zlink2_recorder.cmd --validate-only
```

程序会验证 `zlink2_joint_id_map.json`，打印 `joint_1～joint_7` 和 `gripper`
对应的总线 ID，并列出当前串口。

## 2. 开始一次采集

```powershell
.\tools\run_zlink2_recorder.cmd --session-name pick_cup `
  --task "拿起杯子并放到托盘" `
  --operator "Lucky"
```

程序默认自动查找 CH340。需要指定串口时：

```powershell
.\tools\run_zlink2_recorder.cmd --port COM10 --session-name test
```

将主臂放到希望作为本次起始偏移的姿态，按 Enter 开始。按 `Esc` 或 `Ctrl+C`
正常结束；若配置了 UDP，程序结束前会重复发送 STOP。

如果关节有阻力，不要强拧。只有托住机械臂并理解重力下落风险后，才使用：

```powershell
.\tools\run_zlink2_recorder.cmd --release-torque
```

该选项仍会要求输入 `RELEASE` 才会发送 `PULK`。

## 3. 定时测试

记录 10 秒后自动结束：

```powershell
.\tools\run_zlink2_recorder.cmd --session-name ten_second_test `
  --duration 10
```

目标频率默认是 10 Hz，也可以调整：

```powershell
.\tools\run_zlink2_recorder.cmd --rate-hz 15
```

程序不会伪造目标频率。实际平均频率和未按时完成的扫描次数会写入元数据。

## 3.1 发送到 Ubuntu ROS2 桥接（可选）

指定 Ubuntu 地址和端口后，采集器在保留本地 CSV 的同时发送完整主臂帧：

```powershell
.\tools\run_zlink2_recorder.cmd `
  --udp-target 192.168.50.2:5005 `
  --session-name network_dry_run
```

不提供 `--udp-target` 时网络功能完全关闭。UDP 包只包含只读主臂位置和
夹爪状态，发送端没有机器人上电、使能或运动能力。Ubuntu ROS2 桥接还需要
通过标定、安全检查和显式服务解锁后，才可能发布机器人目标。

真实遥操作必须加 `--deadman`：

```powershell
.\tools\run_zlink2_recorder.cmd `
  --port COM10 `
  --rate-hz 15 `
  --udp-target 192.168.0.36:5005 `
  --deadman `
  --session-name live_with_deadman
```

按住 Space 时帧内才有 `deadman_held=true`。一旦曾经按住后松开，独立键盘线程会
立即重复发送 5 个 STOP 包并结束采集；它不需要等待一次约 60～70 ms 的串口扫描
完成。可用 `--stop-repeat` 和 `--stop-interval` 调整冗余发送，但不能用它们代替
Ubuntu 看门狗或实体急停。

## 4. 输出文件

每次运行会建立一个独立目录：

```text
recordings/
  20260723_153000_pick_cup/
    frames.csv
    metadata.json
```

### frames.csv

基础字段：

- `sequence`：从 0 开始的帧号。
- `timestamp_unix_ns`：UTC Unix 纳秒时间戳，供以后同步相机和 Armstrong 状态。
- `monotonic_ns`：Windows 单调时钟时间戳。
- `elapsed_s`：相对本次采集开始的秒数。
- `scan_duration_ms`：读完 8 路所需时间。
- `complete`：8 路全部收到回复时为 1，否则为 0。
- `reply_count`：本帧实际收到的 ID 数。

每个关节包含：

- `joint_N_pulse`：原始位置值，是目前最重要、最可信的数据。
- `joint_N_delta_pulse`：相对于按 Enter 开始时的变化量。
- `joint_N_position_deg_provisional`：沿用 500～2500 pulse 对应 270° 的临时换算。
- `joint_N_offset_rad_provisional`：临时弧度偏移，尚未应用方向、比例和限位。

夹爪字段：

- `gripper_pulse`：总线原始循环位置。
- `gripper_delta_pulse`：原始值相对启动基准的差值，仅供诊断。
- `gripper_unwrapped_pulse`：按照 2500 pulse 周期连续展开的位置。
- `gripper_normalized`：舒服闭合到舒服打开之间的诊断归一化值。
- `gripper_state`：`OPEN`、`CLOSED` 或尚未跨过阈值时的 `UNKNOWN`。
- `gripper_state_changed`：该帧完成状态切换时为 1。

标定参数保存在项目根目录的 `gripper_calibration.json`。当前状态机使用：

- 展开值 `<= 2650`：闭合。
- 展开值 `>= 3000`：打开。
- 中间区域：保持上一次状态。
- 新状态连续出现 3 帧才完成切换。

可以把历史录制离线送入同一状态机：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" `
  .\tools\zlink2_gripper_state.py `
  .\recordings\20260723_084332_gripper_gesture_calibration\frames.csv
```

归一化值用于观察和调试；发送给从臂的是离散的 `OPEN/CLOSED` 状态。

### metadata.json

包含：

- 任务、操作者和备注。
- 串口参数。
- 完整 ID 映射快照及 SHA-256。
- 夹爪标定快照、SHA-256 和当次实际使用的阈值。
- 本次启动基准 pulse。
- 请求/实际采样率。
- UDP deadman 是否开启、普通包/STOP 包数量和 STOP 发送错误。
- 完整帧、缺失帧和串口丢回复统计。
- 正常结束、人工停止或错误停止原因。

## 5. 与真机数据集的关系

当前文件是主臂动作源数据，还不是完整的 LeRobot/Armstrong 数据集。接入真机后，
Linux ROS2 适配器会按照两台电脑同步后的系统时钟对齐：

- Armstrong 实际关节状态。
- 实际执行目标。
- 夹爪状态。
- 相机帧索引。
- 限位、急停和通信状态。

因为原始 pulse 和纳秒时间戳已经保留，这些数据以后可以稳定同步和重新标定。
