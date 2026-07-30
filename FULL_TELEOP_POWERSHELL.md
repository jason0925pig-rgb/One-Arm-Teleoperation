# PowerShell 一键遥操作

这个入口用于当前已经确认的组合：

- Windows：ZLink2 主臂，默认 `COM10`
- Ubuntu：`tele@192.168.0.36`，SSH 别名 `armstrong-host`
- 从臂：Armstrong 右臂
- 夹爪：CTAG2F120

## 运行前

1. 确认没有其他 Pi0、旧体感遥操作、`robot_timer` 或夹爪节点正在使用机器人。
2. 打开机器人、解除实体急停，并始终把实体急停放在手边。
3. 接好 ZLink2 主臂并确认端口仍是 `COM10`。
4. Ubuntu 仓库必须已拉取当前版本并完成构建。

## 唯一启动命令

打开 Windows PowerShell：

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation
.\tools\start_full_teleop.cmd
```

如果 ZLink2 不是 `COM10`：

```powershell
.\tools\start_full_teleop.cmd -ComPort COM11
```

运行过程：

1. PowerShell 通过 SSH 启动 Ubuntu 三个节点。此时只连接和读取，机器人不会运动。
2. 弹出 ZLink2 发送窗口。把主臂放到舒服起始姿态，按一次 Enter。
3. Ubuntu 检查 UDP、右臂反馈、急停、错误、限位和节点冲突，然后依次上电、使能、进入伺服模式并捕获主从起始偏移。
4. 发送窗口显示 `REMOTE STACK READY` 后，按一次 Space 开始。
5. 再按一次 Space、Esc 或 Ctrl+C 会立即发送重复 STOP；PowerShell 随后退出伺服模式、关闭夹爪、关闭机器人使能并下电。

带起始偏移的绝对映射为：

```text
从臂目标 = 从臂起始角度 + sign × scale × (主臂当前脉冲 - 主臂起始脉冲)
```

## 软件紧急停止

若启动窗口卡住，可另开一个 PowerShell 执行：

```powershell
ssh armstrong-host "cd /home/tele/onearm_teleop/One-Arm-Teleoperation && bash tools/ubuntu_full_teleop_stack.sh stop"
```

软件 STOP 不能代替实体急停。

## 本次参数

- 夹爪：打开 `2000`，关闭 `12000`，速度 `75%`，力 `25%`
- Ubuntu 伺服输出：`125 Hz`（每 `8 ms`）
- Windows 完整八舵机扫描：硬件实测速率约 `17 Hz`，设置 `100 Hz` 只是请求上限
- 主臂滤波：3 帧中值、`alpha=0.45` 低通、2 脉冲死区
- 关节最大速度：`0.90 rad/s`，是原 `0.30 rad/s` 的三倍
- 关节最大加速度：保持 `0.60 rad/s²`
- Windows/UDP/关节命令看门狗：`3.0 s`
- 机器人反馈看门狗：仍为 `0.30 s`

注意：3 秒网络看门狗意味着极端断网情况下，软件最多可能等待约 3 秒才因命令超时退出；实体急停和 0.30 秒机器人反馈看门狗仍是独立保护。
