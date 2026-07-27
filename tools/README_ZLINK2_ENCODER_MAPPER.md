# ZLink2 八路编码器读取与物理关节 ID 映射

适用于当前 Windows 电脑上的 `ZLink2 V2.1.9 / CH340 / COM10`。

程序默认只发送 `PRAD` 位置查询，不发送位置、速度、ID 修改、校准或复位命令。

## 运行环境

使用已经安装了 `pyserial` 的 PlatformIO Python：

```powershell
$pioPython = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
```

Windows 也提供了无需手动选择 Python 的启动器：

```powershell
.\tools\run_zlink2_mapper.cmd --mode once
```

脚本首行的 `/usr/bin/env python3` 是 Linux/macOS 的 shebang。Windows PowerShell
不能把它当作命令直接执行。本工作区已经配置 VS Code 忽略该 shebang，并使用
PlatformIO 的 Python。重新打开 VS Code 终端或执行一次 `Developer: Reload Window`
后，“Run Code”也会使用正确解释器。

## 查看一次八路编码器

```powershell
& $pioPython .\tools\zlink2_encoder_mapper.py --mode once
```

## 实时观察哪个 ID 正在转动

```powershell
& $pioPython .\tools\zlink2_encoder_mapper.py --mode watch
```

表格中的 `MOVING` 表示最近约 0.5 秒变化最大的 ID。按 `Ctrl+C` 停止。

## 按顺序建立物理关节映射

先约定从机械臂底座向末端依次转动 `joint_1` 到 `joint_7`，最后转动夹爪：

```powershell
& $pioPython .\tools\zlink2_encoder_mapper.py --mode map
```

每一步只转动终端提示的一个关节。程序会固定观察 5 秒，并累计每个 ID 在这 5 秒内
走过的总行程。因此遇到限位后反向转动也会继续累计，不要求始终朝同一个方向运动。
其他关节的微小抖动会被噪声阈值和运动量排名过滤。确认的映射会保存到：

```text
zlink2_joint_id_map.json
```

如果机械结构的关节名称已经确定，可以自定义：

```powershell
& $pioPython .\tools\zlink2_encoder_mapper.py --mode map `
  --labels "base_yaw,shoulder_pitch,arm_roll,elbow,wrist_roll,wrist_pitch,wrist_yaw,gripper"
```

识别使用三个可调阈值：

```powershell
& $pioPython .\tools\zlink2_encoder_mapper.py --mode map `
  --confirm-seconds 5 `
  --map-travel-threshold 150 `
  --noise-floor 2 `
  --dominance 1.5
```

- `--confirm-seconds 5`：每个关节固定观察 5 秒。
- `--map-travel-threshold 150`：5 秒累计行程至少达到 150 pulse。
- `--noise-floor 2`：单次读取变化不超过 2 pulse 时按编码器抖动忽略。
- `--dominance 1.5`：第一名累计行程至少是第二名的 1.5 倍。

如果转动约 90 度并来回一次，累计行程通常会远大于 150 pulse。仍然识别不到时，
可以把 `--map-travel-threshold` 调低到 `80`；误把抖动识别成运动时，可以提高到
`250`。不要通过快速或用力扭动关节来满足阈值。

## 扭矩释放安全说明

默认程序不会改变扭矩状态。如果关节有阻力，不要用手强拧。

仓库原代码使用 `PULK` 释放舵机扭矩。本工具只在显式添加
`--release-torque` 后提供该功能，而且还会要求在终端输入 `RELEASE`：

```powershell
& $pioPython .\tools\zlink2_encoder_mapper.py --mode map --release-torque
```

释放扭矩前必须托住所有连杆并清空机械臂周围空间，否则机械臂可能在重力作用下
突然下落、夹手或撞击桌面。某个关节如果仍有明显阻力，应立即停止，不要强行转动。

## 角度说明

`approx_deg` 沿用仓库中的临时换算：

```text
(pulse - 500) / 2000 * 270
```

它适合识别运动和初步记录，但在完成每个关节的零点、方向和实际限位标定前，
不能视作机械臂的真实关节角度。
