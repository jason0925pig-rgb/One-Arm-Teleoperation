# ZLink2 安全探测程序（Windows）

`zlink2_safe_probe.py` 用于通过 ZLink2 查询总线舵机是否在线。它只允许以下两条查询：

- `PVER`：读取固件版本
- `PRAD`：读取当前位置脉宽

程序没有运动、改 ID、校准、恢复出厂、扭矩锁定或扭矩释放命令。

## 运行

在项目根目录的 PowerShell 中执行：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools\zlink2_safe_probe.py `
  --port COM10 --baud 115200 --ids 0-7 --json zlink2_probe_result.json
```

只查看将要发送的命令、不打开串口：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools\zlink2_safe_probe.py `
  --ids 0-7 --dry-run
```

## 结果解释

- `REPLIED` 表示对应 ID 返回了版本或位置数据。
- `pulse` 是舵机返回的原始位置脉宽。
- `approx` 使用项目原代码中的 `500–2500 -> 0–270°` 公式估算，只是舵机轴绝对位置，不能直接当作机械臂关节零位角。
- 没有回复不等于舵机损坏，也可能是 ID、波特率、协议、供电或 DAT 接线不匹配。

