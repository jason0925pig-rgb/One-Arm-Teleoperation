# Armstrong 遥操作数据集工作流

这套流程把 Armstrong 右臂、CTAG2F120 夹爪、头部相机和右腕相机记录为
LeRobot Dataset v3，可用于 SmolVLA 等策略的训练。录制与导出程序均为被动订阅者，
不发布机器人运动命令；是否运动仍由原有安全遥操作栈控制。

## 硬件与频率

- 头部 RGB：Orbbec `CPCD7530003J`
- 右腕 RGB：Orbbec `CPCBC5300077`
- 两路均为 `1280x720 @ 30 FPS`
- 两个胸部相机明确排除，不启动也不录制
- JAKA 伺服循环：8 ms（125 Hz）
- LeRobot 固定帧率：30 FPS

现场在 2026-07-31 对两路压缩图像实测为约 30.20 FPS 和 30.04 FPS。旧体感
程序同样以 30 FPS 写 RGB H.264 MP4，并把 JAKA 数据按约 8 ms 记录，因此本项目
沿用其“相机 30 FPS、控制 125 Hz、按时间戳同步”的原则，但不沿用旧目录结构。

当前只记录 RGB，不记录深度。旧程序的单路深度 H5 在几分钟内就会达到数 GB，
而当前 SmolVLA 任务不需要深度；贸然加入会显著缩短 SSD 可录制时间。

## LeRobot v3 结构

使用官方 `LeRobotDataset.create/resume/add_frame/save_episode/finalize` API 写入，
不要手动改 Parquet、MP4、任务表或 episode 编号。v3 是多 episode 合并到文件
分片的结构，不是 v2.1 的“每个 episode 一个文件”：

```text
onearm_Tele/
├── raw_episodes/
│   └── 20260731_103000_full_teleop/
│       ├── episode_metadata.json
│       ├── rosbag/
│       └── lerobot_export_report.json
└── lerobot_dataset/
    ├── meta/
    │   ├── info.json
    │   ├── stats.json
    │   ├── tasks.parquet
    │   └── episodes/chunk-000/file-000.parquet
    ├── data/chunk-000/file-000.parquet
    └── videos/
        ├── observation.images.head/chunk-000/file-000.mp4
        └── observation.images.wrist_right/chunk-000/file-000.mp4
```

原始 ROS bag 按日期时间命名，便于追溯和重新导出。正式 LeRobot episode index
由官方 API 管理。

## 每帧特征

| 特征 | 维度 | 来源 |
| --- | ---: | --- |
| `observation.state` | 8 | 7 个真实右臂关节角（rad）+ 夹爪二值状态 |
| `action` | 8 | 7 个 SDK 已接受的限速后目标（rad）+ 已接受夹爪命令 |
| `observation.images.head` | RGB | 头部相机 |
| `observation.images.wrist_right` | RGB | 右腕相机 |
| `observation.gripper_contact` | 1 | 夹爪 torque/contact 状态 |
| `observation.gripper_feedback_valid` | 1 | 夹爪位置反馈是否可信 |

夹爪约定为 `0.0=打开`、`1.0=闭合`。这表示命令状态，不要求夹到物体后仍压到
机械端点；夹到物体触发 torque/contact 后停止是正常样本。

导出器在统一的 30 FPS 时间格上取样：

- 相机和真实关节使用最近样本；
- action 只允许使用该帧时刻之前已经被 SDK 接受的目标，不借用未来命令；
- 二值状态使用最近历史值；
- 相机偏差超过 50 ms、action 过期超过 20 ms、重复帧超过 5% 或 episode
  少于 30 帧时拒绝导出。

每次都会生成 `lerobot_export_report.json`，记录时间偏差与重复帧比例。

## SSD

目标数据盘目录固定为：

```text
/media/tele/f05c1455-ef49-4879-9332-d6cf5c5557c4/onearm_Tele
```

启动器会在任何机器人上电/使能之前检查：

- 该路径确实是已挂载、可写的文件系统；
- 当前用户至少还有 10 GiB 可用；
- 两个指定 Orbbec 序列号存在；
- 只有头部和右腕相机进入话题清单；
- 两路压缩 RGB 均通过 30 FPS 实测。

若检查失败，遥操作不会进入上电阶段。

## 一次性安装 LeRobot 环境

Ubuntu 拉取并构建新代码后执行一次：

```bash
cd ~/onearm_teleop/One-Arm-Teleoperation
bash tools/setup_lerobot_ubuntu.sh
```

默认安装 LeRobot 0.6.0 到 `/home/tele/.venvs/onearm-lerobot`，不上传 Hugging Face。

## 一条命令完成采集

在 Windows PowerShell：

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation
.\tools\start_full_teleop.cmd
```

流程如下：

1. 输入本次任务提示词，原文写入 LeRobot。
2. Ubuntu 检查 SSD，启动且只启动头部/右腕相机，实测 30 FPS。
3. 启动机器人 ROS2 接口，此时不运动。
4. 在 SSD 开始被动 ROS bag 录制。
5. Windows 主臂窗口按 Enter 捕获带起始偏移的基线。
6. 主窗口显示安全栈 ready 后，主臂窗口按 Space 开始遥操作。
7. 再按 Space、Esc 或 Ctrl+C 停止。
8. 启动器先关闭映射、伺服、使能与电源，再停止录制和相机。
9. 输入 `S` 标记成功并自动校验、追加到 LeRobot v3；输入 `F` 标记失败，
   原始 bag 保留但不进入行为克隆训练集。

也可以把提示词直接放在命令参数里，避免临场输入差异：

```powershell
.\tools\start_full_teleop.cmd `
  -Task "把农夫山泉放进大纸箱" `
  -Operator "Lucky" `
  -SessionName "pick_water_bottle"
```

同一批训练数据应使用完全一致的任务原文，包括标点和空格。
